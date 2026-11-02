"""Bounded, non-blocking serial reads with acknowledged commands and a heartbeat."""
from collections import deque
import time
import serial
from PySide6.QtCore import QObject, QTimer, Signal
from protocol import Parser, encode, ACK, HELLO, PING, STOP, INFO, READ, ERRORS


class SerialLink(QObject):
    frame = Signal(int, int, bytes)
    message = Signal(str)
    connection = Signal(bool)
    completed = Signal(int, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.port = None
        self.parser = Parser()
        self.queue = deque()
        self.pending = None
        self.seq = 0
        self.last_rx = 0.0
        self.last_ping = 0.0
        self.info_received = False
        self.awaiting_info = None
        self.timer = QTimer(self)
        self.timer.setInterval(5)
        self.timer.timeout.connect(self.poll)

    def open(self, name):
        self.close()
        try:
            self.port = serial.Serial(name, 115200, timeout=0, write_timeout=.05)
            self.port.reset_input_buffer()
        except (serial.SerialException, OSError) as exc:
            self.port = None
            self.message.emit(f"串口打开失败：{exc}")
            return
        self.parser = Parser()
        self.last_rx = self.last_ping = time.monotonic()
        self.info_received = False
        self.timer.start()
        self.connection.emit(True)
        self.send(HELLO)

    def close(self):
        self.timer.stop()
        self.queue.clear()
        self.pending = None
        self.awaiting_info = None
        if self.port:
            try:
                self.seq = (self.seq + 1) & 255
                self.port.write(encode(STOP, self.seq))
            except (serial.SerialException, OSError):
                pass
            self.port.close()
            self.port = None
            self.connection.emit(False)
        self.info_received = False

    def send(self, kind, payload=b""):
        if not self.port:
            return
        if kind == STOP:
            self.queue.clear()
            self.pending = None
            self.awaiting_info = None
        if len(self.queue) >= 16:
            self.message.emit("指令队列已满，请等待回执")
            return
        self.seq = (self.seq + 1) & 255
        self.queue.append([kind, self.seq, encode(kind, self.seq, payload), 0.0, 0])
        if kind in (HELLO, READ):
            self.awaiting_info = [kind, self.seq, time.monotonic(), 0]

    def poll(self):
        if not self.port:
            return
        try:
            now = time.monotonic()
            for kind, seq, payload in self.parser.feed(self.port.read(min(self.port.in_waiting, 8192))):
                self.last_rx = now
                if kind == INFO:
                    self.info_received = True
                    if self.awaiting_info and seq == self.awaiting_info[1]:
                        self.awaiting_info = None
                if kind == ACK and len(payload) == 3:
                    op, result, field = payload
                    if self.pending and (op, seq) == tuple(self.pending[:2]):
                        self.pending = None
                        if op != PING or result:
                            text = ERRORS.get(result, f"未知错误 {result}")
                            self.message.emit(f"指令 {op}：{text}" + (f"（字段 {field}）" if field else ""))
                        self.completed.emit(op, result, field)
                self.frame.emit(kind, seq, payload)
            if now - self.last_rx > 2.5:
                self.message.emit("通信中断：已断开；远程运行由固件 2 秒超时停机保护")
                self.close()
                return
            if self.awaiting_info and now - self.awaiting_info[2] > .7 and not self.pending and not self.queue:
                if self.awaiting_info[3] >= 2:
                    self.message.emit("参数回读超时，请重新连接")
                    self.close()
                    return
                op, seq = self.awaiting_info[:2]
                self.queue.append([op, seq, encode(op, seq), 0.0, 0])
                self.awaiting_info[2] = now
                self.awaiting_info[3] += 1
            # Initial HELLO also requests INFO. Repeat if INFO was lost (never starts the motor).
            if not self.info_received and now - self.last_ping >= .5:
                if not self.pending and not self.queue and not self.awaiting_info:
                    self.send(HELLO)
                self.last_ping = now
            elif self.info_received and now - self.last_ping >= .4:
                if not self.pending and not self.queue:
                    self.send(PING)
                self.last_ping = now
            if not self.pending and self.queue:
                self.pending = self.queue.popleft()
            if self.pending and (self.pending[4] == 0 or now - self.pending[3] >= .4):
                if self.pending[4] >= 3:
                    self.message.emit("指令回执超时：断开连接，不自动重启电机")
                    self.close()
                    return
                self.port.write(self.pending[2])
                self.pending[3] = now
                self.pending[4] += 1
        except (serial.SerialException, OSError) as exc:
            self.message.emit(f"串口异常：{exc}")
            self.close()
