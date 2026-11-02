import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6.QtWidgets import QApplication
import transport
from protocol import ACK, INFO, HELLO, READ, RUN, STOP, SAMPLE, encode, Parser


class FakePort:
    def __init__(self):
        self.rx = bytearray()
        self.sent = []
        self.closed = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, count):
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, data):
        self.sent.append(data)
        return len(data)

    def close(self):
        self.closed = True


def setup_link(monkeypatch):
    app = QApplication.instance() or QApplication([])
    clock = [10.0]
    monkeypatch.setattr(transport.time, "monotonic", lambda: clock[0])
    link = transport.SerialLink()
    port = FakePort()
    link.port = port
    link.info_received = True
    link.last_rx = link.last_ping = clock[0]
    return app, link, port, clock


def test_retry_uses_same_sequence_and_stop_preempts(monkeypatch):
    app, link, port, clock = setup_link(monkeypatch)
    link.send(RUN)
    link.poll()
    first = port.sent[-1]
    clock[0] += .45
    link.poll()
    assert port.sent[-1] == first
    link.send(STOP)
    link.poll()
    op, seq, _ = Parser().feed(port.sent[-1])[0]
    assert op == STOP
    port.rx.extend(encode(ACK, seq, bytes([STOP, 0, 0])))
    link.poll()
    assert link.pending is None
    link.close()


def test_missing_info_is_retried_after_ack(monkeypatch):
    app, link, port, clock = setup_link(monkeypatch)
    link.send(READ)
    link.poll()
    frame = port.sent[-1]
    op, seq, _ = Parser().feed(frame)[0]
    port.rx.extend(encode(ACK, seq, bytes([op, 0, 0])))
    link.poll()
    assert link.pending is None
    clock[0] += .8
    link.poll()
    assert port.sent[-1] == frame
    link.close()


def test_disconnect_cancels_run_retries(monkeypatch):
    app, link, port, clock = setup_link(monkeypatch)
    link.send(RUN)
    link.poll()
    clock[0] += 3
    link.poll()
    assert link.port is None and port.closed
    assert not link.queue and link.pending is None
    assert Parser().feed(port.sent[-1])[0][0] == STOP
