"""FOC serial v4. Pure Python framing and gain calculation; no hardware access."""
from dataclasses import dataclass, fields
import math
import struct

MAGIC = b"\xa5\x5a"
HELLO, CONFIG, RUN, STOP, CLEAR, PING, READ, RATE = range(1, 9)
ACK, INFO, STATUS, SAMPLE = range(0x80, 0x84)
FAULT_EVENT = 0x84
MAX_PAYLOAD = 160
ERRORS = {
    0: "已接受", 1: "报文长度错误", 2: "参数超出范围", 3: "控制板忙",
    4: "请先停机再修改模式、带宽或滤波", 5: "启动/清故障条件不满足",
    6: "串口会话已失效，请重新连接",
}
MODES = ["待机 / DISABLED", "固定占空比 / PWM TEST", "电压 / VOLTAGE",
         "电流环 / CURRENT", "速度环 / SPEED", "位置环 / POSITION"]
CHANNELS = [("i_d", "A", "#ed3590"), ("i_q", "A", "#385eae"),
            ("转速（减速箱输出轴）", "rpm", "#1874ef"), ("位置（减速箱输出轴）", "°", "#e9a025"),
            ("机械角（电机转子）", "°", "#a540bf"),
            ("NTC 温度", "°C", "#159773")]
FLOAT_NAMES = ["id_a", "iq_a", "spd_rpm", "pos_deg", "ud_v", "uq_v", "open_speed",
               "iq_limit", "pos_speed_limit", "cur_bw", "spd_bw", "pos_bw",
               "spd_lpf", "pos_ki", "pos_kd", "duty_a", "duty_b", "duty_c", "prot_oc", "prot_ov", "prot_uv", "cur_lpf"]
LIMITS = list(zip(
    [-16, -16, -6000, 0, -40, -40, -10000, .05, .01, 10, 1, .1, 1, 0, 0, 0, 0, 0, .5, 13, 12, 0],
    [16, 16, 6000, 360, 40, 40, 10000, 16, 20, 500, 60, 20, 1000, 100, 10, 1, 1, 1, 16, 80, 60, 5000]))
CONFIG_STRUCT = struct.Struct("<4B22f")
STATUS_STRUCT = struct.Struct("<I8BHH10f3I")


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 0x8000 else 0)) & 0xFFFF
    return crc


def encode(kind: int, seq: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("Payload too large")
    body = struct.pack("<BBH", kind, seq, len(payload)) + payload
    return MAGIC + body + struct.pack("<H", crc16(body))


class Parser:
    def __init__(self):
        self.buffer = bytearray()
        self.crc_errors = 0

    def feed(self, data: bytes):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 2:
            start = self.buffer.find(MAGIC)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == MAGIC[:1] else b""
                break
            del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            kind, seq, size = struct.unpack_from("<BBH", self.buffer, 2)
            if size > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            if len(self.buffer) < size + 8:
                break
            body = bytes(self.buffer[2:6 + size])
            check = struct.unpack_from("<H", self.buffer, 6 + size)[0]
            if crc16(body) != check:
                self.crc_errors += 1
                del self.buffer[0]
                continue
            result.append((kind, seq, bytes(self.buffer[6:6 + size])))
            del self.buffer[:size + 8]
        return result


@dataclass
class Config:
    mode: int = 0
    open_angle: int = 0
    hold: int = 1
    id_a: float = 0
    iq_a: float = 0
    spd_rpm: float = 0
    pos_deg: float = 0
    ud_v: float = 0
    uq_v: float = 0
    open_speed: float = 200
    iq_limit: float = 16
    pos_speed_limit: float = .5
    cur_bw: float = 250
    spd_bw: float = 10
    pos_bw: float = 2
    spd_lpf: float = 150
    pos_ki: float = 0
    pos_kd: float = .3
    duty_a: float = .3
    duty_b: float = .4
    duty_c: float = .6

    prot_oc: float = 16
    prot_ov: float = 65
    prot_uv: float = 12
    cur_lpf: float = 1500

    def pack(self) -> bytes:
        if self.mode not in range(6) or self.open_angle not in (0, 1) or self.hold not in (0, 1):
            raise ValueError("非法模式或开关")
        values = [getattr(self, key) for key in FLOAT_NAMES]
        for name, value, (lo, hi) in zip(FLOAT_NAMES, values, LIMITS):
            if not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f"{name} 应在 {lo}～{hi} 范围内")
        if self.prot_uv >= self.prot_ov:
            raise ValueError("欠压阈值必须小于过压阈值")
        if self.iq_limit > self.prot_oc:
            raise ValueError("电流给定限幅不能大于相电流过流阈值")
        return CONFIG_STRUCT.pack(self.mode, self.open_angle, self.hold, 0, *values)

    @classmethod
    def unpack(cls, data: bytes):
        mode, angle, hold, _, *values = CONFIG_STRUCT.unpack(data)
        return cls(mode, angle, hold, **dict(zip(FLOAT_NAMES, values)))


@dataclass
class MotorParameters:
    resistance: float = .725
    inductance: float = .000410
    kt: float = .60
    inertia: float = .00030
    pole_pairs: float = 14
    gear_ratio: float = 8
    damping: float = 1
    fast_hz: float = 10000

    def gains(self, config: Config):
        w_cur = 2 * math.pi * config.cur_bw
        w_spd = 2 * math.pi * config.spd_bw
        return (w_cur * self.inductance, w_cur * self.resistance,
                2 * self.damping * w_spd * self.inertia / self.kt / self.pole_pairs,
                w_spd ** 2 * self.inertia / self.kt / self.pole_pairs,
                2 * math.pi * config.pos_bw, config.pos_ki, config.pos_kd)


def decode_info(payload: bytes):
    if len(payload) != 156:
        raise ValueError("固件协议不兼容，请烧录配套 v4 电流低通固件")
    major, minor, period = struct.unpack_from("<BBH", payload)
    if major != 4:
        raise ValueError(f"不支持的固件协议版本 {major}.{minor}")
    meta = MotorParameters(*struct.unpack_from("<8f", payload, 4))
    if not all(math.isfinite(getattr(meta, f.name)) and getattr(meta, f.name) > 0 for f in fields(meta)):
        raise ValueError("固件电机参数无效")
    config = Config.unpack(payload[36:128])
    gains = struct.unpack_from("<7f", payload, 128)
    return meta, config, gains, period


def decode_status(payload: bytes):
    values = STATUS_STRUCT.unpack(payload)
    names = ["ms", "state", "mode", "run", "align", "offset", "encoder", "driver",
             "timed_out", "fault", "period", "vbus", "temperature", "ud", "uq",
             "voltage_limit", "id_ref", "iq_ref", "speed_ref", "position_ref", "loop_us",
             "encoder_errors", "encoder_rejects", "overruns"]
    return dict(zip(names, values))


def fault_text(bits: int) -> str:
    labels = [(1, "驱动故障"), (2, "相过流"), (4, "母线过压"), (8, "母线欠压"), (16, "编码器失效")]
    return " / ".join(text for mask, text in labels if bits & mask) or "无故障"


FAULT_EVENT_STRUCT = struct.Struct("<IIHH8f")

def decode_fault_event(payload):
    values = FAULT_EVENT_STRUCT.unpack(payload)
    if not values[0] or not values[2] or not all(math.isfinite(v) for v in values[4:]):
        raise ValueError("故障快照无效")
    return dict(zip(("seq", "time_ms", "bits", "vbus_adc", "vbus", "vbus_raw",
                     "ia", "ib", "ic", "oc", "ov", "uv"), values))
