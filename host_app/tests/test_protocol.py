from dataclasses import asdict
import math
import struct
import pytest
from protocol import (Parser, encode, crc16, Config, MotorParameters, decode_info,
                      decode_status, SAMPLE, INFO, STATUS_STRUCT)


def test_crc_known_vector():
    assert crc16(b"123456789") == 0x29B1


def test_split_and_concatenated_stream_with_legacy_noise():
    p = Parser()
    one = encode(SAMPLE, 2, struct.pack("<I5f", 123, 1, 2, 3, 4, 5))
    two = encode(SAMPLE, 3, struct.pack("<I5f", 128, 6, 7, 8, 9, 10))
    result = []
    for b in b"legacy\x00\x00\x80\x7f" + one + two:
        result.extend(p.feed(bytes([b])))
    assert [x[1] for x in result] == [2, 3]
    assert struct.unpack("<I5f", result[1][2]) == (128, 6, 7, 8, 9, 10)


def test_corrupt_and_oversized_packets_resynchronize():
    frame = bytearray(encode(SAMPLE, 1, b"abcd"))
    frame[-1] ^= 1
    valid = encode(SAMPLE, 2, b"1234")
    p = Parser()
    result = p.feed(b"\xa5\x5a\x01\x01\xff\xff" + frame + valid)
    assert result == [(SAMPLE, 2, b"1234")]
    assert p.crc_errors == 1


def test_config_round_trip_and_validation():
    c = Config(mode=5, pos_deg=123.5)
    actual = Config.unpack(c.pack())
    for key, value in asdict(c).items():
        assert getattr(actual, key) == pytest.approx(value)
    for bad in [math.nan, math.inf, 1000, -1]:
        c.cur_bw = bad
        with pytest.raises(ValueError):
            c.pack()


def test_speed_bandwidth_halving_quarters_integral():
    meta = MotorParameters()
    a = meta.gains(Config(cur_bw=500, spd_bw=30))
    b = meta.gains(Config(cur_bw=250, spd_bw=15))
    assert b[0] == pytest.approx(a[0] / 2)
    assert b[1] == pytest.approx(a[1] / 2)
    assert b[2] == pytest.approx(a[2] / 2)
    assert b[3] == pytest.approx(a[3] / 4)


def test_status_layout_and_fault_bits():
    p = STATUS_STRUCT.pack(100, 3, 4, 0, 1, 1, 1, 0, 0, 10, 5,
                           48, 30, 1, 2, 27.7, 0, .5, 240, 30, 25, 1, 2, 3)
    s = decode_status(p)
    assert s["fault"] == 10
    assert s["vbus"] == 48
    assert s["overruns"] == 3


def test_reject_incompatible_info():
    with pytest.raises(ValueError):
        decode_info(bytes(10))
    with pytest.raises(ValueError):
        decode_info(bytes([3]) + bytes(151))


@pytest.mark.parametrize("changes", [
    {"prot_oc": 16.1}, {"prot_oc": float("nan")}, {"prot_ov": 80.1},
    {"prot_uv": 11}, {"prot_uv": 50, "prot_ov": 50},
    {"iq_limit": 5, "prot_oc": 4},
])
def test_protection_config_rejects_invalid_thresholds(changes):
    with pytest.raises(ValueError):
        Config(**changes).pack()


def test_protection_roundtrip():
    config = Config(prot_oc=6, prot_ov=58, prot_uv=20, iq_limit=3)
    result = Config.unpack(config.pack())
    assert (result.prot_oc, result.prot_ov, result.prot_uv, result.iq_limit) == (6, 58, 20, 3)


def test_overvoltage_80v_allowed():
    assert Config.unpack(Config(prot_ov=80).pack()).prot_ov == 80



def test_removed_torque_mode_rejected():
    with pytest.raises(ValueError):
        Config(mode=6).pack()


@pytest.mark.parametrize("field", ["id_a", "iq_a", "iq_limit", "prot_oc"])
def test_current_16a_boundary(field):
    config = Config(**{field: 16})
    assert getattr(Config.unpack(config.pack()), field) == 16
    with pytest.raises(ValueError):
        Config(**{field: 16.01}).pack()


def test_negative_current_boundary():
    assert Config.unpack(Config(id_a=-16, iq_a=-16).pack()).iq_a == -16
    with pytest.raises(ValueError):
        Config(iq_a=-16.01).pack()


@pytest.mark.parametrize("cutoff", [0, 1000, 1500, 5000])
def test_current_lpf_roundtrip(cutoff):
    assert Config.unpack(Config(cur_lpf=cutoff).pack()).cur_lpf == cutoff


@pytest.mark.parametrize("cutoff", [-1, 5001, math.nan, math.inf])
def test_current_lpf_invalid(cutoff):
    with pytest.raises(ValueError):
        Config(cur_lpf=cutoff).pack()


def test_old_info_rejected():
    with pytest.raises(ValueError, match="v4"):
        decode_info(bytes([2, 1]) + bytes(150))
