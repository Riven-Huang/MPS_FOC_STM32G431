from pathlib import Path
import shutil
import subprocess
import pytest
from protocol import Parser, INFO, decode_info


@pytest.fixture(scope="module")
def firmware_exe(tmp_path_factory):
    root = Path(__file__).resolve().parents[2]
    gcc = shutil.which("gcc") or "C:/Program Files (x86)/Dev-Cpp/MinGW64/bin/gcc.exe"
    if not Path(gcc).is_file():
        pytest.skip("Native GCC is required for the firmware harness")
    out = tmp_path_factory.mktemp("firmware") / "harness.exe"
    subprocess.run([gcc, "-std=c99", "-Wall", "-Wextra",
                    "-I" + str(root / "host_app/tests/stubs"), "-I" + str(root / "program/App"),
                    str(root / "host_app/tests/firmware_harness.c"), "-o", str(out), "-lm"], check=True)
    return out


def test_actual_firmware_command_validation(firmware_exe):
    run = subprocess.run([str(firmware_exe)], capture_output=True, text=True, check=True)
    assert "checks passed" in run.stdout


def test_python_decodes_real_firmware_info_and_gains(firmware_exe):
    run = subprocess.run([str(firmware_exe), "--info"], capture_output=True, text=True, check=True)
    frames = Parser().feed(bytes.fromhex(run.stdout.strip()))
    assert len(frames) == 1 and frames[0][0] == INFO
    meta, config, gains, period = decode_info(frames[0][2])
    assert (config.prot_oc, config.prot_ov, config.prot_uv) == (16, 65, 12)
    assert period == 5
    assert config.cur_lpf == 1500
    assert config.mode == 4
    assert config.spd_rpm == 240
    assert config.cur_bw == pytest.approx(250)
    assert config.spd_bw == pytest.approx(10)
    assert gains == pytest.approx(meta.gains(config), rel=2e-6, abs=1e-7)


def test_fault_snapshot_retained_and_decodes(firmware_exe):
    from protocol import FAULT_EVENT, decode_fault_event
    run = subprocess.run([str(firmware_exe), "--event"], capture_output=True, text=True, check=True)
    frames = Parser().feed(bytes.fromhex(run.stdout.strip()))
    assert frames[0][0] == FAULT_EVENT
    record = decode_fault_event(frames[0][2])
    assert record["bits"] == 6
    assert record["time_ms"] == 1234
    assert record["vbus"] == 66.25
    assert record["vbus_raw"] == 68.5
    assert record["ia"] == 10.5
    assert record["ov"] == 65
