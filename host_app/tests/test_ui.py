import os
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
import pytest
from PySide6.QtWidgets import QApplication
from app import MainWindow


@pytest.fixture(scope="module")
def qt_app():
    return QApplication.instance() or QApplication([])


def test_mode_interlocks_and_position_start(qt_app):
    w = MainWindow(demo=True)
    w.demo_timer.stop()
    w.mode_combo.setCurrentIndex(5)
    assert w.inputs["pos_deg"].isEnabled()
    assert not w.inputs["iq_a"].isEnabled()
    assert not w.inputs["spd_rpm"].isEnabled()
    w.apply_config()
    w.run_motor()
    assert w.status["state"] == 1
    assert not w.inputs["pos_deg"].isEnabled()
    for _ in range(90):
        w.demo_tick()
    assert w.status["state"] == 2
    assert w.inputs["pos_deg"].isEnabled()
    assert not w.inputs["cur_bw"].isEnabled()
    w.stop_motor()
    assert w.inputs["cur_bw"].isEnabled()
    w.mode_combo.setCurrentIndex(2)
    w.angle_combo.setCurrentIndex(1)
    assert w.inputs["open_speed"].isEnabled()
    assert not w.inputs["pos_deg"].isEnabled()
    w.close()


def test_timestamp_wrap_and_pause_keeps_capture(qt_app):
    w = MainWindow(demo=True)
    w.demo_timer.stop()
    w.add_sample(0xFFFFFFFE, [1, 2, 3, 4, 5])
    w.add_sample(3, [1, 2, 3, 4, 5])
    assert w.rows[-1][0] - w.rows[-2][0] == pytest.approx(.005)
    w.pause_button.setChecked(True)
    n = len(w.rows)
    w.add_sample(8, [1, 2, 3, 4, 5])
    w.draw()
    assert len(w.rows) == n + 1
    w.close()


def test_protection_tab_and_running_interlock(qt_app):
    w = MainWindow(demo=True)
    w.demo_timer.stop()
    assert [w.tabs.tabText(i) for i in range(w.tabs.count())] == ["目标", "限幅", "带宽 → PI", "PWM 测试"]
    assert w.inputs["prot_oc"].isEnabled()
    w.status["state"] = 2
    w.update_enables()
    assert not w.inputs["prot_oc"].isEnabled()
    assert not w.inputs["prot_ov"].isEnabled()
    assert not w.inputs["prot_uv"].isEnabled()
    w.close()


def test_window_buttons_center_click_and_offline_stop(qt_app):
    from PySide6.QtTest import QTest
    from PySide6.QtCore import Qt
    w = MainWindow()
    w.show()
    qt_app.processEvents()
    previous = w.window_seconds.value()
    QTest.mouseClick(w.window_more, Qt.LeftButton, pos=w.window_more.rect().center())
    assert w.window_seconds.value() == previous + 1
    QTest.mouseClick(w.window_less, Qt.LeftButton, pos=w.window_less.rect().center())
    assert w.window_seconds.value() == previous
    w.on_connection(False)
    assert not w.stop_button.isEnabled()
    assert "未连接" in w.stop_button.text()
    assert "未知" in w.fault_label.text()
    w.close()


def test_stop_button_center_and_edges_send_stop(qt_app):
    from unittest.mock import patch
    from PySide6.QtTest import QTest
    from PySide6.QtCore import Qt, QPoint
    from protocol import STOP
    w = MainWindow()
    w.on_connection(True)
    w.show()
    qt_app.processEvents()
    with patch.object(w.link, "send") as send:
        for x in (.1, .5, .9):
            for y in (.2, .5, .8):
                QTest.mouseClick(w.stop_button, Qt.LeftButton,
                                 pos=QPoint(int(w.stop_button.width()*x), int(w.stop_button.height()*y)))
        assert send.call_count == 9
        send.assert_called_with(STOP)
    assert "正在发送" in w.banner.text()
    w.on_completed(STOP, 0, 0)
    assert "已受理" in w.banner.text()
    assert w.online
    w.close()


def test_fault_history_dedup_and_disconnect(qt_app, tmp_path):
    from unittest.mock import patch
    from protocol import decode_fault_event, FAULT_EVENT_STRUCT
    w = MainWindow()
    payload = FAULT_EVENT_STRUCT.pack(1, 1234, 4, 3400, 66.25, 68.5, 1, -1, 0, 10, 65, 12)
    with patch("app.QtCore.QStandardPaths.writableLocation", return_value=str(tmp_path)):
        w.record_fault(decode_fault_event(payload))
        w.record_fault(decode_fault_event(payload))
    assert len(w.fault_records) == 1
    assert "66.25" in w.fault_history_box.toPlainText()
    w.on_connection(False)
    assert "66.25" in w.fault_history_box.toPlainText()
    assert len((tmp_path / "fault-events.jsonl").read_text(encoding="utf-8").splitlines()) == 1
    w.close()


def test_display_conversion_gain_and_ntc(qt_app):
    import math
    import numpy as np
    w = MainWindow(demo=True)
    w.demo_timer.stop()
    w.meta.gear_ratio = 8
    w.meta.pole_pairs = 14
    command_before = w.collect_config().pack()
    w.unit_combos[2].setCurrentIndex(1)
    assert w.convert_display(2, 100) == 800
    w.unit_combos[2].setCurrentIndex(3)
    assert w.convert_display(2, 100) == pytest.approx(800*14*math.pi/30)
    w.unit_combos[3].setCurrentIndex(2)
    assert w.convert_display(3, 10) == 80
    w.unit_combos[4].setCurrentIndex(2)
    assert w.convert_display(4, 30) == 60
    w.unit_combos[2].setCurrentIndex(0)
    w.display_gains[2].setValue(.001)
    w.visible_checks[2].setChecked(True)
    w.add_sample(1000, [1, 2, 3000, 10, 30])
    w.draw()
    assert w.curves[2].getData()[1][-1] == pytest.approx(3)
    assert float(w.value_labels[2].text()) == 3000
    assert w.rows[-1][3] == 3000
    assert w.collect_config().pack() == command_before
    w.demo = False
    w.online = True
    w.latest_temperature = float("nan")
    w.visible_checks[5].setChecked(True)
    w.add_sample(1005, [1, 2, 3000, 10, 30])
    w.draw()
    assert "无效" in w.value_labels[5].text()
    assert np.isnan(w.curves[5].getData()[1][-1])
    w.latest_temperature = 32.5
    w.add_sample(1010, [1, 2, 3000, 10, 30])
    assert w.rows[-1][-1] == 32.5
    w.close()



def test_no_separate_torque_mode(qt_app):
    w = MainWindow(demo=True)
    assert w.mode_combo.count() == 6
    assert "torque_nm" not in w.inputs
    w.mode_combo.setCurrentIndex(5)
    assert w.inputs["iq_limit"].isEnabled()
    w.close()


def test_stop_confirmation_requires_status_after_ack(qt_app):
    from protocol import STOP, STATUS, STATUS_STRUCT
    w = MainWindow()
    w.on_completed(STOP, 0, 0)
    assert w.stop_awaiting_status
    payload = STATUS_STRUCT.pack(100, 0, 5, 0, 0, 1, 1, 0, 0, 0, 5,
                                 48, 30, 0, 0, 27.7, 0, 0, 0, 30, 25, 0, 0, 0)
    w.on_frame(STATUS, 0, payload)
    assert not w.stop_awaiting_status
    assert "已确认待机" in w.banner.text()
    w.close()


def test_current_lpf_control_and_idle_interlock(qt_app):
    w = MainWindow(demo=True)
    w.demo_timer.stop()
    spin = w.inputs["cur_lpf"]
    assert (spin.minimum(), spin.maximum(), spin.value()) == (0, 5000, 1500)
    spin.setValue(1000)
    w.apply_config()
    assert w.config.cur_lpf == 1000
    w.status["state"] = 2
    w.update_enables()
    assert not spin.isEnabled()
    w.status["state"] = 0
    w.update_enables()
    assert spin.isEnabled()
    w.close()


def test_axis_sliders_independent_and_pause_follow(qt_app):
    w = MainWindow(demo=True)
    w.demo_timer.stop()
    w.draw_timer.stop()
    before_y = w.plot.viewRange()[1]
    w.x_zoom_slider.setValue(800)
    xlo, xhi = w.plot.viewRange()[0]
    assert xhi == pytest.approx(0)
    assert xhi-xlo == pytest.approx(w.window_seconds.value())
    assert xhi-xlo < 10
    assert w.plot.viewRange()[1] == pytest.approx(before_y)
    w.add_sample(1000, [1,2,3,4,5])
    w.draw()
    assert w.plot.viewRange()[0] == pytest.approx((xlo,xhi))
    w.pause_button.setChecked(True)
    w.follow.setChecked(False)
    w.plot.setXRange(-30,-20,padding=0)
    w.x_zoom_slider.setValue(600)
    xlo,xhi = w.plot.viewRange()[0]
    assert (xlo+xhi)/2 == pytest.approx(-25)
    w.plot.setYRange(10,30,padding=0)
    w.y_zoom_slider.setValue(700)
    ylo,yhi = w.plot.viewRange()[1]
    assert (ylo+yhi)/2 == pytest.approx(20)
    assert yhi-ylo < 20
    assert w.plot.viewRange()[0] == pytest.approx((xlo,xhi))
    w.draw()
    assert w.plot.viewRange()[1] == pytest.approx((ylo,yhi))
    w.window_seconds.setValue(8)
    xlo,xhi = w.plot.viewRange()[0]
    assert xhi-xlo == pytest.approx(8)
    assert (xlo+xhi)/2 == pytest.approx(-25)
    w.close()


def test_axis_sliders_sync_after_fit_and_preset(qt_app):
    w = MainWindow(demo=True)
    w.demo_timer.stop()
    w.plot.setXRange(-10,0,padding=0)
    w.curves[0].setData([-2,-1,0],[-4,0,6])
    w.fit_vertical()
    lo,hi = w.plot.viewRange()[1]
    assert w.y_zoom_slider.value() == w.zoom_position(hi-lo,.002,200000)
    w.preset.setCurrentIndex(1)
    assert w.y_zoom_slider.value() == w.zoom_position(400,.002,200000)
    w.x_zoom_slider.setValue(0)
    assert w.window_seconds.value() == 240
    w.x_zoom_slider.setValue(1000)
    assert w.window_seconds.value() == .1
    w.close()
