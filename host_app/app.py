"""MPS FOC Studio: serial bandwidth tuning, mode commands, and live waveforms."""
from collections import deque
from dataclasses import asdict
from pathlib import Path
import argparse
import csv
import json
import math
import os
import struct
import sys
import time

import numpy as np
import pyqtgraph as pg
from serial.tools import list_ports
from PySide6 import QtCore, QtGui, QtWidgets as W

from protocol import (Config, MotorParameters, FLOAT_NAMES, LIMITS, MODES, CHANNELS,
                      CONFIG, RUN, STOP, CLEAR, READ, RATE, INFO, STATUS, SAMPLE,
                      decode_info, decode_status, fault_text, FAULT_EVENT, decode_fault_event)
from transport import SerialLink

ASSETS = Path(__file__).resolve().parent / "assets"

STYLE = """
QMainWindow, QWidget#root { background: #f1f5fa; color: #20334d; }
QWidget { font-family: 'Microsoft YaHei UI', 'Segoe UI'; font-size: 12px; }
QFrame#header, QFrame#card { background: white; border: 1px solid #dee6ef; border-radius: 8px; }
QGroupBox { background: white; border: 1px solid #dee6ef; border-radius: 7px; margin-top: 16px; padding: 12px 9px 7px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; color: #1469c5; }
QPushButton { background: white; border: 1px solid #cbd8e6; border-radius: 5px; padding: 7px 11px; color: #244361; }
QPushButton:hover { background: #eaf3ff; border-color: #438bdd; }
QPushButton:disabled { color: #a2aebd; background: #eff2f6; border-color: #e1e6ed; }
QPushButton#primary { background: #1670db; color: white; border: 0; }
QPushButton#stop { background: #df4354; color: white; border: 0; font-weight: bold; font-size: 15px; }
QPushButton#primary:disabled, QPushButton#stop:disabled { background: #c1cbd8; }
QDoubleSpinBox, QSpinBox, QComboBox, QLineEdit { background: white; border: 1px solid #ccd8e5; border-radius: 4px; padding: 5px; min-height: 18px; }
QDoubleSpinBox:disabled, QComboBox:disabled, QCheckBox:disabled { color: #a2acbb; background: #f0f3f6; }
QTabWidget::pane { border: 0; }
QTabBar::tab { padding: 9px 12px; background: #e9eff7; color: #546a84; }
QTabBar::tab:selected { background: white; color: #1469c5; border-top: 2px solid #1670db; }
QTableWidget { background: white; border: 1px solid #e1e7ef; gridline-color: #edf1f6; }
QHeaderView::section { background: #eff5fc; padding: 5px; border: 0; color: #526b89; }
QPlainTextEdit { background: #f7faff; border: 1px solid #dfe7f0; border-radius: 5px; color: #53677f; }
QLabel#title { font-size: 22px; font-weight: bold; color: #1265bd; }
QLabel#muted { color: #7c8ea5; }
QScrollArea { border: 0; background: transparent; }
"""


class NoWheelSpinBox(W.QDoubleSpinBox):
    def wheelEvent(self, event):
        # Let the surrounding panel scroll without changing the field.
        event.ignore()


class NoWheelComboBox(W.QComboBox):
    def wheelEvent(self, event):
        event.ignore()


class MainWindow(W.QMainWindow):
    def __init__(self, demo=False):
        super().__init__()
        self.setWindowTitle("MPS FOC Studio · 三环调参与波形")
        self.setWindowIcon(QtGui.QIcon(str(ASSETS / "feibi.png")))
        self.resize(1450, 930)
        self.setMinimumSize(1160, 740)
        self.setStyleSheet(STYLE)
        self.meta = MotorParameters()
        self.config = Config()
        self.fault_seen = set()
        self.fault_records = deque(maxlen=200)
        self.actual_gains = None
        self.status = {}
        self.stop_awaiting_status = False
        self.online = False
        self.ready = False
        self.demo = False
        self.loading = False
        self.refresh_inputs = False
        self.config_inflight = False
        self.rows = deque(maxlen=50000)
        self.last_ms = None
        self.time_offset = 0.0
        self.latest = [0.0] * 5 + [float("nan")]
        self.latest_temperature = float("nan")
        self.received = 0
        self.last_rate_count = 0
        self.last_rate_time = time.monotonic()
        self.link = SerialLink(self)
        self.link.frame.connect(self.on_frame)
        self.link.message.connect(self.log)
        self.link.connection.connect(self.on_connection)
        self.link.completed.connect(self.on_completed)
        self.build_ui()
        for button in self.findChildren(W.QAbstractButton):
            button.clicked.connect(self.play_click)
        for combo in self.findChildren(W.QComboBox):
            combo.activated.connect(self.play_click)
        for tabs in self.findChildren(W.QTabBar):
            tabs.tabBarClicked.connect(self.play_click)
        self.load_inputs(self.config)
        self.scan_ports()
        self.update_enables()
        self.draw_timer = QtCore.QTimer(self)
        self.draw_timer.timeout.connect(self.draw)
        self.draw_timer.start(33)
        self.demo_timer = QtCore.QTimer(self)
        self.demo_timer.timeout.connect(self.demo_tick)
        self.demo_ms = 0
        self.demo_start_ms = 0
        self.demo_speed = 0.0
        self.demo_pos = 0.0
        if demo:
            self.demo_check.setChecked(True)

    def build_ui(self):
        root = W.QWidget(objectName="root")
        self.setCentralWidget(root)
        layout = W.QVBoxLayout(root)
        layout.setContentsMargins(15, 12, 15, 12)
        header = W.QFrame(objectName="header")
        top = W.QHBoxLayout(header)
        badge = W.QLabel()
        badge.setPixmap(QtGui.QPixmap(str(ASSETS / "feibi.png")).scaled(52, 52, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        top.addWidget(badge)
        title_col = W.QVBoxLayout()
        title_col.addWidget(W.QLabel("MPS FOC Studio", objectName="title"))
        title_col.addWidget(W.QLabel("三环带宽整定  /  实时观测  /  串口控制", objectName="muted"))
        top.addLayout(title_col)
        top.addStretch()
        self.sound_check = W.QCheckBox("点击音效")
        self.sound_check.setChecked(QtCore.QSettings("MPS", "FOCStudio").value("click_sound", True, type=bool))
        self.sound_check.toggled.connect(lambda enabled: QtCore.QSettings("MPS", "FOCStudio").setValue("click_sound", enabled))
        top.addWidget(self.sound_check)
        self.state_label = W.QLabel("● 未连接")
        self.state_label.setStyleSheet("font-size: 16px; color: #77899f;")
        top.addWidget(self.state_label)
        self.stop_button = W.QPushButton("■  停止电机  Esc", objectName="stop")
        self.stop_button.setMinimumWidth(165)
        self.stop_button.clicked.connect(self.stop_motor)
        top.addWidget(self.stop_button)
        layout.addWidget(header)
        self.banner = W.QLabel("连接后先读取控制板参数；启动与参数下发分开操作。")
        self.banner.setStyleSheet("padding: 6px; color: #537397;")
        layout.addWidget(self.banner)
        splitter = W.QSplitter(QtCore.Qt.Orientation.Horizontal)
        layout.addWidget(splitter, 1)
        left_scroll = W.QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setMinimumWidth(320)
        left = W.QWidget()
        left_layout = W.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 8, 0)
        left_scroll.setWidget(left)
        splitter.addWidget(left_scroll)

        group = W.QGroupBox("协议与连接")
        form = W.QFormLayout(group)
        row = W.QHBoxLayout()
        self.port_combo = NoWheelComboBox()
        self.port_combo.setSizeAdjustPolicy(W.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.port_combo.setMinimumContentsLength(12)
        row.addWidget(self.port_combo, 1)
        scan = W.QPushButton("刷新")
        scan.clicked.connect(self.scan_ports)
        row.addWidget(scan)
        form.addRow("串口", row)
        form.addRow("协议", W.QLabel("FOC Serial v4 · 115200 / 8N1"))
        self.connect_button = W.QPushButton("连接控制板", objectName="primary")
        self.connect_button.clicked.connect(self.toggle_connect)
        form.addRow(self.connect_button)
        self.demo_check = W.QCheckBox("演示模式（模拟数据，不连接电机）")
        self.demo_check.toggled.connect(self.set_demo)
        form.addRow(self.demo_check)
        left_layout.addWidget(group)

        group = W.QGroupBox("模式与运行")
        form = W.QFormLayout(group)
        self.mode_combo = NoWheelComboBox()
        self.mode_combo.addItems(MODES)
        self.mode_combo.currentIndexChanged.connect(self.update_enables)
        form.addRow("工作模式", self.mode_combo)
        self.angle_combo = NoWheelComboBox()
        self.angle_combo.setSizeAdjustPolicy(W.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.angle_combo.setMinimumContentsLength(14)
        self.angle_combo.setToolTip("MA600A：读取电机转子机械角，结合极对数及对齐偏移换算换相电角度。\n开环积分：积分电角速度得到换相电角度，不使用编码器角度反馈。")
        self.angle_combo.addItems(["MA600A（换算电角度）", "开环积分（电角度）"])
        self.angle_combo.currentIndexChanged.connect(self.update_enables)
        form.addRow("换相角度（电角度）", self.angle_combo)
        self.run_button = W.QPushButton("▶  启动已下发模式", objectName="primary")
        self.run_button.clicked.connect(self.run_motor)
        self.clear_button = W.QPushButton("清除故障")
        self.clear_button.clicked.connect(lambda: self.command(CLEAR))
        form.addRow(self.run_button, self.clear_button)
        note = W.QLabel("模式与带宽修改需先停机；运行中可下发目标值。\n位置模式启动后先保持当前位置，再下发位置目标。")
        note.setWordWrap(True)
        note.setObjectName("muted")
        form.addRow(note)
        left_layout.addWidget(group)

        self.tabs = W.QTabWidget()
        self.inputs = {}
        target_page = W.QWidget()
        targets = W.QFormLayout(target_page)
        spec = [("id_a", "d 轴电流 / A"), ("iq_a", "q 轴电流 / A"),
                ("spd_rpm", "转速（电机转子）/ rpm"), ("pos_deg", "位置（减速箱输出轴）/ °"),
                ("ud_v", "d 轴电压 / V"), ("uq_v", "q 轴电压 / V"),
                ("open_speed", "开环速度（电角速度）/ rad/s")]
        for key, label in spec:
            targets.addRow(label, self.make_spin(key))
        self.speed_hint = W.QLabel()
        self.speed_hint.setObjectName("muted")
        targets.addRow(self.speed_hint)
        self.inputs["spd_rpm"].valueChanged.connect(self.update_speed_hint)
        self.tabs.addTab(target_page, "目标")
        limit_page = W.QWidget()
        limits_form = W.QFormLayout(limit_page)
        for key, label in [("iq_limit", "电流给定限幅 / A"),
                           ("pos_speed_limit", "限速（减速箱输出轴）/ rad/s"),
                           ("prot_oc", "相电流过流阈值 / A"),
                           ("prot_ov", "母线过压阈值 / V"),
                           ("prot_uv", "母线欠压阈值 / V")]:
            limits_form.addRow(label, self.make_spin(key))
        protection_note = W.QLabel(
            "保护阈值需停机后下发，复位恢复默认值。\n"
            "过流：任一相电流绝对值超限，默认 16 A（0x02）。\n"
            "过压：默认 65 V，可设上限 80 V（0x04）。\n"
            "当前分压理论量程 82.5 V；80 V 为阈值设置上限，不代表整板额定耐压。\n"
            "欠压：默认 12 V，非待机时检测（0x08）。\n"
            "电流给定限幅不等同于过流保护，对齐也受过流保护。\n\n"
            "固定保护：驱动器故障引脚（0x01）；对齐或需编码器的运行模式下编码器失效（0x10）；远程运行失联 2 秒停机。\n"
            "未实现：过温、超速故障保护。输出轴限速仅限制位置环给定。")
        protection_note.setWordWrap(True)
        protection_note.setObjectName("muted")
        limits_form.addRow(protection_note)
        self.tabs.addTab(limit_page, "限幅")
        tune_page = W.QWidget()
        tuning = W.QFormLayout(tune_page)
        for key, label in [("cur_bw", "电流环带宽 / Hz"), ("spd_bw", "速度环带宽 / Hz"),
                           ("pos_bw", "位置环带宽 / Hz"), ("cur_lpf", "电流低通 / Hz"),
                           ("spd_lpf", "测速低通 / Hz"),
                           ("pos_ki", "位置积分 Ki"), ("pos_kd", "位置阻尼 Kd")]:
            tuning.addRow(label, self.make_spin(key))
        self.inputs["cur_lpf"].setToolTip("dq 电流反馈一阶低通，默认 1500 Hz；0=关闭。作用于电流环和电流曲线，过流保护使用未滤波相电流。停机后修改并下发。")
        self.hold_check = W.QCheckBox("位置保持 / 爬行抑抖")
        tuning.addRow(self.hold_check)
        self.gain_table = W.QTableWidget(3, 3)
        self.gain_table.setHorizontalHeaderLabels(["环路", "Kp 预览", "Ki 预览"])
        self.gain_table.verticalHeader().hide()
        self.gain_table.horizontalHeader().setSectionResizeMode(W.QHeaderView.ResizeMode.Stretch)
        self.gain_table.setEditTriggers(W.QAbstractItemView.EditTrigger.NoEditTriggers)
        self.gain_table.setFixedHeight(128)
        tuning.addRow(self.gain_table)
        self.actual_label = W.QLabel("实际 PI：连接后读取")
        self.actual_label.setWordWrap(True)
        self.actual_label.setObjectName("muted")
        tuning.addRow(self.actual_label)
        self.tabs.addTab(tune_page, "带宽 → PI")
        pwm_page = W.QWidget()
        pwm_form = W.QFormLayout(pwm_page)
        for key, label in [("duty_a", "A 相占空比"), ("duty_b", "B 相占空比"), ("duty_c", "C 相占空比")]:
            pwm_form.addRow(label, self.make_spin(key))
        pwm_note = W.QLabel("PWM TEST 直接输出占空比，供断开电机时检查发波链路。")
        pwm_note.setWordWrap(True)
        pwm_form.addRow(pwm_note)
        self.tabs.addTab(pwm_page, "PWM 测试")
        left_layout.addWidget(self.tabs)
        action_row = W.QHBoxLayout()
        self.apply_button = W.QPushButton("下发参数 / 目标", objectName="primary")
        self.apply_button.clicked.connect(self.apply_config)
        self.read_button = W.QPushButton("读取板上参数")
        self.read_button.clicked.connect(self.read_config)
        action_row.addWidget(self.apply_button)
        action_row.addWidget(self.read_button)
        left_layout.insertLayout(2, action_row)
        files_row = W.QHBoxLayout()
        save = W.QPushButton("保存方案")
        save.clicked.connect(self.save_config)
        load = W.QPushButton("载入方案")
        load.clicked.connect(self.load_config)
        files_row.addWidget(save)
        files_row.addWidget(load)
        left_layout.addLayout(files_row)
        self.model_label = W.QLabel("模型参数：未连接，PI 预览采用演示基准。")
        self.model_label.setWordWrap(True)
        self.model_label.setObjectName("muted")
        left_layout.addWidget(self.model_label)
        left_layout.addStretch()

        center = W.QFrame(objectName="card")
        center_layout = W.QVBoxLayout(center)
        toolbar = W.QHBoxLayout()
        toolbar.addWidget(W.QLabel("实时波形", objectName="title"))
        toolbar.addStretch()
        self.preset = NoWheelComboBox()
        self.preset.addItems(["电流", "速度", "位置 / 角度", "全部通道", "NTC 温度"])
        self.preset.currentIndexChanged.connect(self.set_preset)
        toolbar.addWidget(self.preset)
        self.pause_button = W.QPushButton("暂停显示")
        self.pause_button.setCheckable(True)
        self.pause_button.toggled.connect(lambda value: self.pause_button.setText("继续显示" if value else "暂停显示"))
        toolbar.addWidget(self.pause_button)
        center_layout.addLayout(toolbar)
        pg.setConfigOptions(antialias=False, background="w", foreground="#61738b")
        self.plot = pg.PlotWidget()
        self.plot.showGrid(x=True, y=True, alpha=.2)
        self.plot.setLabel("bottom", "相对时间（右侧 0 为最新数据）", units="s")
        self.plot.hideAxis("left")
        self.plot.showAxis("right")
        self.plot.getAxis("right").setWidth(85)
        self.plot.disableAutoRange()
        self.plot.setXRange(-10, 0, padding=0)
        self.plot.setYRange(-1, 1, padding=0)
        self.plot_time_anchor = 0.0
        self.plot.setLabel("right", "电流", units="A")
        self.plot.getPlotItem().setMenuEnabled(True)
        self.curves = [self.plot.plot(pen=pg.mkPen(color, width=1.3), name=name,
                                      autoDownsample=True, clipToView=True)
                       for name, _, color in CHANNELS]
        self.cursor_a = pg.InfiniteLine(pos=0, angle=90, movable=True, pen=pg.mkPen("#f05f70", style=QtCore.Qt.PenStyle.DashLine))
        self.cursor_b = pg.InfiniteLine(pos=1, angle=90, movable=True, pen=pg.mkPen("#9b51bd", style=QtCore.Qt.PenStyle.DashLine))
        self.plot.addItem(self.cursor_a)
        self.plot.addItem(self.cursor_b)
        self.cursor_a.sigPositionChanged.connect(self.update_cursors)
        self.cursor_b.sigPositionChanged.connect(self.update_cursors)
        self.cursor_a.hide()
        self.cursor_b.hide()
        center_layout.addWidget(self.plot, 1)
        controls = W.QHBoxLayout()
        self.follow = W.QCheckBox("跟随最新")
        self.follow.setChecked(True)
        controls.addWidget(self.follow)
        controls.addWidget(W.QLabel("窗口 / s"))
        self.window_seconds = NoWheelSpinBox()
        self.window_seconds.setRange(.1, 240)
        self.window_seconds.setValue(10)
        self.window_seconds.setButtonSymbols(W.QAbstractSpinBox.ButtonSymbols.NoButtons)
        self.window_seconds.setMaximumWidth(90)
        controls.addWidget(self.window_seconds)
        self.window_less = W.QPushButton("−")
        self.window_more = W.QPushButton("+")
        for button, tip, action in ((self.window_less, "缩短时间窗口", self.window_seconds.stepDown),
                                    (self.window_more, "增大时间窗口", self.window_seconds.stepUp)):
            button.setFixedSize(32, 32)
            button.setToolTip(tip)
            button.clicked.connect(action)
            controls.addWidget(button)
        self.cursor_check = W.QCheckBox("双游标")
        self.cursor_check.toggled.connect(self.toggle_cursors)
        controls.addWidget(self.cursor_check)
        self.cursor_label = W.QLabel("Δt —")
        controls.addWidget(self.cursor_label)
        controls.addStretch()
        center_layout.addLayout(controls)
        zoom_controls = W.QGridLayout()
        self.x_zoom_slider = W.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.y_zoom_slider = W.QSlider(QtCore.Qt.Orientation.Horizontal)
        self.x_zoom_label = W.QLabel()
        self.y_zoom_label = W.QLabel()
        for row, (name, slider, label) in enumerate((
                ("X 轴缩放", self.x_zoom_slider, self.x_zoom_label),
                ("Y 轴缩放", self.y_zoom_slider, self.y_zoom_label))):
            slider.setRange(0, 1000)
            slider.setMinimumWidth(120)
            slider.setToolTip("向左缩小，向右放大；仅改变波形视图")
            label.setMinimumWidth(130)
            zoom_controls.addWidget(W.QLabel(name), row, 0)
            zoom_controls.addWidget(W.QLabel("−"), row, 1)
            zoom_controls.addWidget(slider, row, 2)
            zoom_controls.addWidget(W.QLabel("+"), row, 3)
            zoom_controls.addWidget(label, row, 4)
        zoom_controls.setColumnStretch(2, 1)
        center_layout.addLayout(zoom_controls)
        self.x_zoom_slider.valueChanged.connect(self.zoom_horizontal)
        self.y_zoom_slider.valueChanged.connect(self.zoom_vertical)
        self.window_seconds.valueChanged.connect(self.set_time_window)
        self.plot.getViewBox().sigRangeChanged.connect(self.sync_zoom_controls)
        self.sync_zoom_controls()
        controls2 = W.QHBoxLayout()
        controls2.addWidget(W.QLabel("采集频率"))
        self.rate_combo = NoWheelComboBox()
        for label, ms in [("200 Hz", 5), ("100 Hz", 10), ("50 Hz", 20), ("20 Hz", 50)]:
            self.rate_combo.addItem(label, ms)
        self.rate_combo.currentIndexChanged.connect(self.change_rate)
        controls2.addWidget(self.rate_combo)
        clear = W.QPushButton("清空波形")
        clear.clicked.connect(self.clear_plot)
        controls2.addWidget(clear)
        fit_y = W.QPushButton("适配纵轴")
        fit_y.setToolTip("按当前可见曲线调整一次纵轴，随后保持固定；也可手动缩放")
        fit_y.clicked.connect(self.fit_vertical)
        controls2.addWidget(fit_y)
        export = W.QPushButton("导出 CSV")
        export.clicked.connect(self.export_csv)
        controls2.addWidget(export)
        controls2.addStretch()
        self.rate_label = W.QLabel("0 帧/s · 0 / 50000")
        controls2.addWidget(self.rate_label)
        center_layout.addLayout(controls2)
        self.log_box = W.QPlainTextEdit()
        self.log_box.setReadOnly(True)
        self.log_box.setMaximumBlockCount(250)
        self.log_box.setFixedHeight(105)
        center_layout.addWidget(self.log_box)
        splitter.addWidget(center)

        right_scroll = W.QScrollArea()
        right_scroll.setWidgetResizable(True)
        right_scroll.setMinimumWidth(310)
        right = W.QFrame(objectName="card")
        right_scroll.setWidget(right)
        right.setMinimumWidth(200)
        right_layout = W.QVBoxLayout(right)
        right_layout.addWidget(W.QLabel("实时数据", objectName="title"))
        self.display_gains = []
        self.unit_combos = {}
        self.visible_checks = []
        self.value_labels = []
        for i, (name, unit, color) in enumerate(CHANNELS):
            check = W.QCheckBox(name + "  / " + unit)
            check.setChecked(i < 2)
            check.setStyleSheet(f"color: {color}; font-weight: bold; padding-top: 10px;")
            check.toggled.connect(lambda value, index=i: self.curves[index].setVisible(value))
            check.toggled.connect(self.refresh_display_units)
            self.visible_checks.append(check)
            label = W.QLabel("—")
            label.setStyleSheet(f"color: {color}; font-size: 21px; font-family: Consolas; padding-bottom: 8px;")
            self.value_labels.append(label)
            right_layout.addWidget(check)
            right_layout.addWidget(label)
            display_row = W.QHBoxLayout()
            if i in (1, 2, 3, 4):
                units = NoWheelComboBox()
                choices = {
                    1: ["q 轴电流 / A", "转子估算扭矩 / N·m", "输出轴估算扭矩 / N·m"],
                    2: ["减速箱输出轴 / rpm", "电机转子 / rpm", "电机转子 / rad/s", "电角速度 / rad/s", "减速箱输出轴 / rad/s"],
                    3: ["减速箱输出轴 / °", "减速箱输出轴 / rad", "转子等效位置 / °", "转子等效位置 / rad", "电角等效位置 / °"],
                    4: ["转子机械角 / °", "转子机械角 / rad", "电角度（未对齐）/ °", "电角度（未对齐）/ rad", "输出轴单圈折算 / °"],
                }
                units.addItems(choices[i])
                units.setMinimumContentsLength(10)
                units.setSizeAdjustPolicy(W.QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
                units.setToolTip("仅改变显示。位置倍数折算沿用原通道零点；电角度没有对齐零偏；编码器单圈折算不能恢复输出轴绝对位置。")
                self.unit_combos[i] = units
                units.currentIndexChanged.connect(self.refresh_display_units)
                right_layout.addWidget(units)
            display_row.addWidget(W.QLabel("曲线增益 ×"))
            gain = NoWheelSpinBox()
            gain.setRange(.00001, 10000)
            gain.setDecimals(5)
            gain.setSingleStep(.1)
            gain.setValue(1)
            gain.setButtonSymbols(W.QAbstractSpinBox.ButtonSymbols.NoButtons)
            gain.setToolTip("仅缩放曲线，不改变控制增益、右侧实际数值或 CSV 原始数据。直接输入，例如 0.001。")
            gain.valueChanged.connect(self.refresh_display_units)
            self.display_gains.append(gain)
            display_row.addWidget(gain)
            right_layout.addLayout(display_row)
        self.fault_label = W.QLabel("故障：—")
        self.fault_label.setWordWrap(True)
        right_layout.addWidget(self.fault_label)
        self.status_label = W.QLabel("母线：—\n温度：—\n电流零偏：—\n编码器：—")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("color: #647993; line-height: 150%;")
        right_layout.addWidget(self.status_label)
        right_layout.addWidget(W.QLabel("故障触发记录（非峰值）"))
        self.fault_history_box = W.QPlainTextEdit()
        self.fault_history_box.setReadOnly(True)
        self.fault_history_box.setMaximumBlockCount(1200)
        self.fault_history_box.setMinimumHeight(160)
        self.fault_history_box.setPlaceholderText("等待故障快照；需配套固件。\n清故障不会清除此记录。")
        right_layout.addWidget(self.fault_history_box, 1)
        export_faults = W.QPushButton("导出故障记录")
        export_faults.clicked.connect(self.export_fault_records)
        right_layout.addWidget(export_faults)
        right_layout.addWidget(W.QLabel("鼠标拖动平移 · 滚轮缩放\n暂停仅停止绘图，电机仍运行。", objectName="muted"))
        splitter.addWidget(right_scroll)
        splitter.setSizes([350, 780, 320])
        self.set_preset(0)
        for child in left.findChildren(W.QWidget):
            child_layout = child.layout()
            if isinstance(child_layout, W.QFormLayout):
                child_layout.setRowWrapPolicy(W.QFormLayout.RowWrapPolicy.WrapLongRows)
        QtGui.QShortcut(QtGui.QKeySequence("Escape"), self, activated=self.stop_motor)

    def make_spin(self, key):
        spin = NoWheelSpinBox()
        lo, hi = LIMITS[FLOAT_NAMES.index(key)]
        spin.setRange(lo, hi)
        spin.setDecimals(4 if key.startswith("duty") else 3)
        spin.setSingleStep(.1 if key in ("id_a", "iq_a", "ud_v", "uq_v", "iq_limit") else 1)
        spin.setKeyboardTracking(False)
        spin.valueChanged.connect(self.update_gain_preview)
        self.inputs[key] = spin
        return spin

    def scan_ports(self):
        current = self.port_combo.currentData()
        self.port_combo.clear()
        for port in list_ports.comports():
            self.port_combo.addItem(f"{port.device} · {port.description}", port.device)
        index = self.port_combo.findData(current)
        if index >= 0:
            self.port_combo.setCurrentIndex(index)

    def toggle_connect(self):
        if self.link.port:
            self.link.close()
        elif self.port_combo.currentData():
            self.demo_check.setChecked(False)
            self.link.open(self.port_combo.currentData())
        else:
            self.log("未找到串口，请连接 USB 串口后刷新。")

    def on_connection(self, connected):
        self.stop_awaiting_status = False
        self.online = connected
        self.ready = False
        self.config_inflight = False
        self.refresh_inputs = True
        self.connect_button.setText("断开连接（同时停机）" if connected else "连接控制板")
        self.port_combo.setEnabled(not connected)
        self.demo_check.setEnabled(not connected)
        self.state_label.setText("● 握手中" if connected else "● 未连接")
        self.banner.setText("正在读取固件配置；尚未允许启动。" if connected else "串口已断开：停止指令无法发送，当前电机状态未知。重新连接不会自动启动。")
        if connected:
            self.clear_plot()
        else:
            self.fault_label.setText("状态未知（串口已断开）")
            self.fault_label.setStyleSheet("color: #c27b21;")
            self.status_label.setText("实时数据已失效；波形保留为历史。")
        self.update_enables()

    def collect_config(self):
        values = asdict(self.config)
        for key, spin in self.inputs.items():
            # Preserve the exact readback float if the displayed value was not edited.
            if round(values[key], spin.decimals()) != spin.value():
                values[key] = spin.value()
        values.update(mode=self.mode_combo.currentIndex(), open_angle=self.angle_combo.currentIndex(),
                      hold=int(self.hold_check.isChecked()))
        config = Config(**values)
        config.pack()
        return config

    def load_inputs(self, config):
        self.loading = True
        self.config = config
        self.mode_combo.setCurrentIndex(config.mode)
        self.angle_combo.setCurrentIndex(config.open_angle)
        self.hold_check.setChecked(bool(config.hold))
        for key, spin in self.inputs.items():
            spin.setValue(getattr(config, key))
        self.loading = False
        self.update_gain_preview()
        self.update_speed_hint()
        self.update_enables()

    def update_enables(self, *_):
        if self.loading or not hasattr(self, "apply_button"):
            return
        usable = self.ready or self.demo
        state = self.status.get("state", 0)
        idle = state == 0
        mode = self.mode_combo.currentIndex()
        editable = usable and not self.config_inflight
        self.mode_combo.setEnabled(editable and idle)
        self.angle_combo.setEnabled(editable and idle and mode == 2)
        targets = { "id_a": mode == 3, "iq_a": mode == 3, "spd_rpm": mode == 4,
                   "pos_deg": mode == 5, "ud_v": mode == 2, "uq_v": mode == 2,
                   "open_speed": mode == 2 and self.angle_combo.currentIndex() == 1,
                   "iq_limit": mode >= 3, "pos_speed_limit": mode == 5,
                   "duty_a": mode == 1, "duty_b": mode == 1, "duty_c": mode == 1}
        for key, spin in self.inputs.items():
            spin.setEnabled(editable and state in (0, 2) and (targets[key] if key in targets else idle))
        self.hold_check.setEnabled(editable and idle)
        self.apply_button.setEnabled(editable and state in (0, 2))
        self.read_button.setEnabled(usable and not self.config_inflight)
        self.run_button.setEnabled(editable and idle and self.config.mode != 0 and not self.status.get("fault", 0))
        self.stop_button.setEnabled(self.online or self.demo)
        self.stop_button.setText("■  停止电机  Esc" if self.online or self.demo else "停止不可用（未连接）")
        self.stop_button.setToolTip("发送停止命令，保持串口连接" if self.online or self.demo else "串口未连接，无法发送停止命令；电机状态未知")
        self.clear_button.setEnabled(usable and state == 3)
        self.rate_combo.setEnabled(usable)

    def update_gain_preview(self, *_):
        if self.loading or not hasattr(self, "gain_table"):
            return
        try:
            config = self.collect_config()
            gains = self.meta.gains(config)
        except ValueError:
            return
        for row, (name, kp, ki) in enumerate(zip(["电流", "速度", "位置"], gains[::2][:3], gains[1::2][:3])):
            for col, text in enumerate([name, f"{kp:.6g}", f"{ki:.6g}"]):
                self.gain_table.setItem(row, col, W.QTableWidgetItem(text))

    def update_speed_hint(self, *_):
        self.speed_hint.setText(f"速度目标（减速箱输出轴）约 {self.inputs['spd_rpm'].value() / self.meta.gear_ratio:.3f} rpm")

    def apply_config(self):
        try:
            config = self.collect_config()
        except ValueError as exc:
            self.log(str(exc))
            return
        if self.demo:
            self.config = config
            self.actual_gains = self.meta.gains(config)
            self.show_gains()
            self.log("演示：参数已应用到模拟器，未发送串口。")
        else:
            self.config_inflight = True
            self.link.send(CONFIG, config.pack())
        self.update_enables()

    def read_config(self):
        if self.demo:
            self.load_inputs(self.config)
        else:
            self.refresh_inputs = True
            self.link.send(READ)

    def run_motor(self):
        try:
            if self.collect_config().pack() != self.config.pack():
                self.log("有未下发的参数，请先下发并等待回读，再启动。")
                return
        except ValueError as exc:
            self.log(str(exc))
            return
        if self.demo:
            self.demo_start_ms = self.demo_ms
            self.status.update(state=1 if self.config.mode >= 3 or
                               (self.config.mode == 2 and not self.config.open_angle) else 2, run=1)
            self.log("演示：启动模拟电机。")
            self.update_enables()
        else:
            self.link.send(RUN)
            self.run_button.setEnabled(False)

    def stop_motor(self):
        if self.demo:
            self.status.update(state=0, run=0)
            self.log("演示：已停机。")
            self.update_enables()
        elif self.online:
            self.stop_awaiting_status = False
            self.link.send(STOP)
            self.log("停止命令已排入优先队列，等待控制板回执。")
            self.banner.setText("停止请求正在发送，等待控制板回执；串口保持连接。")
        else:
            self.log("无法发送停止：串口未连接，当前电机状态未知。")

    def command(self, kind):
        if self.demo:
            self.status.update(state=0, run=0, fault=0)
            self.update_enables()
        else:
            self.link.send(kind)

    def on_completed(self, op, result, _field):
        if op == STOP:
            self.stop_awaiting_status = result == 0
            self.banner.setText("控制板已受理停止命令；请以状态回读确认待机，串口保持连接。" if result == 0 else "停止命令未被接受，请查看日志和实际电机状态。")
        if op == CONFIG:
            if result == 0:
                self.refresh_inputs = True
                self.link.send(READ)
            else:
                self.config_inflight = False
        if op == READ and result != 0:
            self.config_inflight = False
        self.update_enables()

    def on_frame(self, kind, _seq, payload):
        try:
            if kind == INFO:
                meta, config, gains, period = decode_info(payload)
                self.meta = meta
                self.actual_gains = gains
                self.ready = True
                self.config_inflight = False
                if self.refresh_inputs:
                    self.load_inputs(config)
                    self.refresh_inputs = False
                self.show_gains()
                self.model_label.setText(f"板上模型：R={meta.resistance:.4g} Ω · L={meta.inductance * 1e3:.4g} mH\n"
                                         f"极对数 {meta.pole_pairs:g} · 减速比 {meta.gear_ratio:g} · J={meta.inertia:.4g}\n"
                                         "方案保存到电脑；参数仅写入固件 RAM，复位恢复编译默认值。")
                self.banner.setText("控制板已连接 · 参数回读完成 · 远程运行心跳超时 2 秒停机")
                self.loading = True
                self.rate_combo.setCurrentIndex(max(0, self.rate_combo.findData(period)))
                self.loading = False
                self.update_enables()
            elif kind == FAULT_EVENT:
                self.record_fault(decode_fault_event(payload))
            elif kind == STATUS:
                previous = self.status.get("state", 0)
                self.status = decode_status(payload)
                if self.stop_awaiting_status and self.status["run"] == 0 and self.status["state"] in (0, 3):
                    self.stop_awaiting_status = False
                    text = "已确认待机：运行已撤销，串口和波形采集继续。" if self.status["state"] == 0 else "已确认故障停机：运行已撤销，故障仍锁存。"
                    self.banner.setText(text)
                    self.log(text)
                temperature = self.status["temperature"]
                self.latest_temperature = temperature if math.isfinite(temperature) and temperature > -273 else float("nan")
                # Firmware latches current position on entering POSITION. Reflect that target.
                if self.status["mode"] == 5 and self.status["state"] == 2 and previous != 2:
                    self.config.pos_deg = self.status["position_ref"]
                    self.inputs["pos_deg"].setValue(self.config.pos_deg)
                    self.log("位置模式已保持当前位置，现在可下发新的位置目标。")
                self.update_status()
            elif kind == SAMPLE:
                ms, *values = struct.unpack("<I5f", payload)
                self.add_sample(ms, values)
        except (ValueError, struct.error, ZeroDivisionError) as exc:
            self.log(f"数据解析失败：{exc}")
            if kind == INFO:
                self.link.close()

    def record_fault(self, record):
        key = tuple(record.values())
        if key in self.fault_seen:
            return
        self.fault_seen.add(key)
        if len(self.fault_seen) > 2048:
            self.fault_seen = {key}
        record["received_at"] = time.strftime("%Y-%m-%d %H:%M:%S")
        self.fault_records.append(record)
        text = (f"{record['received_at']} · #{record['seq']}\n"
                f"板时 {record['time_ms']/1000:.3f} s · {fault_text(record['bits'])}\n"
                f"触发母线 {record['vbus']:.2f} V / 原始 {record['vbus_raw']:.2f} V\n"
                f"相电流 A {record['ia']:.3f} / B {record['ib']:.3f} / C {record['ic']:.3f} A\n"
                f"阈值 OC {record['oc']:.2f} A · OV {record['ov']:.2f} V · UV {record['uv']:.2f} V")
        if record["vbus_adc"] >= 4095:
            text += "\n电压 ADC 已满量程，实际值可能更高！"
        self.fault_history_box.appendPlainText(text + "\n")
        self.log(f"已记录故障 #{record['seq']}：母线 {record['vbus']:.2f} V，三相 {record['ia']:.2f}/{record['ib']:.2f}/{record['ic']:.2f} A")
        # Local persistent journal, outside the executable and firmware project.
        try:
            folder = Path(QtCore.QStandardPaths.writableLocation(QtCore.QStandardPaths.StandardLocation.AppLocalDataLocation))
            folder.mkdir(parents=True, exist_ok=True)
            with (folder / "fault-events.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")
        except OSError as exc:
            self.log(f"故障记录保存失败，可手动导出：{exc}")

    def export_fault_records(self):
        path, _ = W.QFileDialog.getSaveFileName(self, "导出本次会话故障记录", "fault-events.json", "JSON (*.json)")
        if path:
            try:
                Path(path).write_text(json.dumps(list(self.fault_records), ensure_ascii=False, indent=2), encoding="utf-8")
            except OSError as exc:
                self.log(f"导出失败：{exc}")

    def show_gains(self):
        if self.actual_gains:
            a = self.actual_gains
            self.actual_label.setText(f"板上实际 PI\n电流 {a[0]:.6g} / {a[1]:.6g}\n"
                                      f"速度 {a[2]:.6g} / {a[3]:.6g}\n"
                                      f"位置 {a[4]:.6g} / {a[5]:.6g} · Kd {a[6]:.4g}")

    def update_status(self):
        s = self.status
        state = s.get("state", 0)
        text = ["待机", "对齐中", "运行中", "故障锁存"][state] if state in range(4) else "未知状态"
        self.state_label.setText(("◉ 演示 · " if self.demo else "● ") + text)
        self.state_label.setStyleSheet(f"font-size: 16px; color: {'#df4354' if state == 3 else '#1469c5'};")
        bits = s.get("fault", 0)
        self.fault_label.setText(f"故障 0x{bits:02X}\n{fault_text(bits)}")
        self.fault_label.setStyleSheet(f"color: {'#df4354' if bits else '#269677'}; padding-top: 12px;")
        temperature = s.get("temperature", float("nan"))
        temperature_text = f"{temperature:.1f} °C" if math.isfinite(temperature) and temperature > -273 else "无效（检查 NTC 采样）"
        self.status_label.setText(f"母线  {s.get('vbus', 0):.2f} V\nNTC  {temperature_text}\n"
                                  f"零偏  {'就绪' if s.get('offset') else '未就绪'}\n"
                                  f"编码器  {'有效' if s.get('encoder') else '无效'}\n"
                                  f"快环  {s.get('loop_us', 0):.1f} μs\n"
                                  f"SPI 错误  {s.get('encoder_errors', 0)}\n"
                                  f"角度剔除  {s.get('encoder_rejects', 0)}\n"
                                  f"超时计数  {s.get('overruns', 0)}")
        self.update_enables()

    def add_sample(self, ms, values):
        if not all(math.isfinite(v) for v in values):
            return
        if self.last_ms is not None and ms < self.last_ms:
            if self.last_ms - ms > 0x80000000:
                self.time_offset += 2**32 / 1000
            else:
                self.clear_plot()
                self.log("控制板时间重新起算，波形缓存已清空。")
        self.last_ms = ms
        temperature = self.status.get("temperature", float("nan")) if self.demo else self.latest_temperature
        self.latest = [*values, temperature]
        self.rows.append((self.time_offset + ms / 1000, *values, temperature))
        self.received += 1

    def draw(self):
        if not self.rows:
            return
        if not self.pause_button.isChecked():
            array = np.asarray(self.rows, dtype=float)
            if self.follow.isChecked():
                self.plot_time_anchor = array[-1, 0]
            relative_time = array[:, 0] - self.plot_time_anchor
            for i, curve in enumerate(self.curves):
                if self.visible_checks[i].isChecked():
                    curve.setData(relative_time, self.convert_display(i, array[:, i + 1]) * self.display_gains[i].value(), connect="finite")
            if self.follow.isChecked():
                self.plot.setXRange(-self.window_seconds.value(), 0, padding=0)
        for i, (label, value) in enumerate(zip(self.value_labels, self.latest)):
            value = self.convert_display(i, value)
            label.setText((f"{value:.5f}" if math.isfinite(value) else "NTC 无效") if self.online or self.demo else "—")
        now = time.monotonic()
        if now - self.last_rate_time >= 1:
            rate = (self.received - self.last_rate_count) / (now - self.last_rate_time)
            self.rate_label.setText(f"{rate:.0f} 帧/s · {len(self.rows)} / 50000 · CRC {self.link.parser.crc_errors}")
            self.last_rate_count = self.received
            self.last_rate_time = now

    def set_preset(self, index):
        if not hasattr(self, "visible_checks"):
            return
        selected = [(0, 1), (2,), (3, 4), (0, 1, 2, 3, 4, 5), (5,)][index]
        for i, check in enumerate(self.visible_checks):
            check.setChecked(i in selected)
            self.curves[i].setVisible(i in selected)
        self.refresh_display_units()
        self.plot.disableAutoRange()
        bounds = [(-1, 1), (-200, 200), (0, 360), (-200, 360), (0, 100)][index]
        self.plot.setYRange(*bounds, padding=0)

    def convert_display(self, channel, value):
        choice = self.unit_combos[channel].currentIndex() if channel in self.unit_combos else 0
        gear, poles = self.meta.gear_ratio, self.meta.pole_pairs
        if channel == 1:
            return value * [1, self.meta.kt, self.meta.kt*gear][choice]
        if channel == 2:
            return value * [1, gear, gear*math.pi/30, gear*poles*math.pi/30, math.pi/30][choice]
        if channel == 3:
            return value * [1, math.pi/180, gear, gear*math.pi/180, gear*poles][choice]
        if channel == 4:
            if choice in (2, 3):
                angle = (value * poles) % 360
                return angle if choice == 2 else angle*math.pi/180
            return value * [1, math.pi/180, 1, 1, 1/gear][choice]
        return value

    def refresh_display_units(self, *_):
        if not hasattr(self, "display_gains") or len(self.display_gains) != len(CHANNELS):
            return
        selected = []
        for i, check in enumerate(self.visible_checks):
            name, unit, _ = CHANNELS[i]
            title = self.unit_combos[i].currentText() if i in self.unit_combos else f"{name} / {unit}"
            check.setText(title)
            if check.isChecked():
                selected.append((title, self.display_gains[i].value()))
        if len(selected) == 1 and selected[0][1] == 1:
            axis = selected[0][0]
        elif selected and all(title.endswith(" / A") and gain == 1 for title, gain in selected):
            axis = "电流 / A"
        else:
            axis = "显示值 = 换算值 × 各通道增益"
        self.plot.setLabel("right", axis)

    @staticmethod
    def zoom_position(span, minimum, maximum):
        span = min(maximum, max(minimum, span))
        return round(1000 * math.log(maximum / span) / math.log(maximum / minimum))

    def sync_zoom_controls(self, *_):
        (xlo, xhi), (ylo, yhi) = self.plot.viewRange()
        xspan, yspan = xhi - xlo, yhi - ylo
        for slider, span, minimum, maximum in (
                (self.x_zoom_slider, xspan, .1, 240),
                (self.y_zoom_slider, yspan, .002, 200000)):
            blocked = slider.blockSignals(True)
            slider.setValue(self.zoom_position(span, minimum, maximum))
            slider.blockSignals(blocked)
        blocked = self.window_seconds.blockSignals(True)
        self.window_seconds.setValue(xspan)
        self.window_seconds.blockSignals(blocked)
        self.x_zoom_label.setText(f"窗口 {xspan:.2f} s")
        self.y_zoom_label.setText(f"跨度 {yspan:.4g}")

    def zoom_horizontal(self, position):
        self.window_seconds.setValue(240 * (.1 / 240) ** (position / 1000))
        # Apply even if rounding leaves the numeric value unchanged.
        self.set_time_window(self.window_seconds.value())

    def set_time_window(self, seconds):
        lo, hi = self.plot.viewRange()[0]
        right = 0 if self.follow.isChecked() else (lo + hi) / 2 + seconds / 2
        self.plot.setXRange(right - seconds, right, padding=0)

    def zoom_vertical(self, position):
        span = 200000 * (.002 / 200000) ** (position / 1000)
        lo, hi = self.plot.viewRange()[1]
        center = (lo + hi) / 2
        self.plot.setYRange(center - span / 2, center + span / 2, padding=0)

    def fit_vertical(self):
        lo, hi = self.plot.viewRange()[0]
        values = []
        for check, curve in zip(self.visible_checks, self.curves):
            if check.isChecked():
                x, y = curve.getData()
                if x is not None:
                    values.extend(y[(x >= lo) & (x <= hi) & np.isfinite(y)])
        if values:
            low, high = min(values), max(values)
            margin = max((high - low) * .1, .01)
            self.plot.setYRange(low - margin, high + margin, padding=0)

    def toggle_cursors(self, checked):
        self.cursor_a.setVisible(checked)
        self.cursor_b.setVisible(checked)
        if checked:
            lo, hi = self.plot.viewRange()[0]
            self.cursor_a.setValue(lo + (hi - lo) * .3)
            self.cursor_b.setValue(lo + (hi - lo) * .7)

    def update_cursors(self):
        dt = abs(self.cursor_b.value() - self.cursor_a.value())
        self.cursor_label.setText(f"Δt {dt * 1000:.2f} ms" + (f" · {1/dt:.2f} Hz" if dt > 1e-6 else ""))

    def change_rate(self, _index):
        if not self.loading and self.ready and not self.demo:
            self.link.send(RATE, struct.pack("<H", self.rate_combo.currentData()))

    def clear_plot(self):
        self.rows.clear()
        self.last_ms = None
        self.time_offset = 0.0
        self.latest_temperature = float("nan")
        for curve in self.curves:
            curve.setData([], [])

    def log(self, text):
        self.log_box.appendPlainText(time.strftime("%H:%M:%S") + "  " + text)

    def export_csv(self):
        if not self.rows:
            self.log("暂无波形可导出。")
            return
        path, _ = W.QFileDialog.getSaveFileName(self, "导出当前缓存", "foc_capture.csv", "CSV (*.csv)")
        if path:
            try:
                with open(path, "w", newline="", encoding="utf-8-sig") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(["device_time_s", "id_A", "iq_A", "output_rpm", "output_deg", "encoder_deg", "ntc_C_5Hz_held"])
                    writer.writerows(self.rows)
                self.log(f"已导出 {len(self.rows)} 帧：{path}")
            except OSError as exc:
                self.log(f"导出失败：{exc}")

    def save_config(self):
        path, _ = W.QFileDialog.getSaveFileName(self, "保存调参方案", "foc_settings.json", "JSON (*.json)")
        if path:
            try:
                content = {"format": 1, "config": asdict(self.collect_config()), "motor": asdict(self.meta)}
                Path(path).write_text(json.dumps(content, ensure_ascii=False, indent=2), encoding="utf-8")
                self.log("方案已保存到电脑。")
            except (ValueError, OSError) as exc:
                self.log(str(exc))

    def load_config(self):
        if self.status.get("state", 0) != 0:
            self.log("请先停机再载入方案。")
            return
        path, _ = W.QFileDialog.getOpenFileName(self, "载入调参方案", "", "JSON (*.json)")
        if path:
            try:
                content = json.loads(Path(path).read_text(encoding="utf-8"))
                if content.get("format") != 1:
                    raise ValueError("不支持的方案格式")
                config = Config(**content["config"])
                config.pack()
                # Loading a file edits controls only; the board model remains authoritative.
                applied = self.config
                self.load_inputs(config)
                self.config = applied
                self.update_gain_preview()
                self.log("方案已载入界面，尚未下发。")
            except (OSError, ValueError, TypeError, KeyError) as exc:
                self.log(f"载入失败：{exc}")

    def set_demo(self, enabled):
        if enabled:
            self.link.close()
        self.demo = enabled
        self.ready = False
        self.clear_plot()
        if enabled:
            self.status = dict(state=0, run=0, fault=0, mode=4, align=1, offset=1, encoder=1,
                               vbus=48, temperature=29, loop_us=24)
            self.load_inputs(Config(mode=4, spd_rpm=240))
            self.actual_gains = self.meta.gains(self.config)
            self.show_gains()
            self.banner.setText("演示模式 · 所有波形和状态均为模拟数据 · 不发送串口指令")
            self.demo_timer.start(10)
            self.update_status()
        else:
            self.demo_timer.stop()
            self.status = {}
            self.state_label.setText("● 未连接")
            self.banner.setText("连接后读取控制板参数；启动不会自动发生。")
        self.connect_button.setEnabled(not enabled)
        self.update_enables()

    def demo_tick(self):
        for _ in range(2):
            self.demo_ms += 5
            t = self.demo_ms / 1000
            state = self.status.get("state", 0)
            if state == 1 and self.demo_ms - self.demo_start_ms >= 800:
                self.status["state"] = state = 2
                if self.config.mode == 5:
                    self.config.pos_deg = self.demo_pos % 360
                    self.inputs["pos_deg"].setValue(self.config.pos_deg)
            target = 0
            if state == 2:
                if self.config.mode == 4:
                    target = self.config.spd_rpm / self.meta.gear_ratio
                elif self.config.mode == 5:
                    error = (self.config.pos_deg - self.demo_pos + 180) % 360 - 180
                    limit = self.config.pos_speed_limit * 60 / (2 * math.pi)
                    target = max(-limit, min(limit, error * .8))
                elif self.config.mode == 3:
                    target = self.config.iq_a * 30
                elif self.config.mode == 2:
                    target = self.config.uq_v * 8
            self.demo_speed += (target - self.demo_speed) * .03
            self.demo_pos += self.demo_speed * 6 * .005
            current_d = 2.5 if state == 1 else .02 * math.sin(2 * math.pi * 17 * t) if state == 2 else 0
            current_q = (self.config.iq_a if self.config.mode == 3 else self.demo_speed * .005) if state == 2 else 0
            current_q += .008 * math.sin(2 * math.pi * 23 * t) if state == 2 else 0
            self.add_sample(self.demo_ms, [current_d, current_q, self.demo_speed,
                                          self.demo_pos % 360, (self.demo_pos * self.meta.gear_ratio) % 360])
        self.status["mode"] = self.config.mode
        self.update_status()

    def play_click(self, *_args):
        if os.name == "nt" and self.sound_check.isChecked():
            import winsound
            try:
                winsound.PlaySound(str(ASSETS / "click.wav"), winsound.SND_FILENAME | winsound.SND_ASYNC | winsound.SND_NODEFAULT)
            except RuntimeError:
                pass  # Audio availability must not affect motor commands.

    def closeEvent(self, event):
        self.link.close()
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="MPS FOC Studio")
    parser.add_argument("--demo", action="store_true")
    parser.add_argument("--screenshot", type=Path, help="Capture a simulated preview then exit")
    args = parser.parse_args()
    if os.name == "nt":
        import ctypes
        # Set before creating any windows so Windows groups this as our app.
        set_app_id = ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID
        set_app_id.argtypes = [ctypes.c_wchar_p]
        set_app_id.restype = ctypes.c_long
        set_app_id("RivenHuang.MPSFOCStudio.Desktop")
    app = W.QApplication(sys.argv[:1])
    # Explicit registration also gives Qt's offscreen renderer Chinese glyphs.
    if os.name == "nt":
        fonts = Path(os.environ.get("WINDIR", "C:/Windows")) / "Fonts"
        for name in ("msyh.ttc", "msyhbd.ttc", "consola.ttf", "segoeui.ttf"):
            if (fonts / name).is_file():
                QtGui.QFontDatabase.addApplicationFont(str(fonts / name))
        app.setFont(QtGui.QFont("Microsoft YaHei UI", 9))
    app.setApplicationName("MPS FOC Studio")
    app.setWindowIcon(QtGui.QIcon(str(ASSETS / "feibi.png")))
    window = MainWindow(demo=args.demo or bool(args.screenshot))
    window.show()
    if args.screenshot:
        # Offscreen preview never opens a COM port.
        window.run_motor()
        for _ in range(500):
            window.demo_tick()
        window.draw()
        def capture():
            args.screenshot.parent.mkdir(parents=True, exist_ok=True)
            window.grab().save(str(args.screenshot))
            app.quit()
        QtCore.QTimer.singleShot(500, capture)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
