#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import time

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rqt_gui_py.plugin import Plugin

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QWidget,
)

from spinal_msgs.msg import ESCTelemetryArray, PwmTest, Pwms
from std_msgs.msg import Float32


DISCOVERY_TIMEOUT_SEC = 2.0
MOTOR_COUNT = 4
IDLE_DUTY = 0.5
MAX_DUTY = 1.0
COMMAND_UNIT_PERCENT = 'percent'
COMMAND_UNIT_DUTY = 'duty'
PERCENT_SLIDER_SCALE = 10.0
DUTY_SLIDER_SCALE = 1000.0


def _ns_join(ns: str, name: str) -> str:
    if not ns:
        return '/' + name if not name.startswith('/') else name
    return f'{ns.rstrip("/")}/{name.lstrip("/")}'


def _best_effort_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))


def _percent_to_duty(percent: float) -> float:
    percent = _clamp(percent, 0.0, 100.0)
    return IDLE_DUTY + (MAX_DUTY - IDLE_DUTY) * percent / 100.0


def _duty_to_percent(duty: float) -> float:
    duty = _clamp(duty, IDLE_DUTY, MAX_DUTY)
    return (duty - IDLE_DUTY) * 100.0 / (MAX_DUTY - IDLE_DUTY)


class ThrusterMonitor(Plugin):
    def __init__(self, context):
        super(ThrusterMonitor, self).__init__(context)
        self.setObjectName('ThrusterMonitor')

        parser = argparse.ArgumentParser()
        parser.add_argument('-q', '--quiet', action='store_true',
                            dest='quiet', help='Put plugin in silent mode')
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print(f'arguments: {args}')
            print(f'unknowns: {unknowns}')

        self._rclpy_initialized_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._rclpy_initialized_here = True

        self.node = rclpy.create_node(f'thruster_monitor_rqt_{id(self)}')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(10)

        robot_ns = self._discover_robot_ns()

        self.pwm_test_pub_ = self.node.create_publisher(
            PwmTest, _ns_join(robot_ns, 'pwm_test'), 1)
        self.motor_pwms_sub_ = self.node.create_subscription(
            Pwms, _ns_join(robot_ns, 'motor_pwms'), self.motorPwmsCallback,
            _best_effort_qos(10))
        self.esc_telem_sub_ = self.node.create_subscription(
            ESCTelemetryArray, _ns_join(robot_ns, 'esc_telem'), self.escTelemetryCallback,
            _best_effort_qos(10))
        self.battery_voltage_sub_ = self.node.create_subscription(
            Float32, _ns_join(robot_ns, 'battery_voltage_status'), self.batteryVoltageCallback,
            _best_effort_qos(10))

        self._widget = QWidget()
        ui_file = os.path.join(get_package_share_directory('spinal'),
                               'resource', 'thruster_monitor.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ThrusterMonitor')

        self._syncing_controls = False
        self._test_mode_requested = False
        self._command_duty = [IDLE_DUTY] * MOTOR_COUNT
        self._sliders = []
        self._spin_boxes = []
        self._table_headers = [
            'throttle [%]',
            'duty',
            'temperature [C]',
            'esc voltage [V]',
            'current [A]',
            'consumption [mAh]',
            'rpm',
            'crc error',
        ]

        self._buildUi()

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + f' ({context.serial_number()})')
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        if getattr(self, 'spin_timer', None) is not None:
            self.spin_timer.stop()
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass
        try:
            self.node.destroy_node()
        except Exception:
            pass
        if self._rclpy_initialized_here and rclpy.ok():
            rclpy.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass

    def _spin_once(self):
        try:
            self.executor.spin_once(timeout_sec=0.0)
        except Exception:
            pass

    def _discover_robot_ns(self, timeout_sec: float = DISCOVERY_TIMEOUT_SEC) -> str:
        deadline = time.monotonic() + timeout_sec
        last_error = None
        topic_suffixes = (
            '/motor_pwms',
            '/pwm_test',
            '/esc_telem',
            '/battery_voltage_status',
        )

        while time.monotonic() <= deadline:
            try:
                topics = self.node.get_topic_names_and_types()
                for topic_name, _types in topics:
                    if topic_name in topic_suffixes or topic_name.endswith(topic_suffixes):
                        for suffix in topic_suffixes:
                            if topic_name == suffix or topic_name.endswith(suffix):
                                robot_ns = topic_name[:-len(suffix)]
                                self.node.get_logger().info(
                                    f"Detected robot namespace: '{robot_ns}' from topic '{topic_name}'")
                                return robot_ns
            except Exception as e:
                last_error = e

            try:
                self.executor.spin_once(timeout_sec=0.05)
            except Exception as e:
                last_error = e

        if last_error is not None:
            self.node.get_logger().warn(
                f"Topic discovery failed: {last_error}. Using root namespace.")
        else:
            self.node.get_logger().warn(
                "No thruster topic found before timeout. Using root namespace.")
        return ''

    def _buildUi(self):
        self._widget.mainLayout.setContentsMargins(6, 6, 6, 6)
        self._widget.mainLayout.setSpacing(8)

        status_group = QGroupBox('status', self._widget)
        status_layout = QGridLayout(status_group)
        status_layout.setContentsMargins(8, 8, 8, 8)
        status_layout.setSpacing(8)

        self.modeValueLabel = QLabel('idle', status_group)
        self.modeValueLabel.setFrameShape(QFrame.StyledPanel)
        self.modeValueLabel.setAlignment(Qt.AlignCenter)
        self.batteryVoltageValueLabel = QLabel('-- V', status_group)
        self.batteryVoltageValueLabel.setFrameShape(QFrame.StyledPanel)
        self.batteryVoltageValueLabel.setAlignment(Qt.AlignCenter)

        status_layout.addWidget(QLabel('mode', status_group), 0, 0)
        status_layout.addWidget(self.modeValueLabel, 0, 1)
        status_layout.addWidget(QLabel('battery voltage', status_group), 0, 2)
        status_layout.addWidget(self.batteryVoltageValueLabel, 0, 3)
        status_layout.setColumnStretch(1, 1)
        status_layout.setColumnStretch(3, 1)
        self._widget.mainLayout.addWidget(status_group)

        control_group = QGroupBox('command', self._widget)
        control_layout = QGridLayout(control_group)
        control_layout.setContentsMargins(8, 8, 8, 8)
        control_layout.setSpacing(8)

        self.allMotorCheckBox = QCheckBox('all motors', control_group)
        self.allMotorCheckBox.stateChanged.connect(self._allMotorModeChanged)
        self.commandUnitComboBox = QComboBox(control_group)
        self.commandUnitComboBox.addItem('%', COMMAND_UNIT_PERCENT)
        self.commandUnitComboBox.addItem('duty', COMMAND_UNIT_DUTY)
        self.commandUnitComboBox.currentIndexChanged.connect(self._commandUnitChanged)
        self.testModeButton = QPushButton('test mode', control_group)
        self.testModeButton.clicked.connect(self._testModeButtonClicked)
        self.idleModeButton = QPushButton('idle mode', control_group)
        self.idleModeButton.clicked.connect(self._idleModeButtonClicked)

        button_layout = QHBoxLayout()
        button_layout.addWidget(self.allMotorCheckBox)
        button_layout.addStretch(1)
        button_layout.addWidget(QLabel('unit', control_group))
        button_layout.addWidget(self.commandUnitComboBox)
        button_layout.addWidget(self.testModeButton)
        button_layout.addWidget(self.idleModeButton)
        control_layout.addLayout(button_layout, 0, 0, 1, 3)

        control_layout.addWidget(QLabel('motor', control_group), 1, 0)
        control_layout.addWidget(QLabel('throttle', control_group), 1, 1)
        self.commandUnitHeaderLabel = QLabel('%', control_group)
        control_layout.addWidget(self.commandUnitHeaderLabel, 1, 2)

        for i in range(MOTOR_COUNT):
            label = QLabel(f'motor{i + 1}', control_group)
            slider = QSlider(Qt.Horizontal, control_group)
            slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

            spin = QDoubleSpinBox(control_group)

            slider.valueChanged.connect(lambda value, index=i: self._sliderChanged(index, value))
            spin.valueChanged.connect(lambda value, index=i: self._spinBoxChanged(index, value))

            self._sliders.append(slider)
            self._spin_boxes.append(spin)

            row = i + 2
            control_layout.addWidget(label, row, 0)
            control_layout.addWidget(slider, row, 1)
            control_layout.addWidget(spin, row, 2)

        control_layout.setColumnStretch(1, 1)
        self._configureCommandControls()
        self._widget.mainLayout.addWidget(control_group)

        telemetry_group = QGroupBox('telemetry', self._widget)
        telemetry_layout = QGridLayout(telemetry_group)
        telemetry_layout.setContentsMargins(8, 8, 8, 8)
        self.telemetryTableWidget = QTableWidget(MOTOR_COUNT, len(self._table_headers), telemetry_group)
        self.telemetryTableWidget.setHorizontalHeaderLabels(self._table_headers)
        self.telemetryTableWidget.setVerticalHeaderLabels([f'motor{i + 1}' for i in range(MOTOR_COUNT)])
        self.telemetryTableWidget.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.telemetryTableWidget.setSelectionMode(QAbstractItemView.NoSelection)
        for row in range(MOTOR_COUNT):
            for col in range(len(self._table_headers)):
                self._setTableItem(row, col, '--')
        telemetry_layout.addWidget(self.telemetryTableWidget, 0, 0)
        self._widget.mainLayout.addWidget(telemetry_group, 1)

    def _sliderChanged(self, index: int, value: int):
        if self._syncing_controls:
            return
        duty = self._sliderValueToDuty(value)
        self._syncControl(index, duty)
        self._publishTestCommandIfEnabled()

    def _spinBoxChanged(self, index: int, value: float):
        if self._syncing_controls:
            return
        duty = self._uiValueToDuty(value)
        self._syncControl(index, duty)
        self._publishTestCommandIfEnabled()

    def _syncControl(self, index: int, duty: float):
        duty = _clamp(duty, IDLE_DUTY, MAX_DUTY)
        self._syncing_controls = True
        self._setControlDuty(index, duty)
        if self.allMotorCheckBox.isChecked():
            for i in range(MOTOR_COUNT):
                self._setControlDuty(i, duty)
        self._syncing_controls = False

    def _allMotorModeChanged(self, _state):
        if self.allMotorCheckBox.isChecked():
            self._syncControl(0, self._command_duty[0])
            self._publishTestCommandIfEnabled()

    def _commandUnitChanged(self, _index):
        self._configureCommandControls()

    def _testModeButtonClicked(self):
        self._test_mode_requested = True
        self._publishTestCommand()

    def _idleModeButtonClicked(self):
        self._test_mode_requested = False
        self.pwm_test_pub_.publish(PwmTest())

    def _publishTestCommandIfEnabled(self):
        if self._test_mode_requested:
            self._publishTestCommand()

    def _publishTestCommand(self):
        msg = PwmTest()
        msg.motor_index = list(range(MOTOR_COUNT))
        if self.allMotorCheckBox.isChecked():
            msg.pwms = [self._command_duty[0]] * MOTOR_COUNT
        else:
            msg.pwms = list(self._command_duty)
        self.pwm_test_pub_.publish(msg)

    def _currentCommandUnit(self):
        unit = self.commandUnitComboBox.itemData(self.commandUnitComboBox.currentIndex())
        return unit if unit in (COMMAND_UNIT_PERCENT, COMMAND_UNIT_DUTY) else COMMAND_UNIT_PERCENT

    def _configureCommandControls(self):
        unit = self._currentCommandUnit()
        self._syncing_controls = True
        if unit == COMMAND_UNIT_DUTY:
            self.commandUnitHeaderLabel.setText('duty')
            for slider, spin in zip(self._sliders, self._spin_boxes):
                slider.setRange(int(IDLE_DUTY * DUTY_SLIDER_SCALE), int(MAX_DUTY * DUTY_SLIDER_SCALE))
                slider.setSingleStep(1)
                slider.setPageStep(10)
                spin.setRange(IDLE_DUTY, MAX_DUTY)
                spin.setDecimals(3)
                spin.setSingleStep(0.001)
                spin.setSuffix('')
        else:
            self.commandUnitHeaderLabel.setText('%')
            for slider, spin in zip(self._sliders, self._spin_boxes):
                slider.setRange(0, int(100 * PERCENT_SLIDER_SCALE))
                slider.setSingleStep(int(1 * PERCENT_SLIDER_SCALE))
                slider.setPageStep(int(10 * PERCENT_SLIDER_SCALE))
                spin.setRange(0.0, 100.0)
                spin.setDecimals(1)
                spin.setSingleStep(1.0)
                spin.setSuffix(' %')

        for i, duty in enumerate(self._command_duty):
            self._setControlDuty(i, duty)
        self._syncing_controls = False

    def _setControlDuty(self, index: int, duty: float):
        duty = _clamp(duty, IDLE_DUTY, MAX_DUTY)
        self._command_duty[index] = duty
        self._sliders[index].setValue(self._dutyToSliderValue(duty))
        self._spin_boxes[index].setValue(self._dutyToUiValue(duty))

    def _sliderValueToDuty(self, value: int) -> float:
        if self._currentCommandUnit() == COMMAND_UNIT_DUTY:
            return _clamp(float(value) / DUTY_SLIDER_SCALE, IDLE_DUTY, MAX_DUTY)
        return _percent_to_duty(float(value) / PERCENT_SLIDER_SCALE)

    def _uiValueToDuty(self, value: float) -> float:
        if self._currentCommandUnit() == COMMAND_UNIT_DUTY:
            return _clamp(value, IDLE_DUTY, MAX_DUTY)
        return _percent_to_duty(value)

    def _dutyToUiValue(self, duty: float) -> float:
        if self._currentCommandUnit() == COMMAND_UNIT_DUTY:
            return _clamp(duty, IDLE_DUTY, MAX_DUTY)
        return _duty_to_percent(duty)

    def _dutyToSliderValue(self, duty: float) -> int:
        if self._currentCommandUnit() == COMMAND_UNIT_DUTY:
            return int(round(_clamp(duty, IDLE_DUTY, MAX_DUTY) * DUTY_SLIDER_SCALE))
        return int(round(_duty_to_percent(duty) * PERCENT_SLIDER_SCALE))

    def motorPwmsCallback(self, msg: Pwms):
        self._setModeLabel(int(msg.control_mode))

        motor_values = list(msg.motor_value)
        for i in range(MOTOR_COUNT):
            if i >= len(motor_values):
                self._setTableItem(i, self._table_headers.index('throttle [%]'), '--')
                self._setTableItem(i, self._table_headers.index('duty'), '--')
                continue

            duty = float(motor_values[i]) / 2000.0
            percent = _duty_to_percent(duty)
            self._setTableItem(i, self._table_headers.index('throttle [%]'), f'{percent:.1f}')
            self._setTableItem(i, self._table_headers.index('duty'), f'{duty:.3f}')

    def escTelemetryCallback(self, msg: ESCTelemetryArray):
        telemetry = [
            msg.esc_telemetry_1,
            msg.esc_telemetry_2,
            msg.esc_telemetry_3,
            msg.esc_telemetry_4,
        ]
        for i, esc in enumerate(telemetry):
            self._setTableItem(i, self._table_headers.index('temperature [C]'), str(esc.temperature))
            self._setTableItem(i, self._table_headers.index('esc voltage [V]'), f'{esc.voltage / 100.0:.2f}')
            self._setTableItem(i, self._table_headers.index('current [A]'), f'{esc.current / 100.0:.2f}')
            self._setTableItem(i, self._table_headers.index('consumption [mAh]'), str(esc.consumption))
            self._setTableItem(i, self._table_headers.index('rpm'), str(esc.rpm))
            self._setTableItem(i, self._table_headers.index('crc error'), str(esc.crc_error))

    def batteryVoltageCallback(self, msg: Float32):
        self.batteryVoltageValueLabel.setText(f'{msg.data:.2f} V')

    def _setModeLabel(self, control_mode: int):
        labels = {
            getattr(Pwms, 'CONTROL_MODE_NONE', 0): ('idle', QColor(210, 210, 210)),
            getattr(Pwms, 'CONTROL_MODE_TEST', 1): ('test', QColor(255, 220, 120)),
            getattr(Pwms, 'CONTROL_MODE_START', 2): ('start control', QColor(120, 220, 160)),
        }
        text, color = labels.get(control_mode, (f'unknown ({control_mode})', QColor(255, 160, 160)))
        self.modeValueLabel.setText(text)
        self.modeValueLabel.setStyleSheet(
            f'QLabel {{ background-color: rgb({color.red()}, {color.green()}, {color.blue()}); }}')
        if control_mode == getattr(Pwms, 'CONTROL_MODE_START', 2):
            self._test_mode_requested = False

    def _setTableItem(self, row: int, col, value):
        if isinstance(col, str):
            col = self._table_headers.index(col)
        item = self.telemetryTableWidget.item(row, col)
        if item is None:
            item = QTableWidgetItem()
            self.telemetryTableWidget.setItem(row, col, item)
        item.setText(str(value))
