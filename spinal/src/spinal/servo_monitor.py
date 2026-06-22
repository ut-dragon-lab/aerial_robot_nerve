#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import time
from functools import reduce

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rqt_gui_py.plugin import Plugin

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QPointF, Qt, QRectF, QSize, Signal, QTimer
from python_qt_binding.QtGui import QBrush, QColor, QFont, QFontMetrics, QPainter, QPen
from python_qt_binding.QtWidgets import (
    QAction,
    QCheckBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSizePolicy,
    QSplitter,
    QSpinBox,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from spinal_msgs.msg import ServoControlCmd, ServoStates, ServoTorqueCmd, ServoTorqueStates
from spinal_msgs.srv import GetBoardInfo, SetBoardConfig, SetDirectServoConfig


DISCOVERY_TIMEOUT_SEC = 2.0
SERVICE_WAIT_TIMEOUT_SEC = 2.0
SERVO_POSITION_MIN = 0
SERVO_POSITION_MAX = 4095
SERVO_POSITION_RANGE = SERVO_POSITION_MAX - SERVO_POSITION_MIN + 1
DEFAULT_HOMING_OFFSET = 2048


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


def _position_to_degrees(position) -> float:
    if position is None:
        return 0.0
    return (int(position) % SERVO_POSITION_RANGE) * 360.0 / SERVO_POSITION_RANGE


class ServoDialWidget(QWidget):
    targetPositionChanged = Signal(int, bool)
    targetPositionCommitted = Signal(int)

    def __init__(self, parent=None):
        super(ServoDialWidget, self).__init__(parent)
        self.setMinimumSize(160, 160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self._current_position = None
        self._target_position = 2047
        self._dragging = False

    def sizeHint(self):
        return QSize(260, 260)

    def setCurrentPosition(self, position):
        self._current_position = None if position is None else int(position)
        self.update()

    def setTargetPosition(self, position):
        self._target_position = max(SERVO_POSITION_MIN, min(SERVO_POSITION_MAX, int(position)))
        self.update()

    def paintEvent(self, _event):
        side = max(40, min(self.width(), self.height()) - 14)
        radius = side / 2.0
        center = self.rect().center()
        ring_rect = QRectF(center.x() - radius, center.y() - radius, side, side)

        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), self.palette().window())

        painter.setPen(Qt.NoPen)
        painter.setBrush(QBrush(QColor(38, 41, 47)))
        painter.drawEllipse(ring_rect)

        inner_radius = radius * 0.76
        inner_rect = QRectF(center.x() - inner_radius, center.y() - inner_radius,
                            inner_radius * 2.0, inner_radius * 2.0)
        painter.setBrush(QBrush(QColor(216, 220, 221)))
        painter.drawEllipse(inner_rect)

        core_radius = radius * 0.54
        core_rect = QRectF(center.x() - core_radius, center.y() - core_radius,
                           core_radius * 2.0, core_radius * 2.0)
        painter.setBrush(QBrush(QColor(43, 45, 51)))
        painter.drawEllipse(core_rect)

        self._drawNeedle(painter, center, radius * 0.78, self._target_position,
                         QColor(18, 210, 89), 5)
        if self._current_position is not None:
            self._drawNeedle(painter, center, radius * 0.64, self._current_position,
                             QColor(255, 37, 37), 5)

        self._drawCenterText(painter, core_rect)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self._dragging = True
            self._setTargetFromPoint(event.pos(), True)

    def mouseMoveEvent(self, event):
        if self._dragging:
            self._setTargetFromPoint(event.pos(), True)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self._dragging:
            self._dragging = False
            self._setTargetFromPoint(event.pos(), False)
            self.targetPositionCommitted.emit(self._target_position)

    def _setTargetFromPoint(self, point, dragging):
        center = self.rect().center()
        rad = math.atan2(point.y() - center.y(), point.x() - center.x())
        normalized = (rad + math.pi / 2.0) % (2.0 * math.pi)
        # Dynamixel positions increase counter-clockwise; Qt screen coordinates
        # make positive atan2 angles appear clockwise unless this is inverted.
        ccw_normalized = (2.0 * math.pi - normalized) % (2.0 * math.pi)
        position = int(round(ccw_normalized * SERVO_POSITION_RANGE / (2.0 * math.pi)))
        position = max(SERVO_POSITION_MIN, min(SERVO_POSITION_MAX, position))
        if position == self._target_position:
            return
        self._target_position = position
        self.update()
        self.targetPositionChanged.emit(position, dragging)

    def _drawNeedle(self, painter, center, length, position, color, width):
        rad = (int(position) % SERVO_POSITION_RANGE) * 2.0 * math.pi / SERVO_POSITION_RANGE
        rad = -rad - math.pi / 2.0
        end_x = center.x() + math.cos(rad) * length
        end_y = center.y() + math.sin(rad) * length
        painter.setPen(QPen(color, width, Qt.SolidLine, Qt.RoundCap))
        painter.drawLine(QPointF(center), QPointF(end_x, end_y))

    def _drawCenterText(self, painter, core_rect):
        painter.setPen(QPen(QColor(42, 230, 218)))

        if self._current_position is None:
            large_text = '--'
            small_text = 'current'
        else:
            large_text = f'{_position_to_degrees(self._current_position):.1f}'
            small_text = f'deg / {int(self._current_position)} bit'

        large_rect = QRectF(
            core_rect.left() + core_rect.width() * 0.08,
            core_rect.top() + core_rect.height() * 0.18,
            core_rect.width() * 0.84,
            core_rect.height() * 0.42)
        small_rect = QRectF(
            core_rect.left() + core_rect.width() * 0.10,
            core_rect.top() + core_rect.height() * 0.62,
            core_rect.width() * 0.80,
            core_rect.height() * 0.22)

        large_font = self._fitFont(large_text, large_rect, int(core_rect.height() * 0.38), 12)
        small_font = self._fitFont(small_text, small_rect, int(core_rect.height() * 0.14), 8)

        painter.setFont(large_font)
        painter.drawText(large_rect, Qt.AlignCenter, large_text)
        painter.setFont(small_font)
        painter.drawText(small_rect, Qt.AlignCenter, small_text)

    def _fitFont(self, text, rect, max_pixel_size, min_pixel_size):
        font = QFont('', max_pixel_size, QFont.Bold)
        max_pixel_size = max(min_pixel_size, max_pixel_size)
        for pixel_size in range(max_pixel_size, min_pixel_size - 1, -1):
            font.setPixelSize(pixel_size)
            metrics = QFontMetrics(font)
            width = (metrics.horizontalAdvance(text)
                     if hasattr(metrics, 'horizontalAdvance') else metrics.width(text))
            if width <= rect.width() and metrics.height() <= rect.height():
                return QFont(font)
        font.setPixelSize(min_pixel_size)
        return font


class ServoMonitor(Plugin):
    def __init__(self, context):
        super(ServoMonitor, self).__init__(context)
        self.setObjectName('ServoMonitor')

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

        self.node = rclpy.create_node(f'servo_monitor_rqt_{id(self)}')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(10)

        robot_ns = self._discover_robot_ns()

        self.get_board_info_client_ = self.node.create_client(
            GetBoardInfo, _ns_join(robot_ns, 'get_board_info'))
        self.set_board_config_client_ = self.node.create_client(
            SetBoardConfig, _ns_join(robot_ns, 'set_board_config'))
        self.set_direct_servo_config_client_ = self.node.create_client(
            SetDirectServoConfig, _ns_join(robot_ns, 'direct_servo_config'))
        self.servo_control_pub_ = self.node.create_publisher(
            ServoControlCmd, _ns_join(robot_ns, 'servo/target_states'), 1)
        self.servo_torque_pub_ = self.node.create_publisher(
            ServoTorqueCmd, _ns_join(robot_ns, 'servo/torque_enable'), 1)

        self._widget = QWidget()
        ui_file = os.path.join(get_package_share_directory('spinal'),
                               'resource', 'servo_monitor.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ServoMonitor')

        self._widget.boardInfoUpdateButton.clicked.connect(self.updateButtonCallback)
        self._widget.allServoOnButton.clicked.connect(self.allServoOnButtonCallback)
        self._widget.allServoOffButton.clicked.connect(self.allServoOffButtonCallback)

        self._widget.servoTableWidget.setContextMenuPolicy(Qt.ActionsContextMenu)
        servo_on_action = QAction('servo on', self._widget.servoTableWidget)
        servo_on_action.triggered.connect(self.servoOn)
        self._widget.servoTableWidget.addAction(servo_on_action)
        servo_off_action = QAction('servo off', self._widget.servoTableWidget)
        servo_off_action.triggered.connect(self.servoOff)
        self._widget.servoTableWidget.addAction(servo_off_action)
        joint_calib_action = QAction('joint calib', self._widget.servoTableWidget)
        joint_calib_action.triggered.connect(self.jointCalib)
        self._widget.servoTableWidget.addAction(joint_calib_action)
        board_reboot_action = QAction('board reboot', self._widget.servoTableWidget)
        board_reboot_action.triggered.connect(self.boardReboot)
        self._widget.servoTableWidget.addAction(board_reboot_action)

        self._table_data = []
        self._servo_num = 0
        self._selected_servo_index = None
        self._last_target_pub_time = 0.0
        self._syncing_target_control = False
        self._syncing_homing_offset_control = False
        self._headers = [
            'torque', 'joint name', 'board', 'index', 'id', 'angle',
            'temperature', 'load', 'error', 'pid_gains', 'profile_velocity',
            'current_limit', 'send_data_flag'
        ]

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + f' ({context.serial_number()})')
        context.add_widget(self._widget)

        self._widget.setLayout(self._widget.gridLayout)
        self._installPositionControlPanel()
        self._widget.servoTableWidget.currentCellChanged.connect(self._selectedServoChanged)
        self.joint_id_name_map = self._load_joint_id_name_map(robot_ns)

        self.servo_state_sub_ = self.node.create_subscription(
            ServoStates,
            _ns_join(robot_ns, 'servo/states'),
            self.servoStateCallback,
            _best_effort_qos(10))
        self.servo_torque_state_sub_ = self.node.create_subscription(
            ServoTorqueStates,
            _ns_join(robot_ns, 'servo/torque_states'),
            self.servoTorqueStatesCallback,
            _best_effort_qos(10))

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.updateTable)
        self.update_timer.start(1000)

        self.updateButtonCallback()

    def shutdown_plugin(self):
        for timer in (getattr(self, 'spin_timer', None), getattr(self, 'update_timer', None)):
            if timer is not None:
                timer.stop()
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
        while time.monotonic() <= deadline:
            try:
                services = self.node.get_service_names_and_types()
                candidates = [
                    name for name, _types in services
                    if name == '/get_board_info' or name.endswith('/get_board_info')
                ]
                if candidates:
                    robot_ns = candidates[0].split('/get_board_info')[0]
                    self.node.get_logger().info(
                        f"Detected robot namespace: '{robot_ns}' from service '{candidates[0]}'")
                    return robot_ns
            except Exception as e:
                last_error = e

            try:
                self.executor.spin_once(timeout_sec=0.05)
            except Exception as e:
                last_error = e

        if last_error is not None:
            self.node.get_logger().warn(
                f"Service discovery failed: {last_error}. Using root namespace.")
        else:
            self.node.get_logger().warn(
                "No '/get_board_info' service found before timeout. Using root namespace.")
        return ''

    def _load_joint_id_name_map(self, robot_ns: str):
        # rclpy does not expose arbitrary ROS parameter tree lookup like rospy.
        # Keep this optional and empty unless a later ROS 2 config source is added.
        del robot_ns
        return {}

    def _call_service_sync(self, client, req, timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warn(f"Service not available: {client.srv_name}")
            return None

        future = client.call_async(req)
        try:
            self.executor.spin_until_future_complete(future, timeout_sec=timeout_sec)
        except Exception as e:
            self.node.get_logger().error(f"Service call exception: {e}")
            return None

        if not future.done():
            self.node.get_logger().warn(f"Service call timeout: {client.srv_name}")
            return None

        try:
            return future.result()
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return None

    def servoTorqueControl(self, enable):
        servo_index = self._selectedServoIndex()
        if servo_index is None:
            self.node.get_logger().error('No servo exists')
            return
        msg = ServoTorqueCmd()
        msg.index = [servo_index]
        msg.torque_enable = [int(enable)]
        self.servo_torque_pub_.publish(msg)

    def servoOn(self):
        self.servoTorqueControl(1)

    def servoOff(self):
        self.servoTorqueControl(0)

    def allServoTorqueControl(self, enable):
        servo_num = self._widget.servoTableWidget.rowCount()
        if servo_num == 0:
            self.node.get_logger().error('No servo exists')
            return
        msg = ServoTorqueCmd()
        msg.index = list(range(servo_num))
        msg.torque_enable = [int(enable)] * servo_num
        self.servo_torque_pub_.publish(msg)

    def allServoOnButtonCallback(self):
        self.allServoTorqueControl(1)

    def allServoOffButtonCallback(self):
        self.allServoTorqueControl(0)

    def sendTargetPosition(self, position=None):
        servo_index = self._selectedServoIndex()
        if servo_index is None:
            self.node.get_logger().error('No servo exists')
            return
        if position is None:
            position = self.targetPositionSpinBox.value()

        msg = ServoControlCmd()
        msg.index = [servo_index]
        msg.angles = [int(position)]
        self.servo_control_pub_.publish(msg)

    def jointCalib(self):
        servo_index = self._selectedServoIndex()
        if servo_index is None:
            self.node.get_logger().error('No servo exists')
            return

        board_id = int(self._widget.servoTableWidget.item(
            servo_index, self._headers.index('board')).text())
        servo_id = int(self._widget.servoTableWidget.item(
            servo_index, self._headers.index('index')).text())

        if board_id == 0:
            req = SetDirectServoConfig.Request()
            req.data = [servo_id]
            req.command = SetDirectServoConfig.Request.SET_SERVO_HOMING_OFFSET
            client = self.set_direct_servo_config_client_
        else:
            req = SetBoardConfig.Request()
            req.data = [board_id, servo_id]
            req.command = SetBoardConfig.Request.SET_SERVO_HOMING_OFFSET
            client = self.set_board_config_client_

        req.data.append(self._homingOffsetValue())

        servo_trq_msg = ServoTorqueCmd()
        servo_trq_msg.index = [servo_index]
        servo_trq_msg.torque_enable = [0]
        self.servo_torque_pub_.publish(servo_trq_msg)
        time.sleep(0.5)

        self.node.get_logger().info(f'command: {req.command}')
        self.node.get_logger().info(f'data: {req.data}')
        res = self._call_service_sync(client, req)
        if res is not None:
            self.node.get_logger().info(str(bool(res.success)))

    def boardReboot(self):
        servo_index = self._selectedServoIndex()
        if servo_index is None:
            self.node.get_logger().error('No servo exists')
            return

        board_id = int(self._widget.servoTableWidget.item(
            servo_index, self._headers.index('board')).text())
        if board_id == 0:
            self.node.get_logger().error('Spinal cannot be rebooted from rqt')
            return

        req = SetBoardConfig.Request()
        req.data = [board_id]
        req.command = SetBoardConfig.Request.REBOOT

        self.node.get_logger().info(f'command: {req.command}')
        self.node.get_logger().info(f'data: {req.data}')
        res = self._call_service_sync(self.set_board_config_client_, req)
        if res is not None:
            self.node.get_logger().info(str(bool(res.success)))

    def error2string(self, error):
        error_list = []
        if error & 0b10000000:
            error_list.append('Encoder Connection Error')
        if error & 0b1000000:
            error_list.append('Resoluation Ratio Error')
        if error & 0b100000:
            error_list.append('Overload Error')
        if error & 0b10000:
            error_list.append('Electrical Shock Error')
        if error & 0b1000:
            error_list.append('Motor Encoder Error')
        if error & 0b100:
            error_list.append('Overheating Error')
        if error & 0b10:
            error_list.append('Pulley Skip Error')
        if error & 0b1:
            error_list.append('Input Voltage Error')

        return reduce(lambda a, b: a + ', ' + b, error_list) if error_list else 'No Error'

    def servoStateCallback(self, msg):
        for cnt, servo_state in enumerate(msg.servos, start=1):
            if cnt > self._servo_num:
                return
            if servo_state.index >= len(self._table_data):
                continue
            row = self._table_data[servo_state.index]
            row[self._headers.index('angle')] = servo_state.angle
            row[self._headers.index('temperature')] = servo_state.temp
            row[self._headers.index('load')] = servo_state.load
            row[self._headers.index('error')] = self.error2string(int(servo_state.error))
            if servo_state.index == self._selected_servo_index:
                self._syncControlPanelFromSelection(update_target=False)

    def servoTorqueStatesCallback(self, msg):
        for cnt, torque_enable in enumerate(msg.torque_enable, start=1):
            if cnt > self._servo_num:
                return
            index = cnt - 1
            if index >= len(self._table_data):
                continue
            self._table_data[index][self._headers.index('torque')] = (
                'on' if bool(torque_enable) else 'off')

    def updateTable(self):
        if not self._table_data:
            return

        self._widget.servoTableWidget.setRowCount(len(self._table_data))
        self._widget.servoTableWidget.setColumnCount(len(self._table_data[0]))

        for i, row in enumerate(self._table_data):
            for j, value in enumerate(row):
                item = QTableWidgetItem(str(value))
                self._widget.servoTableWidget.setItem(i, j, item)
                if j == 0 and item.text() == 'on':
                    item.setBackground(QColor('cyan'))
                elif j == 0 and item.text() == 'off':
                    item.setBackground(QColor('gray'))

        for i, header in enumerate(self._headers):
            self._widget.servoTableWidget.setHorizontalHeaderItem(
                i, QTableWidgetItem(str(header)))

        for i in range(len(self._table_data)):
            self._widget.servoTableWidget.setVerticalHeaderItem(
                i, QTableWidgetItem('servo' + str(i)))

        if self._selected_servo_index is None and self._table_data:
            self._widget.servoTableWidget.setCurrentCell(0, 0)
        self._syncControlPanelFromSelection(update_target=False)

        self._widget.servoTableWidget.resizeColumnsToContents()
        self._widget.servoTableWidget.show()

    def updateButtonCallback(self):
        req = GetBoardInfo.Request()
        res = self._call_service_sync(self.get_board_info_client_, req)
        if res is None:
            return

        servo_index = 0
        self._servo_num = 0
        self._table_data = []

        for board in res.boards:
            for i, servo in enumerate(board.servos):
                row_data = [
                    None,
                    self.joint_id_name_map.get(servo_index),
                    board.slave_id,
                    i,
                    servo.id,
                    None,
                    None,
                    None,
                    None,
                    f'{servo.p_gain}, {servo.i_gain}, {servo.d_gain}',
                    servo.profile_velocity,
                    servo.current_limit,
                    str(bool(servo.send_data_flag)),
                ]
                servo_index += 1
                self._table_data.append(row_data)
                self._servo_num += 1

        self.updateTable()

    def _installPositionControlPanel(self):
        panel = QFrame(self._widget)
        panel.setFrameShape(QFrame.StyledPanel)
        panel.setFrameShadow(QFrame.Raised)

        layout = QHBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(10)

        self.servoDialWidget = ServoDialWidget(panel)
        self.servoDialWidget.targetPositionChanged.connect(self._targetPositionChanged)
        self.servoDialWidget.targetPositionCommitted.connect(self.sendTargetPosition)
        layout.addWidget(self.servoDialWidget, 1)

        controls = QVBoxLayout()
        controls.setSpacing(4)
        self.selectedServoLabel = QLabel('servo: -', panel)
        self.currentPositionLabel = QLabel('Position: -- [bit] / -- [deg]', panel)
        self.torqueStateLabel = QLabel('Torque: --', panel)

        torque_controls = QHBoxLayout()
        torque_controls.setSpacing(4)
        self.selectedServoOnButton = QPushButton('servo on', panel)
        self.selectedServoOffButton = QPushButton('servo off', panel)
        self.selectedServoOnButton.clicked.connect(lambda _checked=False: self.servoOn())
        self.selectedServoOffButton.clicked.connect(lambda _checked=False: self.servoOff())
        torque_controls.addWidget(self.selectedServoOnButton)
        torque_controls.addWidget(self.selectedServoOffButton)

        self.targetPositionSpinBox = QSpinBox(panel)
        self.targetPositionSpinBox.setRange(SERVO_POSITION_MIN, SERVO_POSITION_MAX)
        self.targetPositionSpinBox.setValue(2047)
        self.targetPositionSpinBox.setSuffix(' bit')
        self.targetPositionSpinBox.valueChanged.connect(self._targetSpinBoxChanged)

        self.sendWhileDraggingCheckBox = QCheckBox('send while dragging', panel)
        self.sendWhileDraggingCheckBox.setChecked(True)

        self.sendTargetButton = QPushButton('send target', panel)
        self.sendTargetButton.clicked.connect(lambda _checked=False: self.sendTargetPosition())

        homing_offset_controls = QHBoxLayout()
        homing_offset_controls.setSpacing(4)
        self.homingOffsetSpinBox = QSpinBox(panel)
        self.homingOffsetSpinBox.setRange(SERVO_POSITION_MIN, SERVO_POSITION_MAX)
        self.homingOffsetSpinBox.setValue(DEFAULT_HOMING_OFFSET)
        self.homingOffsetSpinBox.setSuffix(' bit')
        self.homingOffsetSpinBox.valueChanged.connect(self._homingOffsetSpinBoxChanged)
        self.setHomingOffsetButton = QPushButton('set homing offset', panel)
        self.setHomingOffsetButton.clicked.connect(lambda _checked=False: self.jointCalib())
        homing_offset_controls.addWidget(QLabel('Homing Offset', panel))
        homing_offset_controls.addWidget(self.homingOffsetSpinBox, 1)
        homing_offset_controls.addWidget(self.setHomingOffsetButton)

        if hasattr(self._widget, 'homingOffsetLineEdit'):
            self._widget.homingOffsetLineEdit.setText(str(DEFAULT_HOMING_OFFSET))
            self._widget.homingOffsetLineEdit.textChanged.connect(
                self._homingOffsetLineEditChanged)

        controls.addWidget(self.selectedServoLabel)
        controls.addWidget(self.currentPositionLabel)
        controls.addWidget(self.torqueStateLabel)
        controls.addLayout(torque_controls)
        controls.addWidget(self.targetPositionSpinBox)
        controls.addWidget(self.sendWhileDraggingCheckBox)
        controls.addWidget(self.sendTargetButton)
        controls.addLayout(homing_offset_controls)
        controls.addStretch(1)

        layout.addLayout(controls, 2)

        self.positionSplitter = QSplitter(Qt.Vertical, self._widget)
        self.positionSplitter.setChildrenCollapsible(False)
        self.positionSplitter.addWidget(panel)
        self._widget.gridLayout.removeWidget(self._widget.servoTableWidget)
        self.positionSplitter.addWidget(self._widget.servoTableWidget)
        self.positionSplitter.setStretchFactor(0, 1)
        self.positionSplitter.setStretchFactor(1, 3)
        self.positionSplitter.setSizes([260, 560])

        self._widget.gridLayout.addWidget(self.positionSplitter, 4, 1, 7, 4)

    def _homingOffsetValue(self):
        if hasattr(self, 'homingOffsetSpinBox'):
            return int(self.homingOffsetSpinBox.value())
        return int(self._widget.homingOffsetLineEdit.text())

    def _homingOffsetSpinBoxChanged(self, value):
        if self._syncing_homing_offset_control:
            return
        if hasattr(self._widget, 'homingOffsetLineEdit'):
            self._syncing_homing_offset_control = True
            self._widget.homingOffsetLineEdit.setText(str(int(value)))
            self._syncing_homing_offset_control = False

    def _homingOffsetLineEditChanged(self, text):
        if self._syncing_homing_offset_control:
            return
        try:
            value = int(text)
        except ValueError:
            return
        value = max(SERVO_POSITION_MIN, min(SERVO_POSITION_MAX, value))
        self._syncing_homing_offset_control = True
        self.homingOffsetSpinBox.setValue(value)
        self._syncing_homing_offset_control = False

    def _selectedServoIndex(self):
        row = self._widget.servoTableWidget.currentIndex().row()
        if row < 0 or row >= self._servo_num:
            return None
        return row

    def _selectedServoChanged(self, current_row, _current_column, _previous_row, _previous_column):
        self._selected_servo_index = current_row if 0 <= current_row < self._servo_num else None
        self._syncControlPanelFromSelection(update_target=True)

    def _syncControlPanelFromSelection(self, update_target):
        servo_index = self._selectedServoIndex()
        self._selected_servo_index = servo_index
        if servo_index is None or servo_index >= len(self._table_data):
            self.selectedServoLabel.setText('servo: -')
            self.currentPositionLabel.setText('Position: -- [bit] / -- [deg]')
            self.torqueStateLabel.setText('Torque: --')
            self.servoDialWidget.setCurrentPosition(None)
            return

        row = self._table_data[servo_index]
        raw_position = row[self._headers.index('angle')]
        torque_state = row[self._headers.index('torque')]
        servo_id = row[self._headers.index('id')]
        self.selectedServoLabel.setText(f'servo: {servo_index} / id: {servo_id}')
        self.torqueStateLabel.setText(f'Torque: {torque_state}')

        if raw_position is None:
            self.currentPositionLabel.setText('Position: -- [bit] / -- [deg]')
            self.servoDialWidget.setCurrentPosition(None)
            return

        position = int(raw_position)
        degrees = _position_to_degrees(position)
        self.currentPositionLabel.setText(f'Position: {position} [bit] / {degrees:.2f} [deg]')
        self.servoDialWidget.setCurrentPosition(position)

        if update_target:
            target = position % SERVO_POSITION_RANGE
            self._syncing_target_control = True
            self.targetPositionSpinBox.setValue(target)
            self.servoDialWidget.setTargetPosition(target)
            self._syncing_target_control = False

    def _targetSpinBoxChanged(self, position):
        if self._syncing_target_control:
            return
        self.servoDialWidget.setTargetPosition(position)

    def _targetPositionChanged(self, position, dragging):
        self._syncing_target_control = True
        self.targetPositionSpinBox.setValue(position)
        self._syncing_target_control = False

        if not dragging:
            return
        if not self.sendWhileDraggingCheckBox.isChecked():
            return

        now = time.monotonic()
        if now - self._last_target_pub_time < 0.05:
            return
        self._last_target_pub_time = now
        self.sendTargetPosition(position)
