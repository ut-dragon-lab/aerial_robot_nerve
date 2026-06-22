#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import time
from functools import reduce

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from rqt_gui_py.plugin import Plugin

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import QAction, QTableWidgetItem, QWidget

from spinal_msgs.msg import ServoStates, ServoTorqueCmd, ServoTorqueStates
from spinal_msgs.srv import GetBoardInfo, SetBoardConfig, SetDirectServoConfig


DISCOVERY_TIMEOUT_SEC = 2.0
SERVICE_WAIT_TIMEOUT_SEC = 2.0


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
        servo_index = self._widget.servoTableWidget.currentIndex().row()
        if servo_index == -1:
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

    def jointCalib(self):
        servo_index = self._widget.servoTableWidget.currentIndex().row()
        if servo_index == -1:
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

        try:
            req.data.append(int(self._widget.homingOffsetLineEdit.text()))
        except ValueError as e:
            print(e)
            return

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
        servo_index = self._widget.servoTableWidget.currentIndex().row()
        if servo_index == -1:
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
