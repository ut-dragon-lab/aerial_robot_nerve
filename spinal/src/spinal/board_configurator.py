#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import time

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.executors import SingleThreadedExecutor

from rqt_gui_py.plugin import Plugin

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QStandardItem, QStandardItemModel
from python_qt_binding.QtWidgets import QAbstractItemView, QWidget

from spinal_msgs.msg import ServoTorqueCmd
from spinal_msgs.srv import GetBoardInfo, SetBoardConfig, SetDirectServoConfig


DISCOVERY_TIMEOUT_SEC = 2.0
SERVICE_WAIT_TIMEOUT_SEC = 2.0


def _ns_join(ns: str, name: str) -> str:
    if not ns:
        return '/' + name if not name.startswith('/') else name
    return f'{ns.rstrip("/")}/{name.lstrip("/")}'


def _str_to_bool(value: str) -> int:
    v = value.strip().lower()
    if v in ('y', 'yes', 't', 'true', 'on', '1'):
        return 1
    if v in ('n', 'no', 'f', 'false', 'off', '0'):
        return 0
    raise ValueError(f'invalid truth value {value!r}')


class BoardConfigurator(Plugin):
    def __init__(self, context):
        super(BoardConfigurator, self).__init__(context)
        self.setObjectName('BoardConfigurator')

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

        self.node = rclpy.create_node(f'board_configurator_rqt_{id(self)}')
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
        self.direct_servo_config_client_ = self.node.create_client(
            SetDirectServoConfig, _ns_join(robot_ns, 'direct_servo_config'))
        self.servo_torque_pub_ = self.node.create_publisher(
            ServoTorqueCmd, _ns_join(robot_ns, 'servo/torque_enable'), 1)

        self._widget = QWidget()
        ui_file = os.path.join(get_package_share_directory('spinal'),
                               'resource', 'board_configurator.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('BoardConfigurator')

        self._widget.boardInfoUpdateButton.clicked.connect(self.updateButtonCallback)
        self._widget.boardInfoTreeView.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._widget.boardInfoTreeView.clicked.connect(self.treeClickedCallback)
        self._widget.configureButton.clicked.connect(self.configureButtonCallback)

        self.model = QStandardItemModel()
        self._widget.boardInfoTreeView.setModel(self.model)
        self._widget.boardInfoTreeView.setUniformRowHeights(True)
        self.model.setHorizontalHeaderLabels(['board ID', 'param'])
        self._widget.boardInfoTreeView.show()
        self._widget.setLayout(self._widget.gridLayout)
        self._widget.boardInfoTreeView.setEditTriggers(QAbstractItemView.NoEditTriggers)

        self._board_id = None
        self._servo_index = None
        self._raw_servo_id = None
        self._command = None
        self._current_servo_serial_index = None

        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + f' ({context.serial_number()})')
        context.add_widget(self._widget)

        self.joint_id_name_map = self._load_joint_id_name_map(robot_ns)
        self.updateButtonCallback()

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

    def updateButtonCallback(self):
        req = GetBoardInfo.Request()
        res = self._call_service_sync(self.get_board_info_client_, req)
        if res is None:
            return

        self.model = QStandardItemModel()
        self._widget.boardInfoTreeView.setModel(self.model)
        self._widget.boardInfoTreeView.setUniformRowHeights(True)
        self.model.setHorizontalHeaderLabels(['param', 'value'])

        servo_serial_index = 0
        for i, board_info in enumerate(res.boards):
            board = QStandardItem(str(board_info.slave_id))
            board.setEditable(False)
            board.appendRow([QStandardItem('board_id'), QStandardItem(str(board_info.slave_id))])
            board.appendRow([QStandardItem('imu_send_data_flag'),
                             QStandardItem(str(bool(board_info.imu_send_data_flag)))])
            board.appendRow([QStandardItem('dynamixel_ttl_rs485_mixed'),
                             QStandardItem(str(bool(board_info.dynamixel_ttl_rs485_mixed)))])
            board.appendRow([QStandardItem('servo_pulley_skip_thresh'),
                             QStandardItem(str(board_info.servo_pulley_skip_thresh))])

            servos = QStandardItem('servo (' + str(len(board_info.servos)) + ')')
            for j, servo_info in enumerate(board_info.servos):
                servo = QStandardItem(str(j))
                servo.appendRow([QStandardItem('servo_id'), QStandardItem(str(servo_info.id))])
                servo.appendRow([QStandardItem('servo_serial_index'),
                                 QStandardItem(str(servo_serial_index))])
                servo.appendRow([QStandardItem('joint_name'),
                                 QStandardItem(str(self.joint_id_name_map.get(servo_serial_index)))])
                servo_serial_index += 1
                servo.appendRow([QStandardItem('pid_gain'),
                                 QStandardItem(f'{servo_info.p_gain}, {servo_info.i_gain}, {servo_info.d_gain}')])
                servo.appendRow([QStandardItem('profile_velocity'),
                                 QStandardItem(str(servo_info.profile_velocity))])
                servo.appendRow([QStandardItem('send_data_flag'),
                                 QStandardItem(str(bool(servo_info.send_data_flag)))])
                servo.appendRow([QStandardItem('current_limit'),
                                 QStandardItem(str(servo_info.current_limit))])
                servo.appendRow([QStandardItem('external_encoder_flag'),
                                 QStandardItem(str(bool(servo_info.external_encoder_flag)))])
                servo.appendRow([QStandardItem('resolution[joint:servo]'),
                                 QStandardItem(f'{servo_info.joint_resolution} : {servo_info.servo_resolution}')])
                servos.appendRow(servo)

            board.appendRow(servos)
            self.model.appendRow(board)
            self._widget.boardInfoTreeView.setFirstColumnSpanned(
                i, self._widget.boardInfoTreeView.rootIndex(), True)

        self._widget.boardInfoTreeView.setColumnWidth(0, 250)
        self._widget.boardInfoTreeView.setColumnWidth(1, 70)
        self._widget.boardInfoTreeView.expandAll()
        self._widget.boardInfoTreeView.show()

    def treeClickedCallback(self, *_args):
        index = self._widget.boardInfoTreeView.currentIndex()
        row = index.row()
        param = index.sibling(row, 0).data()
        value = index.sibling(row, 1).data()

        board_param_list = [
            'board_id', 'imu_send_data_flag', 'dynamixel_ttl_rs485_mixed',
            'servo_pulley_skip_thresh'
        ]
        servo_param_list = [
            'pid_gain', 'profile_velocity', 'current_limit', 'send_data_flag',
            'external_encoder_flag', 'resolution[joint:servo]'
        ]

        if value and param in servo_param_list:
            raw_servo_id = index.sibling(0, 1).data()
            servo_index = index.parent().data()
            board_id = index.parent().parent().parent().data()
            self._widget.paramLabel.setText(
                'board_id: ' + board_id + ' servo_index: ' + servo_index + ' ' + param)
            self._board_id = board_id
            self._servo_index = servo_index
            self._raw_servo_id = raw_servo_id
            self._command = param
            self._current_servo_serial_index = int(index.parent().child(1, 1).data())
        elif value and param in board_param_list:
            board_id = index.parent().data()
            self._widget.paramLabel.setText('board_id: ' + board_id + ' ' + param)
            self._board_id = board_id
            self._command = param
            self._servo_index = None
            self._raw_servo_id = None
        else:
            self._board_id = None
            self._command = None
            self._servo_index = None
            self._raw_servo_id = None
            self._widget.paramLabel.setText('')

    def _make_config_request(self):
        if self._board_id is None:
            self.node.get_logger().error('board id is not registered')
            return None, None, False, None

        spinal_flag = self._board_id == '0'
        if spinal_flag:
            req = SetDirectServoConfig.Request()
            req_cls = SetDirectServoConfig.Request
            client = self.direct_servo_config_client_
        else:
            req = SetBoardConfig.Request()
            req.data = [int(self._board_id)]
            req_cls = SetBoardConfig.Request
            client = self.set_board_config_client_

        return req, req_cls, spinal_flag, client

    def configureButtonCallback(self):
        request_tuple = self._make_config_request()
        if request_tuple[0] is None:
            return
        req, req_cls, spinal_flag, client = request_tuple

        try:
            if self._command == 'board_id':
                if spinal_flag:
                    return
                req.data.append(int(self._widget.lineEdit.text()))
                req.command = req_cls.SET_SLAVE_ID
            elif self._command == 'imu_send_data_flag':
                if spinal_flag:
                    return
                req.data.append(_str_to_bool(self._widget.lineEdit.text()))
                req.command = req_cls.SET_IMU_SEND_FLAG
            elif self._command == 'pid_gain':
                req.data.append(int(self._servo_index))
                pid_gains = [int(x) for x in self._widget.lineEdit.text().split(',')]
                if len(pid_gains) != 3:
                    raise ValueError('Input 3 gains(int)')
                req.data.extend(pid_gains)
                req.command = req_cls.SET_SERVO_PID_GAIN
            elif self._command == 'profile_velocity':
                req.data.extend([int(self._servo_index), int(self._widget.lineEdit.text())])
                req.command = req_cls.SET_SERVO_PROFILE_VEL
            elif self._command == 'send_data_flag':
                req.data.extend([int(self._servo_index), _str_to_bool(self._widget.lineEdit.text())])
                req.command = req_cls.SET_SERVO_SEND_DATA_FLAG
            elif self._command == 'current_limit':
                req.data.extend([int(self._servo_index), int(self._widget.lineEdit.text())])
                req.command = req_cls.SET_SERVO_CURRENT_LIMIT
                servo_trq_msg = ServoTorqueCmd()
                servo_trq_msg.index = [self._current_servo_serial_index]
                servo_trq_msg.torque_enable = [0]
                self.servo_torque_pub_.publish(servo_trq_msg)
                time.sleep(0.5)
            elif self._command == 'dynamixel_ttl_rs485_mixed':
                req.data.append(_str_to_bool(self._widget.lineEdit.text()))
                req.command = req_cls.SET_DYNAMIXEL_TTL_RS485_MIXED
            elif self._command == 'servo_pulley_skip_thresh':
                if spinal_flag:
                    return
                req.data.append(int(self._widget.lineEdit.text()))
                req.command = req_cls.SET_SERVO_PULLEY_SKIP_THRESH
            elif self._command == 'external_encoder_flag':
                req.data.extend([int(self._servo_index), _str_to_bool(self._widget.lineEdit.text())])
                req.command = req_cls.SET_SERVO_EXTERNAL_ENCODER_FLAG
            elif self._command == 'resolution[joint:servo]':
                req.data.append(int(self._servo_index))
                resolutions = [int(x) for x in self._widget.lineEdit.text().split(':')]
                if len(resolutions) != 2:
                    raise ValueError('Input 2 resolution(int)')
                req.data.extend(resolutions)
                req.command = req_cls.SET_SERVO_RESOLUTION_RATIO
            else:
                return
        except ValueError as e:
            print(e)
            return

        self.node.get_logger().info(f'command: {req.command}')
        self.node.get_logger().info(f'data: {req.data}')

        res = self._call_service_sync(client, req)
        if res is not None:
            self.node.get_logger().info(str(bool(res.success)))
            time.sleep(1)
            self.updateButtonCallback()
