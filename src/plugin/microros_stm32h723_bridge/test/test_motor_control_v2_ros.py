"""Real ROS serialization/service smoke with the deterministic MCU peer."""

import os
from pathlib import Path
import sys
import time

import pytest

rclpy = pytest.importorskip('rclpy')
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from microros_stm32h723_bridge.msg import JointCommand, JointCommandV2, MotorControlStatus
from microros_stm32h723_bridge.srv import MotorControl

sys.path.insert(0, str(Path(__file__).parents[1] / 'scripts'))
from mock_motor_control_v2 import MockMotorControlV2


def test_v2_service_stream_competing_v1_and_global_disable():
    assert os.environ.get('ROS_DOMAIN_ID') == '219'
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    rclpy.init()
    mock = MockMotorControlV2()
    client = rclpy.create_node('motor_control_v2_test')
    executor = SingleThreadedExecutor()
    executor.add_node(mock)
    executor.add_node(client)
    statuses = []
    client.create_subscription(MotorControlStatus, '/posvel_test/motor_control_status',
                               statuses.append, qos_profile_sensor_data)
    v1 = client.create_publisher(JointCommand, '/posvel_test/joint_commands', qos_profile_sensor_data)
    v2 = client.create_publisher(JointCommandV2, '/posvel_test/joint_commands_v2', qos_profile_sensor_data)
    service = client.create_client(MotorControl, '/posvel_test/motor_control')

    def until(predicate, timeout=3.0):
        end = time.monotonic() + timeout
        while not predicate() and time.monotonic() < end:
            executor.spin_once(timeout_sec=0.002)
        assert predicate(), 'isolated V2 peer did not satisfy expected state'

    def call(action, op, expected_epoch=None):
        status = statuses[-1]
        request = MotorControl.Request()
        request.action, request.operation_id = action, op
        request.boot_id, request.source_id = status.boot_id, 42
        request.expected_control_epoch = status.control_epoch if expected_epoch is None else expected_epoch
        request.active_joint_mask, request.control_mode = 1, 2
        request.lease_id = status.lease_id
        request.disable_sequence = status.last_command_sequence
        future = service.call_async(request)
        until(future.done)
        return future.result()

    try:
        until(lambda: statuses and service.service_is_ready() and
              v1.get_subscription_count() > 0 and v2.get_subscription_count() > 0, timeout=10.0)
        stop = JointCommand()
        stop.mode, stop.sequence = 3, 1
        v1.publish(stop)
        until(lambda: statuses[-1].last_command_sequence == 1 and statuses[-1].disabled_verified)
        reply = call(MotorControl.Request.CONFIGURE, 1)
        assert reply.accepted
        until(lambda: statuses[-1].operation_id == 1 and statuses[-1].operation_state == 2)
        assert statuses[-1].control_modes[0] == 2 and statuses[-1].mode == 3
        stale = call(MotorControl.Request.ACQUIRE, 2, expected_epoch=0)
        assert not stale.accepted and stale.error_code == MotorControlStatus.ERROR_STALE_IDENTITY
        grant = call(MotorControl.Request.ACQUIRE, 3)
        assert grant.accepted and grant.lease_id
        frame = JointCommandV2()
        frame.boot_id, frame.control_epoch = statuses[-1].boot_id, grant.control_epoch
        frame.source_id, frame.lease_id = 42, grant.lease_id
        frame.sequence, frame.mode, frame.control_mode, frame.active_joint_mask = 2, 1, 2, 1
        frame.position = [0.5] + [0.0] * 6
        frame.velocity_limit = [0.2] + [0.0] * 6
        v2.publish(frame)
        until(lambda: statuses[-1].last_command_sequence == 2)
        assert statuses[-1].enabled_joint_mask == 1 and statuses[-1].mode == 1
        competing = JointCommand()
        competing.sequence, competing.mode = 3, 2
        v1.publish(competing)
        until(lambda: statuses[-1].rejected_sequence == 3)
        assert statuses[-1].reject_code == MotorControlStatus.ERROR_BUSY
        assert statuses[-1].last_command_sequence == 2
        stop.sequence = 4
        v1.publish(stop)
        until(lambda: statuses[-1].last_command_sequence == 4 and statuses[-1].disabled_verified)
        assert statuses[-1].mode == 3 and statuses[-1].enabled_joint_mask == 0
        assert statuses[-1].lease_id == 0
        frame.sequence = 5
        v2.publish(frame)
        until(lambda: statuses[-1].rejected_sequence == 5)
        assert statuses[-1].reject_code == MotorControlStatus.ERROR_LEASE
        assert statuses[-1].mode == 3
    finally:
        executor.shutdown()
        client.destroy_node()
        mock.destroy_node()
        rclpy.shutdown()
