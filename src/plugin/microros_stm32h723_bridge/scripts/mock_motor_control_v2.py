#!/usr/bin/env python3
"""Isolated V2 MCU peer. Run in a non-production ROS_DOMAIN_ID/namespace."""

import secrets
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from microros_stm32h723_bridge.msg import HardwareStatus, JointCommand, JointCommandV2, MotorControlStatus
from microros_stm32h723_bridge.srv import MotorControl
from motor_control_sim import MotorControlSim


class MockMotorControlV2(Node):
    def __init__(self):
        super().__init__('mock_motor_control_v2')
        self.declare_parameter('command_topic', '/posvel_test/joint_commands')
        topic = str(self.get_parameter('command_topic').value)
        parent = topic.rpartition('/')[0]
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.model = MotorControlSim(boot_id=secrets.randbits(63) or 1)
        self.status = self.create_publisher(MotorControlStatus, parent + '/motor_control_status', qos)
        self.legacy = self.create_publisher(HardwareStatus, parent + '/status', qos)
        self.joints = self.create_publisher(JointState, parent + '/joint_states', qos)
        self.create_subscription(JointCommand, topic, self.v1, qos)
        self.create_subscription(JointCommandV2, topic + '_v2', self.v2, qos)
        self.create_service(MotorControl, parent + '/motor_control', self.service)
        self.create_timer(0.01, self.update)

    def v1(self, msg):
        self.model.command_v1({key: getattr(msg, key) for key in
                              ('sequence', 'mode', 'position', 'velocity', 'kp', 'kd', 'effort')}, time.monotonic())

    def v2(self, msg):
        self.model.command_v2({key: getattr(msg, key) for key in
                              ('boot_id', 'control_epoch', 'source_id', 'lease_id', 'sequence',
                               'mode', 'control_mode', 'active_joint_mask', 'position', 'velocity_limit')}, time.monotonic())

    def service(self, request, response):
        result = self.model.service({key: getattr(request, key) for key in
                                     ('action', 'operation_id', 'boot_id', 'expected_control_epoch',
                                      'source_id', 'active_joint_mask', 'control_mode', 'lease_id',
                                      'disable_sequence')}, time.monotonic())
        for key, value in result.items():
            setattr(response, key, value)
        return response

    def update(self):
        self.model.tick(time.monotonic())
        stamp = self.get_clock().now().to_msg()
        status = MotorControlStatus()
        status.header.stamp = stamp
        for key, value in self.model.status().items():
            setattr(status, key, value)
        self.status.publish(status)
        legacy = HardwareStatus()
        legacy.header.stamp = stamp
        legacy.last_command_sequence = self.model.last_command_sequence
        legacy.mode = self.model.mode
        legacy.active_joint_mask = 0x7f
        # Rejections live in MotorControlStatus; they are not latched motor faults.
        legacy.fault_code = self.model.fault_code
        self.legacy.publish(legacy)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_finger_left_joint']
        joints.position = [float(value) for value in self.model.position]
        joints.velocity = [0.0] * 7
        joints.effort = [0.0] * 7
        self.joints.publish(joints)


def main():
    rclpy.init()
    node = MockMotorControlV2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
