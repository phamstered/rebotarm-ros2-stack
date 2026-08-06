#!/usr/bin/env python3
"""Topic-level STM32 simulator for testing the bridge without hardware."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from microros_stm32h723_bridge.msg import HardwareStatus, JointCommand


JOINT_NAMES = [
    'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6',
    'gripper_finger_left_joint',
]


class MockStm32Node(Node):
    def __init__(self):
        super().__init__('mock_stm32h723')
        self.declare_parameter('active_joint_mask', 0x7F)
        self._active_joint_mask = int(
            self.get_parameter('active_joint_mask').value)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._positions = [0.0] * 7
        self._target = [0.0] * 7
        self._last_sequence = 0
        self._mode = HardwareStatus.MODE_DISABLE
        self._fault_code = 0
        self._state_pub = self.create_publisher(JointState, '/stm32/joint_states', qos)
        self._status_pub = self.create_publisher(HardwareStatus, '/stm32/status', qos)
        self.create_subscription(
            JointCommand, '/stm32/joint_commands', self._command_callback, qos)
        self.create_timer(0.01, self._update)

    def _command_callback(self, message):
        arrays = [message.position, message.velocity, message.kp,
                  message.kd, message.effort]
        if not all(len(values) == 7 for values in arrays):
            return
        if not all(math.isfinite(value) for values in arrays for value in values):
            return
        for index in range(7):
            if self._active_joint_mask & (1 << index):
                continue
            if any(abs(float(values[index])) > 1e-9 for values in arrays):
                self._fault_code = 1  # INACTIVE_JOINT_COMMAND
                return
        self._last_sequence = message.sequence
        self._mode = message.mode
        if message.mode != JointCommand.MODE_DISABLE:
            # Fixed-size rosidl arrays can expose NumPy scalar values. JointState
            # setters require built-in Python floats.
            self._target = [float(value) for value in message.position]

    def _update(self):
        if self._mode != JointCommand.MODE_DISABLE:
            self._positions = [
                float(current + max(-0.01, min(0.01, target - current)))
                for current, target in zip(self._positions, self._target)
            ]
        now = self.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = now
        state.name = JOINT_NAMES
        state.position = self._positions
        state.velocity = [0.0] * 7
        state.effort = [0.0] * 7
        self._state_pub.publish(state)

        status = HardwareStatus()
        status.header.stamp = now
        status.last_command_sequence = self._last_sequence
        status.mode = self._mode
        status.active_joint_mask = self._active_joint_mask
        status.fault_code = self._fault_code
        self._status_pub.publish(status)


def main():
    rclpy.init()
    node = MockStm32Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
