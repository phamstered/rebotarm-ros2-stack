#!/usr/bin/env python3
"""Safely exercise only the base motor (joint1) over the STM32 topic protocol."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from microros_stm32h723_bridge.msg import HardwareStatus, JointCommand


class SingleMotorSmokeTest(Node):
    def __init__(self, target, kp, kd, duration):
        super().__init__('single_motor_smoke_test')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._publisher = self.create_publisher(
            JointCommand, '/stm32/joint_commands', qos)
        self.create_subscription(
            HardwareStatus, '/stm32/status', self._status_callback, qos)
        self._target = target
        self._kp = kp
        self._kd = kd
        self._deadline = time.monotonic() + duration
        self._sequence = 0
        self._active_mask_confirmed = False
        self._disable_sent_at = None
        self.done = False
        self.create_timer(0.02, self._tick)

    def _status_callback(self, message):
        if message.active_joint_mask != 0x01:
            self.get_logger().error(
                f'Expected active_joint_mask=0x01, got {message.active_joint_mask:#x}')
            self.done = True
            return
        self._active_mask_confirmed = True
        if self._disable_sent_at is not None and message.mode == HardwareStatus.MODE_DISABLE:
            self.get_logger().info('STM32 confirmed DISABLE')
            self.done = True

    def _tick(self):
        if self.done or not self._active_mask_confirmed:
            return
        command = JointCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        self._sequence += 1
        command.sequence = self._sequence
        command.position = [0.0] * 7
        command.velocity = [0.0] * 7
        command.kp = [0.0] * 7
        command.kd = [0.0] * 7
        command.effort = [0.0] * 7
        if time.monotonic() < self._deadline:
            command.mode = JointCommand.MODE_TRACK
            command.position[0] = self._target
            command.kp[0] = self._kp
            command.kd[0] = self._kd
        else:
            command.mode = JointCommand.MODE_DISABLE
            self._disable_sent_at = time.monotonic()
        self._publisher.publish(command)
        if self._disable_sent_at and time.monotonic() - self._disable_sent_at > 2.0:
            self.get_logger().error('Timed out waiting for DISABLE confirmation')
            self.done = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', type=float, default=0.05)
    parser.add_argument('--kp', type=float, default=2.0)
    parser.add_argument('--kd', type=float, default=0.2)
    parser.add_argument('--duration', type=float, default=2.0)
    args, ros_args = parser.parse_known_args()
    if not math.isfinite(args.target) or abs(args.target) > 0.10:
        parser.error('--target must be finite and within +/-0.10 rad')
    if not (0.0 <= args.kp <= 5.0 and 0.0 <= args.kd <= 1.0):
        parser.error('smoke-test gains exceed the safe bounds (kp<=5, kd<=1)')

    rclpy.init(args=ros_args)
    node = SingleMotorSmokeTest(args.target, args.kp, args.kd, args.duration)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
