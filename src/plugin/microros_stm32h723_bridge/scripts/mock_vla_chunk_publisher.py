#!/usr/bin/env python3
"""Publish safe sine-wave VlaJointChunk messages for VLA bring-up."""

import math

import rclpy
from rclpy.node import Node

from microros_stm32h723_bridge.msg import VlaJointChunk


class MockVlaChunkPublisher(Node):
    def __init__(self):
        super().__init__('mock_vla_chunk_publisher')
        self.declare_parameter('topic', '/vla/joint_chunk')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('amplitude_rad', 0.15)
        self.declare_parameter('frequency_hz', 0.1)
        self.declare_parameter('timeout_sec', 0.0)
        self.declare_parameter('num_joints', 6)

        topic = str(self.get_parameter('topic').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._amplitude = float(self.get_parameter('amplitude_rad').value)
        self._frequency = float(self.get_parameter('frequency_hz').value)
        self._timeout_sec = float(self.get_parameter('timeout_sec').value)
        self._num_joints = int(self.get_parameter('num_joints').value)
        if self._num_joints < 1:
            raise ValueError('num_joints must be >= 1')
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be > 0')

        self._publisher = self.create_publisher(VlaJointChunk, topic, 10)
        self._start = self.get_clock().now()
        period = 1.0 / rate
        self.create_timer(period, self._tick)
        self.get_logger().info(
            'Publishing mock VLA chunks on %s at %.1f Hz '
            '(amp=%.3f rad, freq=%.3f Hz, joints=%d)' % (
                topic, rate, self._amplitude, self._frequency, self._num_joints))

    def _tick(self):
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        omega = 2.0 * math.pi * self._frequency
        message = VlaJointChunk()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'vla_mock'
        # Small phase offsets per joint keep motion visibly decoupled but safe.
        message.positions = [
            float(self._amplitude * math.sin(omega * elapsed + 0.35 * index))
            for index in range(self._num_joints)
        ]
        message.timeout_sec = float(self._timeout_sec)
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = MockVlaChunkPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
