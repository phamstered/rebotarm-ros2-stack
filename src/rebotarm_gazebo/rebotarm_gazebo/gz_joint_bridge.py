#!/usr/bin/env python3
"""ROS2 → Gazebo Transport bridge for joint position commands via service API."""
import os
import sys
import subprocess

import rclpy
from rclpy.node import Node as RosNode
from sensor_msgs.msg import JointState


class GzServiceBridge(RosNode):
    def __init__(self):
        super().__init__('gz_joint_bridge')

        self.declare_parameter('ros_topic', '/joint_commands')
        self.declare_parameter('model_name', 'rebotarm')
        self.declare_parameter('joint_names',
                               ['joint1', 'joint2', 'joint3',
                                'joint4', 'joint5', 'joint6'])

        ros_topic = self.get_parameter('ros_topic').value
        self._model_name = self.get_parameter('model_name').value
        self._joint_names = list(self.get_parameter('joint_names').value)

        self.create_subscription(JointState, ros_topic, self._ros_cb, 10)
        self.get_logger().info(f'ROS subscriber: {ros_topic}')

        self._count = 0
        self._last_positions = {}

    def _ros_cb(self, msg: JointState):
        for name in self._joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                if idx < len(msg.position):
                    self._last_positions[name] = msg.position[idx]

        self._count += 1
        if self._count % 50 == 0:
            self._send_positions()

    def _send_positions(self):
        if not self._last_positions:
            return

        for name, pos in self._last_positions.items():
            topic = f'/model/{self._model_name}/joint/{name}/0/cmd_pos'
            try:
                subprocess.run(
                    ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double',
                     '-p', f'data: {pos}'],
                    capture_output=True, timeout=1)
            except Exception as e:
                self.get_logger().warn(f'Failed to send {name}: {e}')

        if self._count % 200 == 0:
            self.get_logger().info(
                f'Sent {len(self._last_positions)} joint positions to Gazebo')


def main(args=None):
    rclpy.init(args=args)
    node = GzServiceBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
