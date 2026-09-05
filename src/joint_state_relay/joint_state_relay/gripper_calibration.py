#!/usr/bin/env python3
"""Read-only endpoint calibration for the leader and STM32 grippers."""

import argparse
from collections import deque
import math
import statistics
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from microros_stm32h723_bridge.msg import HardwareStatus


FOLLOWER_GRIPPER_MASK = 0x40
MODE_DISABLE = 3


def linear_map(input_closed, input_open, output_closed, output_open):
    """Return scale and offset for output = scale * input + offset."""
    values = (input_closed, input_open, output_closed, output_open)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('calibration endpoints must be finite')
    input_span = input_open - input_closed
    if abs(input_span) < 1e-6:
        raise ValueError('closed and open input endpoints are too close')
    scale = (output_open - output_closed) / input_span
    offset = output_closed - scale * input_closed
    return scale, offset


class GripperCalibrationNode(Node):
    def __init__(self, args):
        super().__init__('gripper_calibration')
        self._leader_joint = args.leader_joint
        self._follower_joint = args.follower_joint
        self._lock = threading.Lock()
        self._samples = {
            'leader': deque(maxlen=2000),
            'follower': deque(maxlen=2000),
        }
        self._status = None
        best_effort_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, args.leader_topic, self._leader_callback, 10)
        self.create_subscription(
            JointState, args.follower_topic, self._follower_callback,
            best_effort_qos)
        self.create_subscription(
            HardwareStatus, args.status_topic, self._status_callback,
            best_effort_qos)

    def _record_joint(self, source, message, joint_name):
        try:
            index = message.name.index(joint_name)
        except ValueError:
            return
        if index >= len(message.position):
            return
        value = float(message.position[index])
        if not math.isfinite(value):
            return
        with self._lock:
            self._samples[source].append((time.monotonic(), value))

    def _leader_callback(self, message):
        self._record_joint('leader', message, self._leader_joint)

    def _follower_callback(self, message):
        self._record_joint('follower', message, self._follower_joint)

    def _status_callback(self, message):
        with self._lock:
            self._status = (
                time.monotonic(), int(message.mode),
                int(message.active_joint_mask), int(message.fault_code))

    def ready(self):
        with self._lock:
            return (
                bool(self._samples['leader']) and
                bool(self._samples['follower']) and
                self._status is not None)

    def require_safe_follower(self, max_age=1.0):
        with self._lock:
            status = self._status
        if status is None or time.monotonic() - status[0] > max_age:
            raise RuntimeError('no fresh /stm32/status message')
        _, mode, active_mask, fault_code = status
        if not active_mask & FOLLOWER_GRIPPER_MASK:
            raise RuntimeError(
                f'gripper bit 0x40 is absent from active_joint_mask=0x{active_mask:02X}')
        if fault_code != 0:
            raise RuntimeError(f'STM32 fault_code={fault_code}, expected 0')
        if mode != MODE_DISABLE:
            raise RuntimeError(
                f'STM32 mode={mode}; endpoint capture requires MODE_DISABLE=3')

    def sample(self, source, duration):
        started_at = time.monotonic()
        time.sleep(duration)
        with self._lock:
            values = [
                value for timestamp, value in self._samples[source]
                if timestamp >= started_at]
        if len(values) < 5:
            raise RuntimeError(
                f'only {len(values)} fresh {source} samples received')
        return statistics.median(values), max(values) - min(values), len(values)


def _capture(node, source, label, duration):
    input(f'\n将{label}移动到位并保持静止，然后按 Enter 采样：')
    node.require_safe_follower()
    value, spread, count = node.sample(source, duration)
    print(f'  {label}: {value:+.9f}，波动 {spread:.9f}，{count} 个样本')
    return value


def _wait_until_ready(node, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if node.ready():
            return
        time.sleep(0.1)
    raise RuntimeError('leader、follower 或 STM32 status 话题未就绪')


def parse_args(args=None):
    parser = argparse.ArgumentParser(
        description='只读采集主从夹爪端点并计算遥操作映射')
    parser.add_argument('--leader-topic', default='/leader/joint_states')
    parser.add_argument('--follower-topic', default='/stm32/joint_states')
    parser.add_argument('--status-topic', default='/stm32/status')
    parser.add_argument('--leader-joint', default='joint7_left')
    parser.add_argument(
        '--follower-joint', default='gripper_finger_left_joint')
    parser.add_argument('--sample-duration', type=float, default=1.0)
    parser.add_argument('--startup-timeout', type=float, default=10.0)
    parser.add_argument(
        '--follower-travel-m', type=float, default=None,
        help='实测指尖单侧行程；提供后同时计算 STM32 raw 到 URDF 米制转换')
    return parser.parse_args(args)


def main(args=None):
    parsed = parse_args(args)
    if parsed.sample_duration <= 0 or parsed.startup_timeout <= 0:
        raise SystemExit('采样时间和启动超时必须大于 0')
    if parsed.follower_travel_m is not None and parsed.follower_travel_m <= 0:
        raise SystemExit('--follower-travel-m 必须大于 0')

    rclpy.init(args=None)
    node = GripperCalibrationNode(parsed)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        print('夹爪端点标定（只读，不发布任何控制指令）')
        print('要求：STM32 为 MODE_DISABLE、fault_code=0、mask 包含 0x40。')
        _wait_until_ready(node, parsed.startup_timeout)
        node.require_safe_follower()

        leader_closed = _capture(
            node, 'leader', '主臂夹爪完全闭合位置', parsed.sample_duration)
        leader_open = _capture(
            node, 'leader', '主臂夹爪完全张开位置', parsed.sample_duration)
        follower_closed = _capture(
            node, 'follower', '从臂夹爪完全闭合位置', parsed.sample_duration)
        follower_open = _capture(
            node, 'follower', '从臂夹爪完全张开位置', parsed.sample_duration)

        raw_scale, raw_offset = linear_map(
            leader_closed, leader_open, follower_closed, follower_open)
        print('\n标定结果（STM32 原始位置单位）：')
        print(f'  leader_closed: {leader_closed:.9f}')
        print(f'  leader_open: {leader_open:.9f}')
        print(f'  follower_raw_closed: {follower_closed:.9f}')
        print(f'  follower_raw_open: {follower_open:.9f}')
        print(f'  follower_raw = {raw_scale:.9f} * leader + {raw_offset:.9f}')

        if parsed.follower_travel_m is None:
            print('\n尚未生成 ROS 米制参数。请测量指尖单侧实际行程后，')
            print('使用 --follower-travel-m <米> 再运行一次。')
        else:
            ros_scale, ros_offset = linear_map(
                follower_closed, follower_open,
                0.0, parsed.follower_travel_m)
            leader_scale, leader_offset = linear_map(
                leader_closed, leader_open,
                0.0, parsed.follower_travel_m)
            print('\n建议的 HardwareInterface 原始值→ROS 米制转换：')
            print(f'  gripper_position_scale: {ros_scale:.12f}')
            print(f'  gripper_position_offset: {ros_offset:.12f}')
            print('\n建议的 relay 遥操作映射：')
            print('  joint7_left: gripper_finger_left_joint')
            print('  gripper_finger_left_joint:')
            print(f'    sign: {leader_scale:.12f}')
            print(f'    offset: {leader_offset:.12f}')
            print(
                f'    leader_limit: [{min(leader_closed, leader_open):.9f}, '
                f'{max(leader_closed, leader_open):.9f}]')
            print(
                f'    follower_limit: [0.0, {parsed.follower_travel_m:.9f}]')
    except (KeyboardInterrupt, EOFError):
        print('\n标定已取消。')
    except (RuntimeError, ValueError) as error:
        raise SystemExit(f'ERROR: {error}') from error
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
