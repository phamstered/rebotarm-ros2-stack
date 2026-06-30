#!/usr/bin/env python3
"""测量主从臂夹爪零点偏移"""

import argparse
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading


class ZeroOffsetMeasurer(Node):
    def __init__(self, leader_topic, follower_topic):
        super().__init__('zero_offset_measurer')

        self.leader_pos = None
        self.follower_pos = None
        self.lock = threading.Lock()
        self.samples = []

        self.create_subscription(
            JointState, leader_topic, self.leader_cb, 10)
        self.create_subscription(
            JointState, follower_topic, self.follower_cb, 10)

    def leader_cb(self, msg):
        for i, name in enumerate(msg.name):
            if 'joint7' in name.lower() or 'gripper' in name.lower():
                with self.lock:
                    self.leader_pos = msg.position[i]
                break

    def follower_cb(self, msg):
        for i, name in enumerate(msg.name):
            if 'gripper' in name.lower():
                with self.lock:
                    self.follower_pos = msg.position[i]
                break

    def sample(self):
        with self.lock:
            if self.leader_pos is not None and self.follower_pos is not None:
                self.samples.append((self.leader_pos, self.follower_pos))
                return True
        return False


def load_config(config_path):
    """从 YAML 配置文件读取 sign 和 offset"""
    try:
        import yaml
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        sign = config['joints']['gripper_finger_left_joint']['sign']
        offset = config['joints']['gripper_finger_left_joint']['offset']
        return sign, offset
    except Exception as e:
        print(f"警告: 无法读取配置文件 {config_path}: {e}")
        return None, None


def main():
    parser = argparse.ArgumentParser(description='测量主从臂夹爪零点偏移')
    parser.add_argument('--leader-topic', default='/joint_states')
    parser.add_argument('--follower-topic', default='/follower/joint_states')
    parser.add_argument('--domain', type=int, default=0)
    parser.add_argument('--duration', type=float, default=3.0, help='采样时长(秒)')
    parser.add_argument('--config', '-c', type=str, default=None,
                        help='YAML 配置文件路径 (默认: relay_real.yaml)')
    args = parser.parse_args()

    os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    # 确定配置文件路径
    if args.config:
        config_path = os.path.expanduser(args.config)
    else:
        # 默认使用项目目录下的 relay_real.yaml
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_dir = os.path.dirname(script_dir)
        config_path = os.path.join(project_dir, 'src/joint_state_relay/config/relay_real.yaml')

    # 读取当前配置
    current_sign, current_offset = load_config(config_path)
    if current_sign is None:
        print("错误: 无法读取配置文件，请使用 --config 指定路径")
        return

    rclpy.init()
    node = ZeroOffsetMeasurer(args.leader_topic, args.follower_topic)

    print("\n" + "="*60)
    print("零点偏移测量")
    print("="*60)
    print(f"\n配置文件: {config_path}")
    print(f"当前 sign: {current_sign}")
    print(f"当前 offset: {current_offset}")
    print("\n请确保:")
    print("1. 主臂夹爪已闭合到零点位置")
    print("2. 保持静止，等待采样完成")
    print("="*60)

    print(f"\n采样中 ({args.duration}秒)...", end='', flush=True)

    start = time.time()
    while time.time() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        if node.sample():
            print(".", end='', flush=True)
        time.sleep(0.05)

    samples = node.samples
    print(f"\n\n采样完成: {len(samples)} 个样本")

    if len(samples) < 10:
        print("样本不足，请检查话题连接")
        node.destroy_node()
        rclpy.shutdown()
        return

    # 计算平均值
    leader_avg = sum(s[0] for s in samples) / len(samples)
    follower_avg = sum(s[1] for s in samples) / len(samples)

    # 计算期望的 follower 位置
    expected_follower = current_sign * leader_avg + current_offset
    error = follower_avg - expected_follower

    print("\n" + "="*60)
    print("测量结果")
    print("="*60)
    print(f"\n主臂 (leader): {leader_avg:.6f} rad")
    print(f"从臂 (follower): {follower_avg:.6f} m")
    print(f"\n当前配置:")
    print(f"  sign = {current_sign}")
    print(f"  offset = {current_offset}")
    print(f"\n期望从臂位置: {expected_follower:.6f} m")
    print(f"实际从臂位置: {follower_avg:.6f} m")
    print(f"偏移误差: {error:.6f} m")

    # 计算新的 offset: follower = sign * leader + offset
    # offset = follower - sign * leader
    new_offset = follower_avg - current_sign * leader_avg
    print(f"\n" + "="*60)
    print("修正建议")
    print("="*60)
    print(f"\n更新 {os.path.basename(config_path)}:")
    print(f"  gripper_finger_left_joint:")
    print(f"    sign: {current_sign}")
    print(f"    offset: {new_offset:.6f}")

    # 验证
    calculated = current_sign * leader_avg + new_offset
    print(f"\n验证 (零点时):")
    print(f"  follower = {current_sign} × {leader_avg:.6f} + {new_offset:.6f}")
    print(f"  follower = {calculated:.6f} m")
    print(f"  实测: {follower_avg:.6f} m")
    print(f"  误差: {abs(calculated - follower_avg):.6f} m")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
