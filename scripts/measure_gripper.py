#!/usr/bin/env python3
"""测量主从臂夹爪映射关系"""

import argparse
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import threading


class GripperMeasurer(Node):
    def __init__(self, leader_topic, follower_topic):
        super().__init__('gripper_measurer')

        self.leader_pos = None
        self.follower_pos = None
        self.leader_name = None
        self.follower_name = None
        self.lock = threading.Lock()

        # Subscribe to leader
        self.create_subscription(
            JointState, leader_topic, self.leader_cb, 10)

        # Subscribe to follower
        self.create_subscription(
            JointState, follower_topic, self.follower_cb, 10)

        self.get_logger().info(f'Listening to:')
        self.get_logger().info(f'  Leader: {leader_topic}')
        self.get_logger().info(f'  Follower: {follower_topic}')

    def leader_cb(self, msg):
        for i, name in enumerate(msg.name):
            if 'joint7' in name.lower() or 'gripper' in name.lower():
                with self.lock:
                    self.leader_pos = msg.position[i]
                    self.leader_name = name
                break

    def follower_cb(self, msg):
        for i, name in enumerate(msg.name):
            if 'gripper' in name.lower():
                with self.lock:
                    self.follower_pos = msg.position[i]
                    self.follower_name = name
                break

    def get_status(self):
        with self.lock:
            return self.leader_name, self.leader_pos, self.follower_name, self.follower_pos


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
    parser = argparse.ArgumentParser(description='测量主从臂夹爪映射')
    parser.add_argument('--leader-topic', default='/joint_states',
                        help='主臂话题 (default: /joint_states)')
    parser.add_argument('--follower-topic', default='/follower/joint_states',
                        help='从臂话题 (default: /follower/joint_states)')
    parser.add_argument('--domain', type=int, default=0,
                        help='ROS_DOMAIN_ID')
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
    node = GripperMeasurer(args.leader_topic, args.follower_topic)

    print("\n" + "="*60)
    print("夹爪映射测量工具")
    print("="*60)
    print(f"\n配置文件: {config_path}")
    print(f"当前 sign: {current_sign}")
    print(f"当前 offset: {current_offset}")
    print("\n操作步骤:")
    print("1. 完全闭合夹爪，按 Enter 记录")
    print("2. 完全张开夹爪，按 Enter 记录")
    print("3. 重复几次确认一致性")
    print("4. 输入 'q' 退出并计算映射")
    print("="*60 + "\n")

    leader_closed = None
    leader_open = None
    follower_closed = None
    follower_open = None
    samples = []

    try:
        while True:
            # Spin once to process callbacks
            rclpy.spin_once(node, timeout_sec=0.1)

            ln, lp, fn, fp = node.get_status()

            if lp is not None and fp is not None:
                print(f"\r  Leader({ln}): {lp:+.4f} rad  |  Follower({fn}): {fp:+.6f} m  ", end='', flush=True)
            elif lp is not None:
                print(f"\r  Leader({ln}): {lp:+.4f} rad  |  Follower: waiting...  ", end='', flush=True)
            else:
                print(f"\r  Leader: waiting...  |  Follower: waiting...  ", end='', flush=True)

            # Check for user input (non-blocking)
            import select
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line.lower() == 'q':
                    break
                elif line == '':
                    # Enter pressed - record current position
                    if lp is not None and fp is not None:
                        samples.append((lp, fp))
                        print(f"\n  ✓ 记录 #{len(samples)}: leader={lp:.4f}, follower={fp:.6f}")
                    else:
                        print("\n  ✗ 数据未就绪，跳过")

    except KeyboardInterrupt:
        pass

    print("\n\n" + "="*60)
    print("测量结果")
    print("="*60)

    if len(samples) >= 2:
        # Assume first sample is closed, last is open
        leader_closed, follower_closed = samples[0]
        leader_open, follower_open = samples[-1]

        leader_range = leader_open - leader_closed
        follower_range = follower_open - follower_closed

        if abs(leader_range) > 0.001:
            sign = follower_range / leader_range
            offset = follower_closed - sign * leader_closed

            print(f"\n闭合位置:")
            print(f"  Leader:  {leader_closed:.4f} rad")
            print(f"  Follower: {follower_closed:.6f} m")

            print(f"\n张开位置:")
            print(f"  Leader:  {leader_open:.4f} rad")
            print(f"  Follower: {follower_open:.6f} m")

            print(f"\n计算映射:")
            print(f"  Leader 范围:  {leader_range:.4f} rad")
            print(f"  Follower 范围: {follower_range:.6f} m")
            print(f"  sign = {sign:.6f} m/rad")
            print(f"  offset = {offset:.6f} m")

            print(f"\n配置文件更新 ({os.path.basename(config_path)}):")
            print(f"  gripper_finger_left_joint:")
            print(f"    sign: {sign:.6f}")
            print(f"    offset: {offset:.6f}")
            print(f"    leader_limit: [{min(leader_closed, leader_open):.4f}, {max(leader_closed, leader_open):.4f}]")
            print(f"    follower_limit: [{min(follower_closed, follower_open):.6f}, {max(follower_closed, follower_open):.6f}]")

            print(f"\n验证公式:")
            print(f"  follower = {sign:.6f} × leader + {offset:.6f}")
            for i, (l, f) in enumerate(samples):
                calc = sign * l + offset
                err = calc - f
                print(f"  样本{i+1}: 计算={calc:.6f}, 实测={f:.6f}, 误差={err:.6f}")
        else:
            print("\n✗ Leader 范围太小，无法计算映射")
    else:
        print(f"\n样本不足 ({len(samples)}/2)")
        print("请至少记录闭合和张开两个位置")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
