#!/usr/bin/env python3
"""
Gripper (joint7) 电机状态监控与抖动分析工具

用法:
  ros2 run joint_state_relay monitor_gripper          # 实时监控
  python3 scripts/monitor_gripper.py --duration 10    # 监控 10 秒后分析
  python3 scripts/monitor_gripper.py --log /tmp/gripper.log  # 从日志分析
"""

import argparse
import time
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class GripperMonitor(Node):
    def __init__(self, duration=10.0, window_size=200):
        super().__init__('gripper_monitor')
        self.duration = duration
        self.window_size = window_size

        # 数据缓冲区
        self.timestamps = deque(maxlen=window_size * 10)
        self.positions = deque(maxlen=window_size * 10)
        self.velocities = deque(maxlen=window_size * 10)
        self.efforts = deque(maxlen=window_size * 10)

        # 统计
        self.cmd_positions = deque(maxlen=window_size * 10)
        self.position_errors = deque(maxlen=window_size * 10)

        self.start_time = None
        self.last_cmd = None

        # 订阅
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(JointState, '/joint_commands', self._cmd_cb, 10)

        self.get_logger().info(f'Gripper monitor started, duration={duration}s')

    def _joint_cb(self, msg: JointState):
        now = time.monotonic()
        if self.start_time is None:
            self.start_time = now

        for i, name in enumerate(msg.name):
            # 匹配 gripper 或 joint7
            if 'gripper' in name.lower() or 'joint7' in name.lower():
                if i < len(msg.position):
                    self.timestamps.append(now - self.start_time)
                    self.positions.append(msg.position[i])
                    self.velocities.append(msg.velocity[i] if i < len(msg.velocity) else 0.0)
                    self.efforts.append(msg.effort[i] if i < len(msg.effort) else 0.0)

                    # 计算跟踪误差
                    if self.last_cmd is not None:
                        self.position_errors.append(msg.position[i] - self.last_cmd)
                    break

    def _cmd_cb(self, msg: JointState):
        for i, name in enumerate(msg.name):
            # 匹配 gripper 或 joint7
            if 'gripper' in name.lower() or 'joint7' in name.lower():
                if i < len(msg.position):
                    self.last_cmd = msg.position[i]
                    self.cmd_positions.append(msg.position[i])
                break

    def analyze(self):
        """分析收集的数据"""
        if len(self.positions) < 10:
            self.get_logger().warn('Not enough data for analysis')
            return None

        pos = np.array(self.positions)
        vel = np.array(self.velocities)
        eff = np.array(self.efforts)
        t = np.array(self.timestamps)

        results = {
            'duration': t[-1] - t[0],
            'sample_count': len(pos),
            'sample_rate': len(pos) / (t[-1] - t[0]) if t[-1] > t[0] else 0,
        }

        # 位置统计
        results['position'] = {
            'mean': float(np.mean(pos)),
            'std': float(np.std(pos)),
            'min': float(np.min(pos)),
            'max': float(np.max(pos)),
            'range': float(np.ptp(pos)),
        }

        # 速度统计
        results['velocity'] = {
            'mean': float(np.mean(vel)),
            'std': float(np.std(vel)),
            'max_abs': float(np.max(np.abs(vel))),
            'rms': float(np.sqrt(np.mean(vel**2))),
        }

        # 力矩统计
        results['effort'] = {
            'mean': float(np.mean(eff)),
            'std': float(np.std(eff)),
            'max_abs': float(np.max(np.abs(eff))),
        }

        # 抖动分析 - 通过速度过零检测
        zero_crossings = np.where(np.diff(np.sign(vel)))[0]
        if len(zero_crossings) > 2:
            # 计算抖动频率
            crossing_times = t[zero_crossings]
            if len(crossing_times) > 1:
                intervals = np.diff(crossing_times)
                results['jitter'] = {
                    'zero_crossings': len(zero_crossings),
                    'frequency_hz': float(1.0 / (2.0 * np.mean(intervals))) if np.mean(intervals) > 0 else 0,
                    'mean_interval': float(np.mean(intervals)),
                    'std_interval': float(np.std(intervals)),
                }

        # 跟踪误差分析
        if len(self.position_errors) > 0:
            errors = np.array(self.position_errors)
            results['tracking'] = {
                'mean_error': float(np.mean(errors)),
                'std_error': float(np.std(errors)),
                'max_error': float(np.max(np.abs(errors))),
            }

        # 位置变化率分析 (检测突变)
        if len(pos) > 1:
            pos_diff = np.diff(pos)
            pos_diff_abs = np.abs(pos_diff)
            results['position_change'] = {
                'mean_abs': float(np.mean(pos_diff_abs)),
                'max_abs': float(np.max(pos_diff_abs)),
                'large_jumps': int(np.sum(pos_diff_abs > 0.01)),  # >10mm jumps
            }

        return results


def print_analysis(results):
    """打印分析结果"""
    print("\n" + "="*60)
    print("Gripper 电机状态分析报告")
    print("="*60)

    print(f"\n📊 基本信息:")
    print(f"  监控时长: {results['duration']:.2f} s")
    print(f"  采样点数: {results['sample_count']}")
    print(f"  采样频率: {results['sample_rate']:.1f} Hz")

    print(f"\n📈 位置统计 (m):")
    p = results['position']
    print(f"  均值: {p['mean']:.6f}")
    print(f"  标准差: {p['std']:.6f}")
    print(f"  范围: [{p['min']:.6f}, {p['max']:.6f}]")
    print(f"  峰峰值: {p['range']:.6f}")

    print(f"\n📉 速度统计 (m/s):")
    v = results['velocity']
    print(f"  均值: {v['mean']:.6f}")
    print(f"  标准差: {v['std']:.6f}")
    print(f"  最大绝对值: {v['max_abs']:.6f}")
    print(f"  RMS: {v['rms']:.6f}")

    print(f"\n💪 力矩统计:")
    e = results['effort']
    print(f"  均值: {e['mean']:.4f}")
    print(f"  标准差: {e['std']:.4f}")
    print(f"  最大绝对值: {e['max_abs']:.4f}")

    if 'jitter' in results:
        j = results['jitter']
        print(f"\n🔀 抖动分析:")
        print(f"  速度过零次数: {j['zero_crossings']}")
        print(f"  估计抖动频率: {j['frequency_hz']:.2f} Hz")
        print(f"  过零间隔均值: {j['mean_interval']:.4f} s")
        print(f"  过零间隔标准差: {j['std_interval']:.4f} s")

    if 'tracking' in results:
        t = results['tracking']
        print(f"\n🎯 跟踪误差:")
        print(f"  平均误差: {t['mean_error']:.6f} m")
        print(f"  误差标准差: {t['std_error']:.6f} m")
        print(f"  最大误差: {t['max_error']:.6f} m")

    if 'position_change' in results:
        c = results['position_change']
        print(f"\n⚡ 位置突变检测:")
        print(f"  平均变化量: {c['mean_abs']:.6f} m")
        print(f"  最大变化量: {c['max_abs']:.6f} m")
        print(f"  大跳变次数 (>10mm): {c['large_jumps']}")

    # 诊断建议
    print(f"\n" + "="*60)
    print("🔍 诊断与建议")
    print("="*60)

    issues = []

    # 检查抖动
    if 'jitter' in results:
        freq = results['jitter']['frequency_hz']
        if freq > 5:
            issues.append(f"⚠️  高频抖动 ({freq:.1f} Hz) - 可能原因:")
            issues.append("   - kp 过高导致振荡")
            issues.append("   - kd 过低无法抑制振荡")
            issues.append("   - 建议: 降低 kp 或增加 kd")

    # 检查速度波动
    if results['velocity']['std'] > 0.1:
        issues.append(f"⚠️  速度波动大 (std={results['velocity']['std']:.4f}) - 可能原因:")
        issues.append("   - 目标位置频繁变化")
        issues.append("   - 滤波不足")
        issues.append("   - 建议: 增加 EMA 滤波系数或降低命令频率")

    # 检查跟踪误差
    if 'tracking' in results:
        if results['tracking']['max_error'] > 0.005:
            issues.append(f"⚠️  跟踪误差大 ({results['tracking']['max_error']*1000:.1f} mm) - 可能原因:")
            issues.append("   - kp 过低")
            issues.append("   - 负载过大")
            issues.append("   - 建议: 增加 kp 或检查机械")

    # 检查位置突变
    if 'position_change' in results:
        if results['position_change']['large_jumps'] > 5:
            issues.append(f"⚠️  频繁位置突变 ({results['position_change']['large_jumps']} 次) - 可能原因:")
            issues.append("   - 上游命令不稳定")
            issues.append("   - 通信丢包")
            issues.append("   - 建议: 检查 relay_node 和主臂数据")

    if not issues:
        print("✅ 未检测到明显问题")
    else:
        for issue in issues:
            print(issue)

    print()


def main():
    parser = argparse.ArgumentParser(description='Gripper 电机监控与抖动分析')
    parser.add_argument('--duration', '-d', type=float, default=10.0,
                        help='监控时长 (秒)')
    parser.add_argument('--domain', type=int, default=0,
                        help='ROS_DOMAIN_ID')
    args = parser.parse_args()

    import os
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    rclpy.init()
    node = GripperMonitor(duration=args.duration)

    print(f"🔍 开始监控 gripper 电机 ({args.duration} 秒)...")
    print(f"   请操作主臂夹爪以产生数据")

    start = time.monotonic()
    while rclpy.ok() and (time.monotonic() - start) < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)

        # 进度显示
        elapsed = time.monotonic() - start
        remaining = args.duration - elapsed
        if int(elapsed) % 2 == 0 and elapsed > 0:
            print(f"\r   已收集 {len(node.positions)} 个采样点, 剩余 {remaining:.1f}s", end='', flush=True)

    print("\n\n📊 分析中...")
    results = node.analyze()
    if results:
        print_analysis(results)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
