#!/usr/bin/env python3
"""
步骤6: 实时监控 joint4
用途: 实时显示 joint4 状态，带抖动检测和报警
运行位置: Jetson
"""

import argparse
import sys
import time
from datetime import datetime
from collections import deque

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False

try:
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ============================================================================
# 配置
# ============================================================================
JOINT4_INDEX = 3  # joint4 在数组中的索引 (0-indexed)
BUFFER_SIZE = 500  # 显示的数据点数

# 抖动检测参数
JITTER_WINDOW = 20       # 检测窗口大小
JITTER_THRESHOLD = 0.02  # 抖动阈值 (rad)
ALERT_COOLDOWN = 2.0     # 报警冷却时间 (秒)


# ============================================================================
# 监控节点
# ============================================================================
class Joint4Monitor(Node):
    """实时监控 joint4 状态"""

    def __init__(self, use_plot: bool = False):
        super().__init__('joint4_monitor')

        self.use_plot = use_plot

        # 数据存储
        self.timestamps = deque(maxlen=BUFFER_SIZE)
        self.positions = deque(maxlen=BUFFER_SIZE)
        self.velocities = deque(maxlen=BUFFER_SIZE)
        self.efforts = deque(maxlen=BUFFER_SIZE)

        # 抖动检测
        self.accel_history = deque(maxlen=JITTER_WINDOW)
        self.jitter_detected = False
        self.last_alert_time = 0
        self.alert_count = 0

        # 统计
        self.sample_count = 0
        self.start_time = time.time()

        # 订阅
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        self.get_logger().info('Joint4 监控器已启动')
        self.get_logger().info(f'监控 joint4 (索引: {JOINT4_INDEX})')

    def joint_callback(self, msg: JointState):
        """关节数据回调"""
        if len(msg.position) <= JOINT4_INDEX:
            return

        # 获取时间戳
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if ts == 0:
            ts = time.time()

        # 提取数据
        pos = msg.position[JOINT4_INDEX]
        vel = msg.velocity[JOINT4_INDEX] if len(msg.velocity) > JOINT4_INDEX else 0.0
        eff = msg.effort[JOINT4_INDEX] if len(msg.effort) > JOINT4_INDEX else 0.0

        # 存储
        self.timestamps.append(ts)
        self.positions.append(pos)
        self.velocities.append(vel)
        self.efforts.append(eff)
        self.sample_count += 1

        # 抖动检测
        self.detect_jitter(pos)

    def detect_jitter(self, current_pos: float):
        """检测抖动"""
        if len(self.positions) < 2:
            return

        # 计算加速度估计
        dt = self.timestamps[-1] - self.timestamps[-2]
        if dt <= 0:
            return

        velocity = (self.positions[-1] - self.positions[-2]) / dt
        if len(self.velocities) >= 2:
            prev_vel = (self.positions[-2] - self.positions[-3]) / dt if len(self.positions) >= 3 else 0
            accel = (velocity - prev_vel) / dt
            self.accel_history.append(accel)

        # 检测抖动
        if len(self.accel_history) >= JITTER_WINDOW:
            accel_array = np.array(self.accel_history)
            accel_std = np.std(accel_array)

            if accel_std > JITTER_THRESHOLD:
                self.jitter_detected = True
                current_time = time.time()

                # 报警冷却
                if current_time - self.last_alert_time > ALERT_COOLDOWN:
                    self.last_alert_time = current_time
                    self.alert_count += 1
                    self.get_logger().warn(
                        f'⚠️  检测到抖动! 加速度标准差: {accel_std:.4f} '
                        f'(阈值: {JITTER_THRESHOLD})'
                    )
            else:
                self.jitter_detected = False


# ============================================================================
# 文本模式
# ============================================================================
def run_text_mode(monitor: Joint4Monitor, duration: float):
    """文本模式运行"""
    print("\n" + "=" * 70)
    print("Joint4 实时监控")
    print("=" * 70)
    print("\n按 Ctrl+C 提前结束\n")

    start_time = time.time()
    last_print = 0

    try:
        while time.time() - start_time < duration:
            rclpy.spin_once(monitor, timeout_sec=0.1)

            # 每 0.5 秒打印一次状态
            current_time = time.time()
            if current_time - last_print >= 0.5:
                last_print = current_time
                elapsed = current_time - start_time

                if monitor.positions:
                    pos = monitor.positions[-1]
                    vel = monitor.velocities[-1] if monitor.velocities else 0
                    eff = monitor.efforts[-1] if monitor.efforts else 0

                    # 计算实时统计
                    recent_pos = list(monitor.positions)[-50:]
                    pos_std = np.std(recent_pos) if len(recent_pos) > 1 else 0

                    # 状态指示
                    status = "🔴 抖动" if monitor.jitter_detected else "🟢 正常"

                    print(f"\r[{elapsed:5.1f}s] "
                          f"位置: {np.degrees(pos):7.2f}° | "
                          f"速度: {vel:6.3f} | "
                          f"力矩: {eff:6.3f} | "
                          f"抖动: {pos_std:.4f} | "
                          f"{status} | "
                          f"报警: {monitor.alert_count}",
                          end='', flush=True)

    except KeyboardInterrupt:
        print("\n\n用户中断")

    print("\n")


# ============================================================================
# 图形模式
# ============================================================================
def run_plot_mode(monitor: Joint4Monitor, duration: float):
    """图形模式运行"""
    if not HAS_MATPLOTLIB:
        print("错误: 图形模式需要 matplotlib")
        print("  pip install matplotlib")
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Joint4 实时监控')

    # 初始化曲线
    pos_line, = axes[0].plot([], [], 'b-', linewidth=0.5, label='位置')
    vel_line, = axes[1].plot([], [], 'g-', linewidth=0.5, label='速度')
    eff_line, = axes[2].plot([], [], 'r-', linewidth=0.5, label='力矩')

    # 抖动区域标记
    jitter_regions = []

    # 设置坐标轴
    axes[0].set_ylabel('位置 (rad)')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_ylabel('速度 (rad/s)')
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].set_xlabel('时间 (s)')
    axes[2].set_ylabel('力矩')
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    # 状态文本
    status_text = fig.text(0.02, 0.02, '', fontsize=10, family='monospace')

    start_time = time.time()

    def update(frame):
        rclpy.spin_once(monitor, timeout_sec=0)

        if not monitor.timestamps:
            return pos_line, vel_line, eff_line

        # 计算相对时间
        t0 = monitor.timestamps[0]
        times = [t - t0 for t in monitor.timestamps]

        # 更新曲线
        pos_line.set_data(times, list(monitor.positions))
        vel_line.set_data(times, list(monitor.velocities))
        eff_line.set_data(times, list(monitor.efforts))

        # 调整坐标轴
        for ax in axes:
            ax.relim()
            ax.autoscale_view()

        # 更新状态文本
        elapsed = time.time() - start_time
        status = "抖动检测" if monitor.jitter_detected else "正常"
        status_text.set_text(
            f'运行: {elapsed:.1f}s | '
            f'样本: {monitor.sample_count} | '
            f'状态: {status} | '
            f'报警: {monitor.alert_count}'
        )

        return pos_line, vel_line, eff_line, status_text

    # 动画
    frames = int(duration * 10)  # 10 FPS
    ani = animation.FuncAnimation(fig, update, frames=frames,
                                  interval=100, blit=False)

    plt.tight_layout()
    plt.show()


# ============================================================================
# 统计报告
# ============================================================================
def generate_report(monitor: Joint4Monitor):
    """生成监控报告"""
    print("\n" + "=" * 60)
    print("Joint4 监控报告")
    print("=" * 60)
    print()
    print(f"监控时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"运行时长: {time.time() - monitor.start_time:.1f} 秒")
    print(f"总样本数: {monitor.sample_count}")
    print()

    if not monitor.positions:
        print("无数据")
        return

    # 位置统计
    pos_array = np.array(monitor.positions)
    print("-" * 40)
    print("位置统计")
    print("-" * 40)
    print(f"范围: {np.degrees(np.min(pos_array)):.2f}° ~ {np.degrees(np.max(pos_array)):.2f}°")
    print(f"平均: {np.degrees(np.mean(pos_array)):.2f}°")
    print(f"标准差: {np.degrees(np.std(pos_array)):.4f}°")
    print()

    # 速度统计
    if monitor.velocities:
        vel_array = np.array(monitor.velocities)
        print("-" * 40)
        print("速度统计")
        print("-" * 40)
        print(f"范围: {np.min(vel_array):.3f} ~ {np.max(vel_array):.3f} rad/s")
        print(f"平均: {np.mean(vel_array):.3f} rad/s")
        print(f"标准差: {np.std(vel_array):.4f} rad/s")
        print()

    # 抖动统计
    print("-" * 40)
    print("抖动统计")
    print("-" * 40)
    print(f"报警次数: {monitor.alert_count}")

    if monitor.positions and len(monitor.positions) > 100:
        # 计算整体抖动指标
        pos_diff = np.diff(list(monitor.positions))
        jitter_metric = np.std(pos_diff)
        print(f"抖动指标: {jitter_metric:.6f}")

        if jitter_metric > 0.01:
            print("⚠️  存在明显抖动")
        elif jitter_metric > 0.005:
            print("⚠️  存在轻微抖动")
        else:
            print("✅ 运行平稳")

    print()
    print("=" * 60)


# ============================================================================
# 主函数
# ============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='实时监控 joint4 状态',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 06_monitor_joint4.py                  # 文本模式
  python3 06_monitor_joint4.py --plot            # 图形模式
  python3 06_monitor_joint4.py -d 60            # 监控 60 秒
  python3 06_monitor_joint4.py --threshold 0.05 # 调整抖动阈值
        """
    )

    parser.add_argument('-d', '--duration', type=float, default=30.0,
                       help='监控时长（秒），默认: 30')
    parser.add_argument('--plot', action='store_true',
                       help='使用图形模式')
    parser.add_argument('--joint', type=int, default=4,
                       help='监控的关节编号 (1-7)，默认: 4')
    parser.add_argument('--threshold', type=float, default=0.02,
                       help='抖动检测阈值 (rad)，默认: 0.02')
    parser.add_argument('--domain', type=int, default=0,
                       help='ROS_DOMAIN_ID，默认: 0')

    args = parser.parse_args()

    # 设置关节索引
    global JOINT4_INDEX, JITTER_THRESHOLD
    JOINT4_INDEX = args.joint - 1
    JITTER_THRESHOLD = args.threshold

    if JOINT4_INDEX < 0 or JOINT4_INDEX >= 7:
        print(f"错误: 关节编号必须在 1-7 之间")
        sys.exit(1)

    # 设置 ROS domain
    import os
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)

    # 初始化 ROS
    if not HAS_RCLPY:
        print("错误: 需要安装 ROS2 Python 库")
        print("  source /opt/ros/humble/setup.bash")
        sys.exit(1)

    rclpy.init()

    try:
        # 创建节点
        monitor = Joint4Monitor(use_plot=args.plot)

        print(f"监控关节: joint{args.joint} (索引: {JOINT4_INDEX})")
        print(f"抖动阈值: {JITTER_THRESHOLD} rad")
        print(f"ROS_DOMAIN_ID: {args.domain}")

        if args.plot:
            run_plot_mode(monitor, args.duration)
        else:
            run_text_mode(monitor, args.duration)

        # 生成报告
        generate_report(monitor)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
