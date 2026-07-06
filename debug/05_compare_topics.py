#!/usr/bin/env python3
"""
步骤5: 对比 topic 数据
用途: 实时对比主臂原始数据和 relay 输出，检测差异
运行位置: Jetson
"""

import argparse
import sys
import time
from datetime import datetime

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
    from collections import deque
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ============================================================================
# 配置
# ============================================================================
JOINT4_INDEX = 3  # joint4 在数组中的索引 (0-indexed)
BUFFER_SIZE = 200  # 显示的数据点数


# ============================================================================
# 数据收集节点
# ============================================================================
class TopicComparator(Node):
    """对比两个 topic 的数据"""

    def __init__(self, duration: float = 10.0, use_plot: bool = False):
        super().__init__('topic_comparator')

        self.duration = duration
        self.use_plot = use_plot

        # 数据存储
        self.leader_data = {
            'timestamps': deque(maxlen=BUFFER_SIZE),
            'positions': deque(maxlen=BUFFER_SIZE),
        }
        self.relay_data = {
            'timestamps': deque(maxlen=BUFFER_SIZE),
            'positions': deque(maxlen=BUFFER_SIZE),
        }

        # 差异统计
        self.diffs = deque(maxlen=BUFFER_SIZE)
        self.max_diff = 0.0
        self.sample_count = 0

        # 订阅
        self.leader_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.leader_callback,
            10
        )

        self.relay_sub = self.create_subscription(
            JointState,
            '/joint_commands',
            self.relay_callback,
            10
        )

        self.get_logger().info('Topic 对比器已启动')
        self.get_logger().info(f'监控 joint4 (索引: {JOINT4_INDEX})')
        self.get_logger().info(f'持续时间: {duration} 秒')

    def leader_callback(self, msg: JointState):
        """主臂数据回调"""
        if len(msg.position) > JOINT4_INDEX:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if ts == 0:
                ts = time.time()
            self.leader_data['timestamps'].append(ts)
            self.leader_data['positions'].append(msg.position[JOINT4_INDEX])

    def relay_callback(self, msg: JointState):
        """Relay 数据回调"""
        if len(msg.position) > JOINT4_INDEX:
            ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if ts == 0:
                ts = time.time()
            self.relay_data['timestamps'].append(ts)
            self.relay_data['positions'].append(msg.position[JOINT4_INDEX])

            # 计算差异（如果有 leader 数据）
            if self.leader_data['positions']:
                leader_pos = self.leader_data['positions'][-1]
                relay_pos = msg.position[JOINT4_INDEX]
                diff = abs(relay_pos - leader_pos)
                self.diffs.append(diff)
                self.max_diff = max(self.max_diff, diff)
                self.sample_count += 1


# ============================================================================
# 文本模式
# ============================================================================
def run_text_mode(comparator: TopicComparator, duration: float):
    """文本模式运行"""
    print("\n" + "=" * 60)
    print("Topic 实时对比")
    print("=" * 60)
    print("\n按 Ctrl+C 提前结束\n")

    start_time = time.time()
    last_print = 0

    try:
        while time.time() - start_time < duration:
            rclpy.spin_once(comparator, timeout_sec=0.1)

            # 每秒打印一次状态
            current_time = time.time()
            if current_time - last_print >= 1.0:
                last_print = current_time
                elapsed = current_time - start_time

                leader_count = len(comparator.leader_data['positions'])
                relay_count = len(comparator.relay_data['positions'])

                print(f"\r[{elapsed:5.1f}s] "
                      f"Leader: {leader_count:4d} | "
                      f"Relay: {relay_count:4d} | "
                      f"差异: {comparator.max_diff:.6f} | "
                      f"样本: {comparator.sample_count}",
                      end='', flush=True)

    except KeyboardInterrupt:
        print("\n\n用户中断")

    print("\n")


# ============================================================================
# 图形模式
# ============================================================================
def run_plot_mode(comparator: TopicComparator, duration: float):
    """图形模式运行"""
    if not HAS_MATPLOTLIB:
        print("错误: 图形模式需要 matplotlib")
        print("  pip install matplotlib")
        return

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle('Joint4 Topic 实时对比')

    leader_line, = ax1.plot([], [], 'b-', label='Leader', linewidth=0.5)
    relay_line, = ax1.plot([], [], 'r-', label='Relay', linewidth=0.5)
    diff_line, = ax2.plot([], [], 'g-', label='差异', linewidth=0.5)

    ax1.set_ylabel('位置 (rad)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.set_xlabel('时间 (s)')
    ax2.set_ylabel('差异 (rad)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    start_time = time.time()

    def update(frame):
        rclpy.spin_once(comparator, timeout_sec=0)

        elapsed = time.time() - start_time

        # 更新 leader 曲线
        if comparator.leader_data['timestamps']:
            leader_t = [t - comparator.leader_data['timestamps'][0]
                        for t in comparator.leader_data['timestamps']]
            leader_p = list(comparator.leader_data['positions'])
            leader_line.set_data(leader_t, leader_p)

        # 更新 relay 曲线
        if comparator.relay_data['timestamps']:
            relay_t = [t - comparator.relay_data['timestamps'][0]
                       for t in comparator.relay_data['timestamps']]
            relay_p = list(comparator.relay_data['positions'])
            relay_line.set_data(relay_t, relay_p)

        # 更新差异曲线
        if comparator.diffs:
            diff_t = list(range(len(comparator.diffs)))
            diff_line.set_data(diff_t, list(comparator.diffs))

        # 调整坐标轴
        ax1.relim()
        ax1.autoscale_view()
        ax2.relim()
        ax2.autoscale_view()

        return leader_line, relay_line, diff_line

    # 动画
    frames = int(duration * 10)  # 10 FPS
    ani = animation.FuncAnimation(fig, update, frames=frames,
                                  interval=100, blit=False)

    plt.show()


# ============================================================================
# 分析报告
# ============================================================================
def generate_report(comparator: TopicComparator):
    """生成对比报告"""
    print("\n" + "=" * 60)
    print("Topic 对比报告")
    print("=" * 60)
    print()
    print(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()

    # 数据统计
    print("-" * 40)
    print("数据统计")
    print("-" * 40)
    print(f"Leader 数据点: {len(comparator.leader_data['positions'])}")
    print(f"Relay 数据点: {len(comparator.relay_data['positions'])}")
    print(f"差异样本数: {comparator.sample_count}")
    print()

    # 差异分析
    if comparator.diffs:
        diff_array = np.array(comparator.diffs)

        print("-" * 40)
        print("差异分析")
        print("-" * 40)
        print(f"平均差异: {np.mean(diff_array):.6f} rad ({np.degrees(np.mean(diff_array)):.4f}°)")
        print(f"最大差异: {np.max(diff_array):.6f} rad ({np.degrees(np.max(diff_array)):.4f}°)")
        print(f"标准差: {np.std(diff_array):.6f} rad")
        print()

        # 判断
        print("-" * 40)
        print("诊断结果")
        print("-" * 40)

        max_diff_deg = np.degrees(np.max(diff_array))

        if max_diff_deg < 0.1:
            print("✅ Leader 和 Relay 数据高度一致")
            print("   问题不在数据转换层")
        elif max_diff_deg < 1.0:
            print("⚠️  存在轻微差异")
            print("   可能原因:")
            print("   - 数据类型转换精度")
            print("   - 时间戳对齐问题")
        else:
            print("❌ 存在明显差异!")
            print("   Relay 可能引入了抖动")
            print("   建议检查 relay_node 转换逻辑")

    print()
    print("=" * 60)


# ============================================================================
# 主函数
# ============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='实时对比主臂和 relay 数据',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 05_compare_topics.py                  # 文本模式
  python3 05_compare_topics.py --plot            # 图形模式
  python3 05_compare_topics.py -d 30            # 运行 30 秒
  python3 05_compare_topics.py --joint 4        # 监控 joint4
        """
    )

    parser.add_argument('-d', '--duration', type=float, default=10.0,
                       help='运行时长（秒），默认: 10')
    parser.add_argument('--plot', action='store_true',
                       help='使用图形模式')
    parser.add_argument('--joint', type=int, default=4,
                       help='监控的关节编号 (1-7)，默认: 4')
    parser.add_argument('--domain', type=int, default=0,
                       help='ROS_DOMAIN_ID，默认: 0')

    args = parser.parse_args()

    # 设置关节索引
    global JOINT4_INDEX
    JOINT4_INDEX = args.joint - 1
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
        comparator = TopicComparator(
            duration=args.duration,
            use_plot=args.plot
        )

        print(f"监控关节: joint{args.joint} (索引: {JOINT4_INDEX})")
        print(f"ROS_DOMAIN_ID: {args.domain}")

        if args.plot:
            run_plot_mode(comparator, args.duration)
        else:
            run_text_mode(comparator, args.duration)

        # 生成报告
        generate_report(comparator)

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        comparator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
