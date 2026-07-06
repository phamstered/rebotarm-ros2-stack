#!/usr/bin/env python3
"""
步骤2: 分析 bag 数据
用途: 分析录制的关节数据，检测高频抖动，对比命令与实际响应
运行位置: 任意（需要 Python3 + rosbags）
"""

import argparse
import json
import sys
from pathlib import Path
from datetime import datetime

import numpy as np

try:
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import Stores, get_typestore
    HAS_ROSBAGS = True
except ImportError:
    HAS_ROSBAGS = False

try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')  # 非交互后端
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


# ============================================================================
# 配置
# ============================================================================
JOINT4_INDEX = 3  # joint4 在数组中的索引 (0-indexed)
JOINT_NAMES = [
    'joint1', 'joint2', 'joint3', 'joint4',
    'joint5', 'joint6', 'gripper_finger_left_joint'
]

# Topic 定义
TOPICS = {
    'leader': '/joint_states',
    'command': '/joint_commands',
    'follower': '/follower/joint_states'
}

# 抖动检测阈值
JITTER_THRESHOLD_DEG = 0.5      # 角度抖动阈值（度）
JITTER_THRESHOLD_VEL = 0.1      # 速度抖动阈值（rad/s）
HIGH_FREQ_THRESHOLD = 50.0      # 高频阈值（Hz）


# ============================================================================
# 数据加载
# ============================================================================
def load_all_topics(bag_path: Path) -> dict:
    """加载 bag 文件中所有 joint 相关 topic 的数据"""
    if not HAS_ROSBAGS:
        print("错误: 需要安装 rosbags 库")
        print("  pip install rosbags")
        sys.exit(1)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # 找到 bag 目录
    joint_states_bag = bag_path / 'joint_data'
    if not joint_states_bag.exists():
        joint_states_bag = bag_path

    # 初始化结果
    all_data = {}
    for name, topic in TOPICS.items():
        all_data[name] = {
            'topic': topic,
            'timestamps': [],
            'positions': [],
            'velocities': [],
            'efforts': []
        }

    try:
        with Reader(joint_states_bag) as reader:
            # 获取所有 joint 相关的 connections
            connections = [c for c in reader.connections if '/joint' in c.topic]

            if not connections:
                print(f"警告: 在 {joint_states_bag} 中未找到 joint 相关 topic")
                return all_data

            print(f"找到 {len(connections)} 个 joint topic:")
            for conn in connections:
                print(f"  - {conn.topic}")

            # 按 topic 分组处理
            for connection, timestamp, rawdata in reader.messages(connections):
                msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                # 确定属于哪个数据集
                data_key = None
                if connection.topic == '/joint_states':
                    data_key = 'leader'
                elif connection.topic == '/joint_commands':
                    data_key = 'command'
                elif connection.topic == '/follower/joint_states':
                    data_key = 'follower'

                if data_key is None:
                    continue

                data = all_data[data_key]

                # 转换时间戳（纳秒 -> 秒）
                ts = timestamp / 1e9
                data['timestamps'].append(ts)

                # 提取位置、速度、力矩
                if hasattr(msg, 'position'):
                    data['positions'].append(list(msg.position))
                if hasattr(msg, 'velocity'):
                    data['velocities'].append(list(msg.velocity))
                if hasattr(msg, 'effort'):
                    data['efforts'].append(list(msg.effort))

    except Exception as e:
        print(f"读取 bag 失败: {e}")
        return all_data

    # 转换为 numpy 数组
    for name in all_data:
        data = all_data[name]
        data['timestamps'] = np.array(data['timestamps'])
        if data['positions']:
            data['positions'] = np.array(data['positions'])
        else:
            data['positions'] = np.array([])
        if data['velocities']:
            data['velocities'] = np.array(data['velocities'])
        else:
            data['velocities'] = np.array([])
        if data['efforts']:
            data['efforts'] = np.array(data['efforts'])
        else:
            data['efforts'] = np.array([])

    return all_data


# ============================================================================
# 抖动检测
# ============================================================================
def detect_jitter(positions: np.ndarray, timestamps: np.ndarray, joint_idx: int = JOINT4_INDEX) -> dict:
    """检测关节抖动"""
    if positions.size == 0 or positions.shape[1] <= joint_idx:
        return {'detected': False, 'reason': '数据不足'}

    # 提取目标关节数据
    joint_pos = positions[:, joint_idx]

    # 计算一阶差分（速度估计）
    dt = np.diff(timestamps)
    dt[dt == 0] = 1e-6  # 避免除零
    velocity = np.diff(joint_pos) / dt

    # 计算二阶差分（加速度估计）
    accel = np.diff(velocity) / dt[:-1]

    # 检测抖动：加速度的高频变化
    window_size = min(20, len(accel) // 4)
    if window_size < 2:
        return {'detected': False, 'reason': '数据点不足'}

    accel_std = np.array([np.std(accel[max(0, i-window_size):i+1])
                          for i in range(len(accel))])

    jitter_mask = accel_std > np.median(accel_std) * 3
    jitter_ratio = np.sum(jitter_mask) / len(jitter_mask)

    result = {
        'detected': jitter_ratio > 0.1,
        'jitter_ratio': float(jitter_ratio),
        'accel_std_mean': float(np.mean(accel_std)),
        'accel_std_max': float(np.max(accel_std)),
        'velocity_range': float(np.max(velocity) - np.min(velocity)),
        'position_range_deg': float(np.degrees(np.max(joint_pos) - np.min(joint_pos)))
    }

    return result


def detect_high_freq_component(signal: np.ndarray, sample_rate: float,
                                 threshold_hz: float = HIGH_FREQ_THRESHOLD) -> dict:
    """检测信号中的高频分量"""
    n = len(signal)
    if n < 10:
        return {'has_high_freq': False, 'reason': '数据不足'}

    # FFT 分析
    fft_vals = np.fft.rfft(signal - np.mean(signal))
    freqs = np.fft.rfftfreq(n, d=1.0/sample_rate)
    magnitudes = np.abs(fft_vals)

    # 找到主要频率分量
    peak_idx = np.argmax(magnitudes[1:]) + 1
    peak_freq = freqs[peak_idx]
    peak_magnitude = magnitudes[peak_idx]

    # 计算高频能量占比
    high_freq_mask = freqs >= threshold_hz
    high_freq_energy = np.sum(magnitudes[high_freq_mask] ** 2)
    total_energy = np.sum(magnitudes ** 2)
    high_freq_ratio = high_freq_energy / total_energy if total_energy > 0 else 0

    return {
        'has_high_freq': high_freq_ratio > 0.1,
        'peak_freq_hz': float(peak_freq),
        'peak_magnitude': float(peak_magnitude),
        'high_freq_ratio': float(high_freq_ratio),
        'frequencies': freqs.tolist(),
        'magnitudes': magnitudes.tolist()
    }


# ============================================================================
# 统计分析
# ============================================================================
def compute_statistics(data: dict, joint_idx: int = JOINT4_INDEX) -> dict:
    """计算关节统计数据"""
    stats = {}

    if data['positions'].size > 0 and data['positions'].shape[1] > joint_idx:
        pos = data['positions'][:, joint_idx]
        stats['position'] = {
            'mean_deg': float(np.degrees(np.mean(pos))),
            'std_deg': float(np.degrees(np.std(pos))),
            'min_deg': float(np.degrees(np.min(pos))),
            'max_deg': float(np.degrees(np.max(pos))),
            'range_deg': float(np.degrees(np.max(pos) - np.min(pos)))
        }

    if data['velocities'].size > 0 and data['velocities'].shape[1] > joint_idx:
        vel = data['velocities'][:, joint_idx]
        stats['velocity'] = {
            'mean': float(np.mean(vel)),
            'std': float(np.std(vel)),
            'max': float(np.max(np.abs(vel)))
        }

    if data['efforts'].size > 0 and data['efforts'].shape[1] > joint_idx:
        eff = data['efforts'][:, joint_idx]
        stats['effort'] = {
            'mean': float(np.mean(eff)),
            'std': float(np.std(eff)),
            'max': float(np.max(np.abs(eff)))
        }

    # 时间统计
    if len(data['timestamps']) > 1:
        dt = np.diff(data['timestamps'])
        stats['timing'] = {
            'duration_s': float(data['timestamps'][-1] - data['timestamps'][0]),
            'sample_rate_hz': float(1.0 / np.mean(dt)),
            'sample_rate_std': float(np.std(1.0 / dt)),
            'num_samples': len(data['timestamps'])
        }

    return stats


# ============================================================================
# 命令跟随分析
# ============================================================================
def analyze_tracking(command_data: dict, follower_data: dict, joint_idx: int = JOINT4_INDEX) -> dict:
    """分析命令跟随性能"""
    if command_data['positions'].size == 0 or follower_data['positions'].size == 0:
        return {'available': False, 'reason': '数据不足'}

    if command_data['positions'].shape[1] <= joint_idx or follower_data['positions'].shape[1] <= joint_idx:
        return {'available': False, 'reason': '关节索引越界'}

    cmd_pos = command_data['positions'][:, joint_idx]
    fdr_pos = follower_data['positions'][:, joint_idx]

    # 对齐时间序列（使用最近邻插值）
    cmd_ts = command_data['timestamps']
    fdr_ts = follower_data['timestamps']

    # 简单处理：取较短的长度
    min_len = min(len(cmd_pos), len(fdr_pos))
    if min_len < 10:
        return {'available': False, 'reason': '数据点不足'}

    cmd_pos = cmd_pos[:min_len]
    fdr_pos = fdr_pos[:min_len]

    # 计算跟踪误差
    error = fdr_pos - cmd_pos
    error_deg = np.degrees(error)

    result = {
        'available': True,
        'error_mean_deg': float(np.mean(np.abs(error_deg))),
        'error_max_deg': float(np.max(np.abs(error_deg))),
        'error_std_deg': float(np.std(error_deg)),
        'correlation': float(np.corrcoef(cmd_pos, fdr_pos)[0, 1]) if min_len > 1 else 0.0
    }

    # 判断跟随质量
    if result['error_max_deg'] > 5.0:
        result['quality'] = '差'
        result['quality_detail'] = '最大误差超过 5°'
    elif result['error_mean_deg'] > 1.0:
        result['quality'] = '中等'
        result['quality_detail'] = '平均误差超过 1°'
    else:
        result['quality'] = '良好'
        result['quality_detail'] = '跟随误差在可接受范围'

    return result


# ============================================================================
# 可视化
# ============================================================================
def plot_analysis(all_data: dict, output_dir: Path, joint_idx: int = JOINT4_INDEX):
    """生成分析图表"""
    if not HAS_MATPLOTLIB:
        print("警告: 未安装 matplotlib，跳过图表生成")
        return

    fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(f'Joint4 振动分析（三 topic 对比）', fontsize=14)

    colors = {'leader': 'blue', 'command': 'green', 'follower': 'red'}
    labels = {'leader': '主臂', 'command': '命令', 'follower': '从臂'}

    has_data = False

    # 1. 位置对比
    for name, data in all_data.items():
        if data['positions'].size > 0 and data['positions'].shape[1] > joint_idx:
            t = data['timestamps'] - data['timestamps'][0]
            pos_deg = np.degrees(data['positions'][:, joint_idx])
            axes[0].plot(t, pos_deg, color=colors[name], linewidth=0.5, label=labels[name])
            has_data = True

    axes[0].set_ylabel('位置 (deg)')
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('关节位置对比')

    # 2. 速度对比
    for name, data in all_data.items():
        if data['velocities'].size > 0 and data['velocities'].shape[1] > joint_idx:
            t = data['timestamps'] - data['timestamps'][0]
            vel = data['velocities'][:, joint_idx]
            axes[1].plot(t, vel, color=colors[name], linewidth=0.5, label=labels[name])

    axes[1].set_ylabel('速度 (rad/s)')
    axes[1].legend(loc='upper right')
    axes[1].grid(True, alpha=0.3)
    axes[1].set_title('关节速度对比')

    # 3. 跟踪误差
    if (all_data['command']['positions'].size > 0 and
        all_data['follower']['positions'].size > 0 and
        all_data['command']['positions'].shape[1] > joint_idx and
        all_data['follower']['positions'].shape[1] > joint_idx):

        cmd_pos = all_data['command']['positions'][:, joint_idx]
        fdr_pos = all_data['follower']['positions'][:, joint_idx]
        min_len = min(len(cmd_pos), len(fdr_pos))

        if min_len > 0:
            error_deg = np.degrees(fdr_pos[:min_len] - cmd_pos[:min_len])
            t = all_data['command']['timestamps'][:min_len] - all_data['command']['timestamps'][0]
            axes[2].plot(t, error_deg, 'purple', linewidth=0.5)
            axes[2].axhline(y=0, color='gray', linestyle='--', alpha=0.5)

    axes[2].set_ylabel('误差 (deg)')
    axes[2].grid(True, alpha=0.3)
    axes[2].set_title('跟随误差 (从臂 - 命令)')

    # 4. FFT 频谱（从臂）
    if (all_data['follower']['positions'].size > 0 and
        all_data['follower']['positions'].shape[1] > joint_idx):

        signal = all_data['follower']['positions'][:, joint_idx]
        if len(signal) > 10:
            sample_rate = 1.0 / np.mean(np.diff(all_data['follower']['timestamps']))
            fft_vals = np.fft.rfft(signal - np.mean(signal))
            freqs = np.fft.rfftfreq(len(signal), d=1.0/sample_rate)
            magnitudes = np.abs(fft_vals)

            axes[3].semilogy(freqs[1:], magnitudes[1:], 'red', linewidth=0.5, label='从臂')
            axes[3].axvline(x=50, color='gray', linestyle='--', alpha=0.5, label='50Hz')
            axes[3].set_xlabel('频率 (Hz)')
            axes[3].set_ylabel('幅值')
            axes[3].grid(True, alpha=0.3)
            axes[3].set_title('从臂频谱分析')
            axes[3].legend(loc='upper right')
            axes[3].set_xlim([0, min(200, sample_rate/2)])

    plt.tight_layout()

    # 保存图表
    plot_path = output_dir / 'vibration_analysis.png'
    plt.savefig(plot_path, dpi=150)
    print(f"图表已保存: {plot_path}")
    plt.close()


# ============================================================================
# 报告生成
# ============================================================================
def generate_report(all_data: dict, all_stats: dict, all_jitter: dict,
                    all_freq: dict, tracking: dict, output_dir: Path,
                    joint_idx: int = JOINT4_INDEX):
    """生成分析报告"""
    report = []
    report.append("=" * 70)
    report.append("Joint4 振动分析报告（三 topic 对比）")
    report.append("=" * 70)
    report.append("")
    report.append(f"分析时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report.append(f"分析关节: {JOINT_NAMES[joint_idx]} (索引: {joint_idx})")
    report.append("")

    # 各 topic 统计
    for name, label in [('leader', '主臂'), ('command', '命令'), ('follower', '从臂')]:
        report.append("-" * 50)
        report.append(f"{label} ({TOPICS[name]})")
        report.append("-" * 50)

        stats = all_stats.get(name, {})
        jitter = all_jitter.get(name, {})

        if 'timing' in stats:
            report.append(f"  数据点: {stats['timing']['num_samples']}")
            report.append(f"  采样率: {stats['timing']['sample_rate_hz']:.1f} Hz")

        if 'position' in stats:
            report.append(f"  位置范围: {stats['position']['min_deg']:.2f}° ~ {stats['position']['max_deg']:.2f}°")
            report.append(f"  位置标准差: {stats['position']['std_deg']:.4f}°")

        if jitter.get('detected'):
            report.append(f"  ⚠️  检测到抖动 (比率: {jitter['jitter_ratio']:.1%})")
        else:
            report.append(f"  ✅ 未检测到明显抖动")

        report.append("")

    # 跟踪性能
    report.append("-" * 50)
    report.append("跟随性能分析")
    report.append("-" * 50)

    if tracking.get('available'):
        report.append(f"  跟随质量: {tracking['quality']}")
        report.append(f"  平均误差: {tracking['error_mean_deg']:.3f}°")
        report.append(f"  最大误差: {tracking['error_max_deg']:.3f}°")
        report.append(f"  相关系数: {tracking['correlation']:.4f}")
        report.append(f"  评价: {tracking['quality_detail']}")
    else:
        reason = tracking.get('reason', '未知')
        report.append(f"  无法分析: {reason}")

    report.append("")

    # 频率分析
    report.append("-" * 50)
    report.append("频率分析 (从臂)")
    report.append("-" * 50)

    freq = all_freq.get('follower', {})
    if 'peak_freq_hz' in freq:
        report.append(f"  主频: {freq['peak_freq_hz']:.1f} Hz")
        report.append(f"  高频能量占比: {freq['high_freq_ratio']:.1%}")
        if freq.get('has_high_freq'):
            report.append("  ⚠️  存在显著高频分量")

    report.append("")

    # 诊断建议
    report.append("-" * 50)
    report.append("诊断建议")
    report.append("-" * 50)

    # 综合判断
    follower_jitter = all_jitter.get('follower', {})
    command_jitter = all_jitter.get('command', {})

    if follower_jitter.get('detected') and not command_jitter.get('detected'):
        report.append("• 命令平滑但从臂抖动 → 问题在从臂驱动层")
        report.append("  - 检查 Kd 增益是否过高")
        report.append("  - 检查机械结构")
    elif follower_jitter.get('detected') and command_jitter.get('detected'):
        report.append("• 命令和从臂都有抖动 → 问题在数据源或 relay")
        report.append("  - 检查主臂数据质量")
        report.append("  - 检查 relay 转换逻辑")
    elif not follower_jitter.get('detected') and not command_jitter.get('detected'):
        report.append("• 数据层面无明显抖动")
        report.append("  - 如果仍有振动感，可能是机械共振")
        report.append("  - 建议检查机械间隙和螺丝紧固")

    if tracking.get('available') and tracking.get('error_max_deg', 0) > 5:
        report.append("• 跟随误差较大")
        report.append("  - 检查电机响应速度")
        report.append("  - 检查通信延迟")

    report.append("")
    report.append("=" * 70)

    # 保存报告
    report_text = "\n".join(report)
    report_path = output_dir / 'analysis_report.txt'
    with open(report_path, 'w', encoding='utf-8') as f:
        f.write(report_text)

    # 打印报告
    print(report_text)

    return report_text


# ============================================================================
# 主函数
# ============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='分析录制的关节数据，检测高频抖动',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 02_analyze_bag.py vibration_20240101_120000/
  python3 02_analyze_bag.py --joint 4 data/
  python3 02_analyze_bag.py --no-plot data/
        """
    )

    parser.add_argument('bag_dir', type=str, help='bag 数据目录')
    parser.add_argument('--joint', type=int, default=4,
                       help='要分析的关节编号 (1-7)，默认: 4')
    parser.add_argument('--threshold', type=float, default=0.5,
                       help='抖动检测阈值（度），默认: 0.5')
    parser.add_argument('--no-plot', action='store_true',
                       help='不生成图表')
    parser.add_argument('--output', type=str, default=None,
                       help='输出目录，默认: bag_dir 内')

    args = parser.parse_args()

    # 调整关节索引
    joint_idx = args.joint - 1
    if joint_idx < 0 or joint_idx >= len(JOINT_NAMES):
        print(f"错误: 关节编号必须在 1-{len(JOINT_NAMES)} 之间")
        sys.exit(1)

    global JITTER_THRESHOLD_DEG
    JITTER_THRESHOLD_DEG = args.threshold

    bag_path = Path(args.bag_dir)
    if not bag_path.exists():
        print(f"错误: 目录不存在: {bag_path}")
        sys.exit(1)

    output_dir = Path(args.output) if args.output else bag_path
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"分析关节: {JOINT_NAMES[joint_idx]} (索引: {joint_idx})")
    print(f"数据目录: {bag_path}")
    print("")

    # 加载所有 topic 数据
    print("加载数据...")
    all_data = load_all_topics(bag_path)

    # 检查数据
    has_data = False
    for name, data in all_data.items():
        if data['timestamps'].size > 0:
            has_data = True
            print(f"  {name}: {len(data['timestamps'])} 个数据点")

    if not has_data:
        print("错误: 未找到有效数据")
        sys.exit(1)

    print("")

    # 统计分析
    print("计算统计...")
    all_stats = {}
    for name, data in all_data.items():
        if data['timestamps'].size > 0:
            all_stats[name] = compute_statistics(data, joint_idx)

    # 抖动检测
    print("检测抖动...")
    all_jitter = {}
    for name, data in all_data.items():
        if data['positions'].size > 0:
            all_jitter[name] = detect_jitter(data['positions'], data['timestamps'], joint_idx)

    # 频率分析
    print("频率分析...")
    all_freq = {}
    for name, data in all_data.items():
        if data['positions'].size > 0 and data['positions'].shape[1] > joint_idx:
            sample_rate = all_stats.get(name, {}).get('timing', {}).get('sample_rate_hz', 100)
            all_freq[name] = detect_high_freq_component(
                data['positions'][:, joint_idx], sample_rate
            )

    # 跟踪分析
    print("分析跟随性能...")
    tracking = analyze_tracking(
        all_data.get('command', {'positions': np.array([]), 'timestamps': np.array([])}),
        all_data.get('follower', {'positions': np.array([]), 'timestamps': np.array([])}),
        joint_idx
    )

    # 生成图表
    if not args.no_plot:
        print("生成图表...")
        plot_analysis(all_data, output_dir, joint_idx)

    # 生成报告
    print("")
    generate_report(all_data, all_stats, all_jitter, all_freq, tracking,
                    output_dir, joint_idx)

    # 保存原始数据为 JSON
    json_data = {
        'joint': JOINT_NAMES[joint_idx],
        'statistics': all_stats,
        'jitter_detection': {k: {kk: vv for kk, vv in v.items() if kk not in ['frequencies', 'magnitudes']}
                            for k, v in all_jitter.items()},
        'frequency_analysis': {k: {kk: vv for kk, vv in v.items() if kk not in ['frequencies', 'magnitudes']}
                              for k, v in all_freq.items()},
        'tracking': tracking
    }

    json_path = output_dir / 'analysis_result.json'
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(json_data, f, indent=2, ensure_ascii=False)
    print(f"\n详细数据已保存: {json_path}")


if __name__ == '__main__':
    main()
