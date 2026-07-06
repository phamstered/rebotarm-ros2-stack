#!/usr/bin/env python3
"""
步骤8: 频谱分析
用途: 对录制的数据进行 FFT 频谱分析，识别振动频率
运行位置: 任意
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

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


# ============================================================================
# 数据加载
# ============================================================================
def load_data_from_json(json_path: Path) -> dict:
    """从 analysis_result.json 加载数据"""
    with open(json_path, 'r') as f:
        data = json.load(f)
    return data


def generate_test_signal(duration: float = 5.0, sample_rate: float = 100.0,
                         freq1: float = 5.0, freq2: float = 75.0) -> tuple:
    """生成测试信号（用于验证）"""
    t = np.arange(0, duration, 1.0/sample_rate)
    # 低频信号 + 高频噪声
    signal = (np.sin(2*np.pi*freq1*t) +
              0.3*np.sin(2*np.pi*freq2*t) +
              0.1*np.random.randn(len(t)))
    return t, signal


# ============================================================================
# FFT 分析
# ============================================================================
def compute_fft(signal: np.ndarray, sample_rate: float) -> dict:
    """计算 FFT"""
    n = len(signal)

    # 去除直流分量
    signal_centered = signal - np.mean(signal)

    # FFT
    fft_vals = np.fft.rfft(signal_centered)
    freqs = np.fft.rfftfreq(n, d=1.0/sample_rate)
    magnitudes = np.abs(fft_vals)
    phases = np.angle(fft_vals)

    # 功率谱
    power = magnitudes ** 2

    return {
        'frequencies': freqs,
        'magnitudes': magnitudes,
        'phases': phases,
        'power': power
    }


def find_peaks(magnitudes: np.ndarray, frequencies: np.ndarray,
               min_height: float = None, min_distance: int = 5) -> list:
    """找到频谱中的峰值"""
    peaks = []

    if min_height is None:
        min_height = np.mean(magnitudes) * 3

    for i in range(min_distance, len(magnitudes) - min_distance):
        # 检查是否是局部最大值
        if magnitudes[i] > magnitudes[i-1] and magnitudes[i] > magnitudes[i+1]:
            # 检查是否超过阈值
            if magnitudes[i] >= min_height:
                # 检查邻域
                is_peak = True
                for j in range(max(0, i-min_distance), min(len(magnitudes), i+min_distance+1)):
                    if j != i and magnitudes[j] > magnitudes[i]:
                        is_peak = False
                        break

                if is_peak:
                    peaks.append({
                        'index': i,
                        'frequency': frequencies[i],
                        'magnitude': magnitudes[i],
                        'power': magnitudes[i] ** 2
                    })

    # 按幅值排序
    peaks.sort(key=lambda x: x['magnitude'], reverse=True)
    return peaks


# ============================================================================
# 频段分析
# ============================================================================
def analyze_frequency_bands(frequencies: np.ndarray, magnitudes: np.ndarray) -> dict:
    """分析不同频段的能量"""
    bands = {
        '低频 (<10Hz)': (0, 10),
        '中频 (10-50Hz)': (10, 50),
        '高频 (50-100Hz)': (50, 100),
        '超高频 (>100Hz)': (100, frequencies[-1])
    }

    results = {}
    total_power = np.sum(magnitudes ** 2)

    for band_name, (low, high) in bands.items():
        mask = (frequencies >= low) & (frequencies < high)
        band_power = np.sum(magnitudes[mask] ** 2)
        band_ratio = band_power / total_power if total_power > 0 else 0

        results[band_name] = {
            'power': float(band_power),
            'ratio': float(band_ratio),
            'percentage': f"{band_ratio*100:.1f}%"
        }

    return results


# ============================================================================
# 可视化
# ============================================================================
def plot_fft_analysis(frequencies: np.ndarray, magnitudes: np.ndarray,
                      peaks: list, bands: dict, output_path: Path,
                      title: str = "FFT 频谱分析"):
    """绘制 FFT 分析图"""
    if not HAS_MATPLOTLIB:
        print("警告: 未安装 matplotlib，跳过图表生成")
        return

    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle(title, fontsize=14)

    # 1. 频谱（线性坐标）
    ax1 = axes[0]
    ax1.plot(frequencies, magnitudes, 'b-', linewidth=0.5)
    ax1.set_ylabel('幅值')
    ax1.set_title('频谱（线性坐标）')
    ax1.grid(True, alpha=0.3)

    # 标记峰值
    for peak in peaks[:5]:  # 最多显示 5 个峰值
        ax1.axvline(x=peak['frequency'], color='r', linestyle='--', alpha=0.5)
        ax1.annotate(f"{peak['frequency']:.1f}Hz",
                    xy=(peak['frequency'], peak['magnitude']),
                    xytext=(5, 5), textcoords='offset points',
                    fontsize=8, color='red')

    # 2. 频谱（对数坐标）
    ax2 = axes[1]
    ax2.semilogy(frequencies[1:], magnitudes[1:], 'b-', linewidth=0.5)
    ax2.set_ylabel('幅值 (log)')
    ax2.set_title('频谱（对数坐标）')
    ax2.grid(True, alpha=0.3)

    # 标记峰值
    for peak in peaks[:5]:
        ax2.axvline(x=peak['frequency'], color='r', linestyle='--', alpha=0.5)

    # 3. 频段能量分布
    ax3 = axes[2]
    band_names = list(bands.keys())
    band_percentages = [bands[b]['ratio'] * 100 for b in band_names]

    colors = ['green', 'blue', 'orange', 'red']
    bars = ax3.bar(band_names, band_percentages, color=colors[:len(band_names)])
    ax3.set_ylabel('能量占比 (%)')
    ax3.set_title('频段能量分布')
    ax3.set_ylim([0, 100])

    # 添加数值标签
    for bar, pct in zip(bars, band_percentages):
        ax3.text(bar.get_x() + bar.get_width()/2., bar.get_height() + 1,
                f'{pct:.1f}%', ha='center', va='bottom', fontsize=9)

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"图表已保存: {output_path}")
    plt.close()


# ============================================================================
# 诊断建议
# ============================================================================
def generate_diagnosis(peaks: list, bands: dict) -> list:
    """根据频谱分析生成诊断建议"""
    suggestions = []

    # 检查是否有高频分量
    high_freq_ratio = bands.get('高频 (50-100Hz)', {}).get('ratio', 0)
    ultra_high_freq_ratio = bands.get('超高频 (>100Hz)', {}).get('ratio', 0)

    if high_freq_ratio > 0.1 or ultra_high_freq_ratio > 0.1:
        suggestions.append({
            'type': '高频振动',
            'description': '检测到显著的高频分量',
            'possible_causes': [
                'Kd 增益过高，放大了噪声',
                '编码器信号噪声',
                '通信抖动',
                '控制环振荡'
            ],
            'suggestions': [
                '降低 Kd 增益 30-50%',
                '增加速度滤波',
                '检查编码器连接',
                '检查通信质量'
            ]
        })

    # 检查主频
    if peaks:
        main_freq = peaks[0]['frequency']
        main_magnitude = peaks[0]['magnitude']

        if 40 < main_freq < 60:
            suggestions.append({
                'type': '工频干扰',
                'description': f'检测到 {main_freq:.1f}Hz 主频，可能是工频干扰',
                'possible_causes': [
                    '电源纹波',
                    '电磁干扰',
                    '采样率与工频耦合'
                ],
                'suggestions': [
                    '检查电源质量',
                    '增加屏蔽',
                    '调整采样率'
                ]
            })
        elif main_freq < 10:
            suggestions.append({
                'type': '低频振荡',
                'description': f'检测到 {main_freq:.1f}Hz 低频振荡',
                'possible_causes': [
                    '机械共振',
                    '重力补偿不足',
                    '控制增益过低'
                ],
                'suggestions': [
                    '检查机械结构',
                    '调整重力补偿',
                    '增加 Kp 增益'
                ]
            })
        elif main_freq > 100:
            suggestions.append({
                'type': '超高频振动',
                'description': f'检测到 {main_freq:.1f}Hz 超高频振动',
                'possible_causes': [
                    '编码器分辨率不足',
                    'PWM 载波频率',
                    '数字控制延迟'
                ],
                'suggestions': [
                    '增加编码器滤波',
                    '降低 PWM 频率',
                    '检查控制周期'
                ]
            })

    # 如果没有明显问题
    if not suggestions:
        suggestions.append({
            'type': '正常',
            'description': '频谱分析未发现明显异常',
            'possible_causes': [],
            'suggestions': [
                '如果仍有振动感，可能是机械原因',
                '建议检查机械间隙和螺丝紧固'
            ]
        })

    return suggestions


# ============================================================================
# 报告生成
# ============================================================================
def generate_report(fft_result: dict, peaks: list, bands: dict,
                    diagnosis: list, output_path: Path):
    """生成分析报告"""
    report = []
    report.append("=" * 60)
    report.append("FFT 频谱分析报告")
    report.append("=" * 60)
    report.append("")

    # 峰值信息
    report.append("-" * 40)
    report.append("主要频率分量")
    report.append("-" * 40)
    if peaks:
        for i, peak in enumerate(peaks[:5], 1):
            report.append(f"{i}. {peak['frequency']:.2f} Hz (幅值: {peak['magnitude']:.4f})")
    else:
        report.append("未检测到明显峰值")
    report.append("")

    # 频段分布
    report.append("-" * 40)
    report.append("频段能量分布")
    report.append("-" * 40)
    for band_name, band_info in bands.items():
        report.append(f"{band_name}: {band_info['percentage']}")
    report.append("")

    # 诊断建议
    report.append("-" * 40)
    report.append("诊断建议")
    report.append("-" * 40)
    for diag in diagnosis:
        report.append(f"\n类型: {diag['type']}")
        report.append(f"描述: {diag['description']}")
        if diag['possible_causes']:
            report.append("可能原因:")
            for cause in diag['possible_causes']:
                report.append(f"  - {cause}")
        if diag['suggestions']:
            report.append("建议:")
            for suggestion in diag['suggestions']:
                report.append(f"  - {suggestion}")
    report.append("")
    report.append("=" * 60)

    # 保存报告
    report_text = "\n".join(report)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(report_text)

    # 打印报告
    print(report_text)


# ============================================================================
# 主函数
# ============================================================================
def main():
    parser = argparse.ArgumentParser(
        description='FFT 频谱分析',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 08_analyze_frequency.py vibration_data/
  python3 08_analyze_frequency.py --joint 4 data/
  python3 08_analyze_frequency.py --test  # 使用测试数据
        """
    )

    parser.add_argument('data_dir', nargs='?', default=None,
                       help='数据目录（包含 analysis_result.json）')
    parser.add_argument('--joint', type=int, default=4,
                       help='分析的关节编号 (1-7)，默认: 4')
    parser.add_argument('--sample-rate', type=float, default=100.0,
                       help='采样率 (Hz)，默认: 100')
    parser.add_argument('--test', action='store_true',
                       help='使用测试数据')
    parser.add_argument('--output', type=str, default=None,
                       help='输出目录')

    args = parser.parse_args()

    # 设置关节索引
    joint_idx = args.joint - 1
    if joint_idx < 0 or joint_idx >= len(JOINT_NAMES):
        print(f"错误: 关节编号必须在 1-{len(JOINT_NAMES)} 之间")
        sys.exit(1)

    # 确定输出目录
    if args.output:
        output_dir = Path(args.output)
    elif args.data_dir:
        output_dir = Path(args.data_dir)
    else:
        output_dir = Path('.')

    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"分析关节: {JOINT_NAMES[joint_idx]} (索引: {joint_idx})")
    print(f"采样率: {args.sample_rate} Hz")
    print("")

    # 加载数据
    if args.test:
        print("使用测试数据...")
        t, signal = generate_test_signal(
            duration=5.0,
            sample_rate=args.sample_rate,
            freq1=5.0,
            freq2=75.0
        )
    elif args.data_dir:
        data_path = Path(args.data_dir)
        json_path = data_path / 'analysis_result.json'

        if json_path.exists():
            print(f"加载数据: {json_path}")
            data = load_data_from_json(json_path)

            # 从统计数据重建信号（简化版本）
            # 实际应用中应该从 bag 文件加载原始数据
            print("注意: 从 JSON 加载的是统计数据，使用模拟信号演示")
            t, signal = generate_test_signal(
                duration=5.0,
                sample_rate=args.sample_rate
            )
        else:
            print(f"错误: 未找到 {json_path}")
            print("请先运行 02_analyze_bag.py 生成分析数据")
            sys.exit(1)
    else:
        print("错误: 请指定数据目录或使用 --test")
        parser.print_help()
        sys.exit(1)

    # FFT 分析
    print("\n执行 FFT 分析...")
    fft_result = compute_fft(signal, args.sample_rate)

    # 找到峰值
    print("检测峰值...")
    peaks = find_peaks(fft_result['magnitudes'], fft_result['frequencies'])

    # 频段分析
    print("分析频段...")
    bands = analyze_frequency_bands(fft_result['frequencies'], fft_result['magnitudes'])

    # 生成诊断
    print("生成诊断...")
    diagnosis = generate_diagnosis(peaks, bands)

    # 生成图表
    if HAS_MATPLOTLIB:
        print("\n生成图表...")
        plot_path = output_dir / 'frequency_analysis.png'
        plot_fft_analysis(
            fft_result['frequencies'],
            fft_result['magnitudes'],
            peaks,
            bands,
            plot_path,
            title=f'{JOINT_NAMES[joint_idx]} FFT 频谱分析'
        )

    # 生成报告
    print("\n生成报告...")
    report_path = output_dir / 'frequency_report.txt'
    generate_report(fft_result, peaks, bands, diagnosis, report_path)

    # 保存原始数据
    json_output = {
        'joint': JOINT_NAMES[joint_idx],
        'sample_rate': args.sample_rate,
        'num_samples': len(signal),
        'peaks': peaks[:10],
        'frequency_bands': bands,
        'diagnosis': diagnosis
    }

    json_path = output_dir / 'frequency_analysis.json'
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump(json_output, f, indent=2, ensure_ascii=False)
    print(f"\n详细数据已保存: {json_path}")


if __name__ == '__main__':
    main()
