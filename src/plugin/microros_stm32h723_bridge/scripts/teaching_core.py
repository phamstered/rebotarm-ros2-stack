"""ROS-independent admission rules for the bounded teaching gripper action."""
import math

JOINT = 'gripper_finger_left_joint'
DURATION = 5.0


def target_position(value):
    value = float(value)
    if not math.isfinite(value) or not 0.0 <= value <= 0.05:
        raise ValueError('夹爪目标必须为标定范围内的有限值（0–0.05 m）')
    return value


def controller_error(controllers):
    active = {c.name: c for c in controllers if c.state == 'active'}
    if 'gravity_compensation_controller' not in active:
        return '重力补偿未激活'
    if 'gripper_trajectory_controller' not in active:
        return '夹爪轨迹控制器未激活'
    expected = {f'{JOINT}/position', f'{JOINT}/velocity'}
    if set(active['gripper_trajectory_controller'].claimed_interfaces) != expected:
        return '夹爪轨迹接口不匹配'
    for name, controller in active.items():
        if name in {'arm_teleop_controller', 'arm_trajectory_controller', 'vla_position_controller',
                    'joint4_calibration_controller', 'gripper_teleop_controller'}:
            return f'示教模式存在冲突控制器：{name}'
        if any(interface == f'joint{i}/{kind}' for interface in controller.claimed_interfaces
               for i in range(1, 7) for kind in ('position', 'velocity')):
            return '六轴位置或速度接口被占用'
    return None


def fresh(received, stamp_ns, now, ros_now_ns):
    return (now - received <= 0.5 and stamp_ns > 0 and
            -50_000_000 <= ros_now_ns - stamp_ns < 500_000_000)
