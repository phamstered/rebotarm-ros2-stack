import signal
import sys
import os
import time
import threading
from pathlib import Path

_project_root = str(Path.home() / 'Project' / 'reBotArm_control_py')
_venv_site = str(Path.home() / 'Project' / 'reBotArm_control_py' / '.venv' / 'lib' / 'python3.10' / 'site-packages')
_venv_pinocchio = os.path.join(_venv_site, 'cmeel.prefix', 'lib', 'python3.10', 'site-packages')

for p in [_project_root, _venv_site, _venv_pinocchio]:
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from reBotArm_control_py.actuator.arm import RobotArm
from reBotArm_control_py.dynamics import compute_generalized_gravity

_JOINT4_GRAVITY_SCALE = 0.85
_ZERO = np.zeros(6)
_RETURN_DURATION = 2.0
_KP_RETURN = 30.0
_KD_RETURN = 3.0
_KP_HOLD = 20.0
_KD_HOLD = 2.0
_ZERO_THRESHOLD = 0.015

_KP_TRACK = 50.0
_KD_TRACK = 3.0
_KP_IDLE = 0.0
_KD_IDLE = 1.0
_CMD_TIMEOUT = 0.1


class _FollowController:
    def __init__(self):
        self._tracking = False
        self._target_buf = [None, None]
        self._write_idx = 0
        self._lock = threading.Lock()
        self._last_cmd_time = time.monotonic()

        self._returning = False
        self._start_pos = None
        self._return_elapsed = 0.0
        self._return_done = False

    def set_target(self, positions: np.ndarray):
        with self._lock:
            self._target_buf[self._write_idx] = positions.copy()
            self._write_idx = 1 - self._write_idx
            self._tracking = True
            self._last_cmd_time = time.monotonic()

    def clear_target(self):
        self._tracking = False

    def start_return(self, current_pos: np.ndarray):
        self._returning = True
        self._start_pos = current_pos.copy()
        self._return_elapsed = 0.0
        self._return_done = False

    @property
    def return_done(self) -> bool:
        return self._return_done

    def __call__(self, arm: RobotArm, dt: float) -> None:
        q = arm.get_positions()
        tau_g = compute_generalized_gravity(q=q)
        tau_g[3] *= _JOINT4_GRAVITY_SCALE

        if self._returning:
            self._return_elapsed += dt
            t = min(self._return_elapsed / _RETURN_DURATION, 1.0)
            alpha = 0.5 * (1.0 - np.cos(np.pi * t))
            target = self._start_pos * (1.0 - alpha)
            kp = np.full(6, _KP_RETURN)
            kd = np.full(6, _KD_RETURN)
            if t >= 1.0:
                self._return_done = True
                kp = np.full(6, _KP_HOLD)
                kd = np.full(6, _KD_HOLD)
        elif self._tracking:
            if time.monotonic() - self._last_cmd_time > _CMD_TIMEOUT:
                self._tracking = False
                target = q
                kp = np.full(6, _KP_IDLE)
                kd = np.full(6, _KD_IDLE)
            else:
                with self._lock:
                    target = self._target_buf[1 - self._write_idx]
                if target is None:
                    target = q
                kp = np.full(6, _KP_TRACK)
                kd = np.full(6, _KD_TRACK)
        else:
            target = q
            kp = np.full(6, _KP_IDLE)
            kd = np.full(6, _KD_IDLE)

        arm.mit(pos=target, vel=_ZERO, kp=kp, kd=kd, tau=tau_g,
                request_feedback=True)


class FollowerDriverNode(Node):
    def __init__(self):
        super().__init__('rebotarm_follower_driver')
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('config_path', '')

        rate = self.get_parameter('rate').value
        config_path = self.get_parameter('config_path').value

        kwargs = {}
        if config_path:
            kwargs['cfg_path'] = config_path
        self.arm = RobotArm(**kwargs)
        self.arm.connect()
        self.arm.enable()
        self.arm.mode_mit(kp=_ZERO, kd=np.full(self.arm.num_joints, 1.0))
        self._ctrl = _FollowController()
        self.arm.start_control_loop(self._ctrl, rate=self.arm._rate)
        self._shutdown_done = False

        self.joint_names = ['joint1', 'joint2', 'joint3',
                            'joint4', 'joint5', 'joint6']

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / rate, self._publish)

        self.create_subscription(
            JointState, '/joint_commands', self._command_cb, 10)

        self.get_logger().info(
            f'Follower driver started at {rate} Hz '
            f'(gravity comp @ {self.arm._rate} Hz)')

    def _command_cb(self, msg: JointState):
        positions = np.zeros(6)
        for i, name in enumerate(msg.name):
            if name in self.joint_names and i < len(msg.position):
                idx = self.joint_names.index(name)
                positions[idx] = msg.position[i]
        self._ctrl.set_target(positions)

    def _publish(self):
        try:
            p, v, t = self.arm.get_positions(), self.arm.get_velocities(), self.arm.get_torques()
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            msg.position = p.tolist()
            msg.velocity = v.tolist()
            msg.effort = t.tolist()
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Publish failed: {e}')

    def _check_zero(self):
        pos = self.arm.get_positions()
        return [(self.joint_names[i], pos[i])
                for i in range(self.arm.num_joints)
                if abs(pos[i]) > _ZERO_THRESHOLD]

    def shutdown(self):
        if self._shutdown_done:
            return
        self._shutdown_done = True
        self._ctrl.clear_target()
        self.get_logger().info('Returning to zero...')
        self._ctrl.start_return(self.arm.get_positions())

        deadline = time.monotonic() + _RETURN_DURATION + 5.0
        while time.monotonic() < deadline:
            time.sleep(0.3)
            if self._ctrl.return_done:
                time.sleep(0.5)
                break

        not_zero = self._check_zero()
        if not not_zero:
            self.get_logger().info('All joints at zero, exiting.')
            self.arm.disconnect()
            sys.exit(0)

        self.get_logger().warn('=== 以下关节未归零，请手动归零 ===')
        for name, pos in not_zero:
            self.get_logger().warn(
                f'  {name}: {pos:+.3f} rad ({np.degrees(pos):+.1f}°)')
        self.get_logger().warn('手动归零后按 Ctrl+C 退出')


def main(args=None):
    rclpy.init(args=args)
    node = FollowerDriverNode()
    _force_exit = [False]

    def _on_sigint(signum, frame):
        if _force_exit[0]:
            node.get_logger().warn('Force exit!')
            node.arm.disconnect()
            sys.exit(1)
        _force_exit[0] = True
        node.get_logger().info('Ctrl+C: returning to zero... (再按 Ctrl+C 强制退出)')
        node.shutdown()

    signal.signal(signal.SIGINT, _on_sigint)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)

    try:
        rclpy.spin(node)
    except Exception:
        pass


if __name__ == '__main__':
    main()
