"""Exercise the adapter's real timer path without ROS or serial hardware."""
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).parents[1]))
from joint_state_relay import leader_driver


@pytest.fixture
def driver(monkeypatch):
    messages = ModuleType('sensor_msgs.msg')
    messages.JointState = lambda: SimpleNamespace(header=SimpleNamespace())
    monkeypatch.setitem(sys.modules, 'sensor_msgs.msg', messages)

    class Vendor:
        def __init__(self):
            self.logger = Mock()
            self.clock = Mock()
            self.joint_states_publisher = Mock()
            self.current_angle = [0.] * 7
            self.replies = {i: float(i * 10) for i in range(7)}
            self.manager = SimpleNamespace(
                servos={i: SimpleNamespace(angle_monitor=123.) for i in range(7)},
                send_sync_servo_monitor=Mock(side_effect=self.monitor),
            )
            self.Servo = SimpleNamespace(
                uservo=self.manager, servo_ids=list(range(7)), SRV_NUM=7,
                JOINT_=[f'joint{i + 1}' for i in range(6)] + ['joint7_left'],
                servoangle2jointstate=Mock(side_effect=lambda servo_id, servo_angle: math.radians(servo_angle)),
            )
        def monitor(self, ids):
            for servo_id, angle in self.replies.items():
                self.manager.servos[servo_id].angle_monitor = angle
        def timer_callback(self):
            self.publish_current_angle()
        def get_logger(self):
            return self.logger
        def get_clock(self):
            return self.clock

    return leader_driver.guarded_driver(Vendor)()


@pytest.mark.parametrize('bad', [None, float('nan'), float('inf'), -float('inf'), '30', True])
def test_invalid_angle_drops_entire_frame_and_recovers(driver, bad):
    driver.replies[3] = bad
    driver.timer_callback()
    driver.joint_states_publisher.publish.assert_not_called()
    driver.Servo.servoangle2jointstate.assert_not_called()
    driver.clock.now.assert_not_called()
    assert '[3]' in driver.logger.warning.call_args.args[0]
    driver.replies[3] = 30.
    driver.timer_callback()
    message = driver.joint_states_publisher.publish.call_args.args[0]
    assert message.position == pytest.approx([math.radians(i * 10) for i in range(7)])
    assert message.name == driver.Servo.JOINT_
    assert message.velocity == message.effort == []
    assert message.header.stamp == driver.clock.now().to_msg()


def test_lost_reply_after_success_does_not_republish_cached_pose(driver):
    driver.timer_callback()
    del driver.replies[6]
    driver.timer_callback()
    assert driver.joint_states_publisher.publish.call_count == 1
    assert '[6]' in driver.logger.warning.call_args.args[0]


def test_undiscovered_servo_drops_frame(driver):
    del driver.replies[2]
    del driver.manager.servos[2]
    driver.timer_callback()
    driver.joint_states_publisher.publish.assert_not_called()
    assert '[2]' in driver.logger.warning.call_args.args[0]


@pytest.mark.parametrize('error', [OSError('serial disconnected'), TypeError('SDK bug')])
def test_real_sdk_failures_propagate(driver, error):
    driver.manager.send_sync_servo_monitor.side_effect = error
    with pytest.raises(type(error), match=str(error)):
        driver.timer_callback()


def test_invalid_conversion_remains_a_failure(driver):
    driver.Servo.servoangle2jointstate.side_effect = lambda **kwargs: float('nan')
    with pytest.raises(ValueError, match='non-finite'):
        driver.timer_callback()
    driver.joint_states_publisher.publish.assert_not_called()


def test_entrypoint_uses_guarded_subclass_and_vendor_cleanup(monkeypatch):
    vendor = ModuleType('robo_driver.robo_driver')
    vendor.Arm_contorl = type('Vendor', (), {})
    monkeypatch.setitem(sys.modules, 'robo_driver.robo_driver', vendor)
    spin = Mock()
    monkeypatch.setattr(leader_driver, 'spin_node', spin)
    leader_driver.main(args=['--ros-args'])
    assert issubclass(spin.call_args.args[0], vendor.Arm_contorl)
    assert spin.call_args.kwargs['args'] == ['--ros-args']
    node = Mock()
    spin.call_args.kwargs['before_destroy'](node)
    node.node_close.assert_called_once()
