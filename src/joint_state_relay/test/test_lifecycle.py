"""Lifecycle tests need no ROS installation or serial hardware."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import Mock

import pytest

MODULE = Path(__file__).parents[1] / 'joint_state_relay' / 'lifecycle.py'
spec = importlib.util.spec_from_file_location('relay_lifecycle', MODULE)
lifecycle = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lifecycle)


class ExternalShutdownException(Exception):
    pass


class RCLError(Exception):
    pass


@pytest.fixture
def ros(monkeypatch):
    module = ModuleType('rclpy')
    module.init = Mock()
    module.spin = Mock()
    module.try_shutdown = Mock()
    module.ok = Mock(return_value=True)
    binding = ModuleType("rclpy._rclpy_pybind11")
    binding.RCLError = RCLError
    monkeypatch.setitem(sys.modules, "rclpy._rclpy_pybind11", binding)
    executors = ModuleType('rclpy.executors')
    executors.ExternalShutdownException = ExternalShutdownException
    monkeypatch.setitem(sys.modules, 'rclpy', module)
    monkeypatch.setitem(sys.modules, 'rclpy.executors', executors)
    return module


@pytest.mark.parametrize('error', [None, KeyboardInterrupt(), ExternalShutdownException()])
def test_expected_shutdown_cleans_node_without_attempting_rosout(ros, error):
    node = SimpleNamespace(destroy_node=Mock(), get_logger=Mock())
    close = Mock()
    ros.spin.side_effect = error
    lifecycle.spin_node(lambda: node, args=['--ros-args', '-r', '__ns:=/leader'], before_destroy=close)
    ros.init.assert_called_once_with(args=['--ros-args', '-r', '__ns:=/leader'])
    close.assert_called_once_with(node)
    node.destroy_node.assert_called_once()
    node.get_logger.assert_not_called()
    ros.try_shutdown.assert_called_once()


def test_runtime_errors_are_not_hidden_and_still_cleanup(ros):
    node = SimpleNamespace(destroy_node=Mock())
    ros.spin.side_effect = RuntimeError('serial failure')
    with pytest.raises(RuntimeError, match='serial failure'):
        lifecycle.spin_node(lambda: node)
    node.destroy_node.assert_called_once()
    ros.try_shutdown.assert_called_once()


def test_constructor_failure_is_nonzero_and_closes_context(ros):
    with pytest.raises(RuntimeError, match='initialization failed'):
        lifecycle.spin_node(Mock(side_effect=RuntimeError('initialization failed')))
    ros.spin.assert_not_called()
    ros.try_shutdown.assert_called_once()


def test_vendor_close_failure_still_destroys_node_and_context(ros):
    node = SimpleNamespace(destroy_node=Mock())
    with pytest.raises(RuntimeError, match='close failed'):
        lifecycle.spin_node(lambda: node, before_destroy=Mock(side_effect=RuntimeError('close failed')))
    node.destroy_node.assert_called_once()
    ros.try_shutdown.assert_called_once()


@pytest.mark.parametrize('context_alive', [True, False])
def test_native_wait_set_failure_is_expected_only_after_context_shutdown(ros, context_alive):
    node = SimpleNamespace(destroy_node=Mock())
    ros.ok.return_value = context_alive
    ros.spin.side_effect = RCLError('failed to initialize wait set: context invalid')
    if context_alive:
        with pytest.raises(RCLError):
            lifecycle.spin_node(lambda: node)
    else:
        lifecycle.spin_node(lambda: node)
    node.destroy_node.assert_called_once()
    ros.try_shutdown.assert_called_once()


def test_non_ros_errors_are_not_suppressed_even_after_context_shutdown(ros):
    node = SimpleNamespace(destroy_node=Mock())
    ros.ok.return_value = False
    ros.spin.side_effect = RuntimeError('real driver error')
    with pytest.raises(RuntimeError, match='real driver error'):
        lifecycle.spin_node(lambda: node)
    node.destroy_node.assert_called_once()
    ros.try_shutdown.assert_called_once()
