import importlib.util
from pathlib import Path

spec = importlib.util.spec_from_file_location('feedback_clock', Path(__file__).parents[1] / 'scripts/feedback_clock.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
FeedbackClock = module.FeedbackClock


def valid(clock, now=10000000000, steady=0):
    return clock.observe(now-1000000, now, steady)


def diag(state=1, epoch=1):
    data = [2] + [0]*23
    data[16], data[22] = state, epoch
    return data


def test_startup_stability_and_timestamp_boundaries():
    c = FeedbackClock()
    assert not valid(c)
    assert not valid(c, 10199000000, 199000000)
    assert valid(c, 10200000000, 200000000)
    for stamp in (0, 12000000000, 8000000000):
        assert not c.observe(stamp, 10300000000, 300000000)
        assert c.state == 'unsynchronized'
    assert not valid(c, 10400000000, 400000000)
    assert valid(c, 10600000000, 600000000)


def test_future_then_large_correction_does_not_freeze_on_future_stamp():
    c = FeedbackClock()
    assert not c.observe(25000000000, 10000000000, 0)
    assert not valid(c, 10010000000, 10000000)
    assert valid(c, 10210000000, 210000000)
    assert c.last_stamp == 10209000000
    assert not c.observe(10209000000, 10220000000, 220000000)  # duplicate
    assert not c.observe(10100000000, 10230000000, 230000000)  # regression
    assert not valid(c, 10240000000, 240000000)
    assert valid(c, 10440000000, 440000000)


def test_firmware_versions_epoch_and_expiry():
    c = FeedbackClock()
    c.firmware([1]+[0]*15, 0)
    assert not valid(c)
    assert valid(c, 10200000000, 200000000)
    c.firmware(diag(), 200000000)
    c.firmware(diag(epoch=2), 210000000)
    assert not valid(c, 10220000000, 220000000)
    assert valid(c, 10420000000, 420000000)
    c.firmware(diag(state=2, epoch=2), 430000000)
    assert not valid(c, 10440000000, 440000000)
    c.firmware(diag(epoch=2), 450000000)
    assert not valid(c, 10460000000, 460000000)
    assert valid(c, 10660000000, 660000000)
    assert not valid(c, 13450000000, 3450000000)
    assert c.state == 'unsynchronized'


def test_silence_and_host_clock_jump_restart_recovery():
    c = FeedbackClock()
    valid(c)
    assert valid(c, 10200000000, 200000000)
    c.refresh(1200000000)
    assert c.state == 'stale'
    assert not valid(c, 11210000000, 1210000000)
    assert valid(c, 11410000000, 1410000000)
    assert not valid(c, 21420000000, 1420000000)
    assert valid(c, 21620000000, 1620000000)
