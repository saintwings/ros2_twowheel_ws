import pytest

from two_wheel_robot.imu_node import SensorClockSync


def test_tracks_minimum_latency_and_resets_on_jump():
    sync = SensorClockSync()
    # sensor at 1 s maps to host 101.005 (5 ms USB latency)
    assert sync.to_host(101.005, 1_000_000) == pytest.approx(101.005)
    # a faster-arriving packet tightens the offset
    assert sync.to_host(101.011, 1_010_000) == pytest.approx(101.011)
    # a late packet keeps its sensor-clock spacing instead of host jitter
    assert sync.to_host(101.030, 1_014_000) == pytest.approx(101.015, abs=1e-5)
    # sensor reset (clock jumps back) re-anchors
    assert sync.to_host(102.0, 0) == pytest.approx(102.0)
