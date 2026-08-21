"""Tests for battery_events_node: percentage threshold hysteresis.

_ThresholdWatcher is tested standalone (pure Python, no ROS). Node-level tests construct
a real BatteryEventsNode but monkeypatch each _StateService's set() method to record calls
instead of actually calling its SetBool service - no live service discovery needed to
verify the edge-detection logic that decides *when* set() should fire.
"""

import pytest
import rclpy
from ina260_ros2.battery_events_node import BatteryEventsNode, _ThresholdWatcher
from sensor_msgs.msg import BatteryState


class TestThresholdWatcherFalling:

    def _watcher(self, threshold=0.2, hysteresis=0.05):
        triggered = []
        reset = []
        watcher = _ThresholdWatcher(
            threshold, hysteresis, 'falling',
            on_trigger=lambda: triggered.append(None),
            on_reset=lambda: reset.append(None),
        )
        return watcher, triggered, reset

    def test_fires_once_crossing_at_or_below_threshold(self):
        watcher, triggered, reset = self._watcher()
        watcher.update(0.5)
        watcher.update(0.25)
        assert triggered == []
        watcher.update(0.20)
        assert len(triggered) == 1
        watcher.update(0.15)
        assert len(triggered) == 1, 'must not re-fire while still below threshold'

    def test_resets_only_above_threshold_plus_hysteresis(self):
        watcher, triggered, reset = self._watcher(threshold=0.2, hysteresis=0.05)
        watcher.update(0.10)
        assert len(triggered) == 1
        watcher.update(0.22)  # above threshold but still inside the hysteresis band
        assert reset == [], 'must not reset inside the hysteresis band'
        watcher.update(0.26)  # past threshold + hysteresis
        assert len(reset) == 1

    def test_refires_after_a_genuine_second_fall(self):
        watcher, triggered, reset = self._watcher(threshold=0.2, hysteresis=0.05)
        watcher.update(0.10)
        watcher.update(0.30)  # recover and re-arm
        assert len(triggered) == 1
        watcher.update(0.15)
        assert len(triggered) == 2


class TestThresholdWatcherRising:

    def _watcher(self, threshold=0.98, hysteresis=0.02):
        triggered = []
        reset = []
        watcher = _ThresholdWatcher(
            threshold, hysteresis, 'rising',
            on_trigger=lambda: triggered.append(None),
            on_reset=lambda: reset.append(None),
        )
        return watcher, triggered, reset

    def test_fires_once_crossing_at_or_above_threshold(self):
        watcher, triggered, reset = self._watcher()
        watcher.update(0.90)
        assert triggered == []
        watcher.update(0.98)
        assert len(triggered) == 1
        watcher.update(1.0)
        assert len(triggered) == 1, 'must not re-fire while still at/above threshold'

    def test_resets_only_below_threshold_minus_hysteresis(self):
        watcher, triggered, reset = self._watcher(threshold=0.98, hysteresis=0.02)
        watcher.update(1.0)
        assert len(triggered) == 1
        watcher.update(0.97)  # below threshold but still inside the hysteresis band
        assert reset == []
        watcher.update(0.95)  # past threshold - hysteresis
        assert len(reset) == 1


@pytest.fixture
def node():
    n = BatteryEventsNode()
    yield n
    n.destroy_node()


def _battery_state(percentage=float('nan')):
    msg = BatteryState()
    msg.percentage = percentage
    return msg


class TestPercentageThresholds:

    def test_nan_percentage_updates_nothing(self, node, monkeypatch):
        low_calls = []
        monkeypatch.setattr(node._battery_low, 'set', low_calls.append)

        node._on_battery_state(_battery_state(percentage=float('nan')))

        assert low_calls == []

    def test_low_fires_once_on_falling_edge(self, node, monkeypatch):
        low_calls = []
        monkeypatch.setattr(node._battery_low, 'set', low_calls.append)

        node._on_battery_state(_battery_state(percentage=0.5))
        node._on_battery_state(_battery_state(percentage=0.2))
        node._on_battery_state(_battery_state(percentage=0.15))

        assert low_calls == [True]

    def test_critical_and_low_can_both_fire_on_a_big_drop(self, node, monkeypatch):
        low_calls = []
        critical_calls = []
        monkeypatch.setattr(node._battery_low, 'set', low_calls.append)
        monkeypatch.setattr(node._battery_critical, 'set', critical_calls.append)

        node._on_battery_state(_battery_state(percentage=0.5))
        node._on_battery_state(_battery_state(percentage=0.05))  # past both thresholds at once

        assert low_calls == [True]
        assert critical_calls == [True]

    def test_full_fires_once_on_rising_edge(self, node, monkeypatch):
        full_calls = []
        monkeypatch.setattr(node._battery_full, 'set', full_calls.append)

        node._on_battery_state(_battery_state(percentage=0.5))
        node._on_battery_state(_battery_state(percentage=0.99))
        node._on_battery_state(_battery_state(percentage=1.0))

        assert full_calls == [True]


class TestThresholdValidation:

    def test_rejects_critical_not_below_low(self):
        with pytest.raises(ValueError):
            BatteryEventsNode(
                parameter_overrides=[
                    rclpy.parameter.Parameter('critical_battery_threshold', value=0.3),
                    rclpy.parameter.Parameter('low_battery_threshold', value=0.2),
                ]
            )

    def test_rejects_low_not_below_full(self):
        with pytest.raises(ValueError):
            BatteryEventsNode(
                parameter_overrides=[
                    rclpy.parameter.Parameter('low_battery_threshold', value=0.99),
                    rclpy.parameter.Parameter('full_battery_threshold', value=0.98),
                ]
            )
