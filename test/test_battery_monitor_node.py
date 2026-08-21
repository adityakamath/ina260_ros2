"""Node-level unit tests for BatteryMonitorNode's status/health/message-mapping logic.

Stubs INA260Sensor so these run without real I2C hardware - the sensor driver itself
(register decoding) is exercised separately against the datasheet's documented layout
in test_ina260_sensor.py.
"""

import pytest
import rclpy
from ina260_ros2 import battery_monitor_node as node_module
from sensor_msgs.msg import BatteryState


class _FakeSensor:
    """Records constructor args and returns a fixed (voltage, current, power) reading."""

    last_instance = None

    def __init__(self, bus_number, address):
        self.bus_number = bus_number
        self.address = address
        self.reading = (14.8, 0.0, 0.0)
        self.closed = False
        _FakeSensor.last_instance = self

    def check_identity(self):
        pass

    def read(self):
        return self.reading

    def close(self):
        self.closed = True


class _MissingSensor:
    """Simulates no INA260 answering on the bus - check_identity() always fails."""

    def __init__(self, bus_number, address):
        pass

    def check_identity(self):
        raise node_module.INA260Error('simulated: no device at that address')

    def close(self):
        raise AssertionError('close() should never be called - the sensor never opened')


@pytest.fixture(autouse=True)
def fake_sensor(monkeypatch):
    monkeypatch.setattr(node_module, 'INA260Sensor', _FakeSensor)


@pytest.fixture
def node(tmp_path):
    n = node_module.BatteryMonitorNode(
        parameter_overrides=[
            rclpy.parameter.Parameter('state_file_path', value=str(tmp_path / 'state.yaml')),
        ]
    )
    yield n
    n.destroy_node()


class TestStatus:

    def test_idle_current_is_not_charging(self, node):
        assert node._status(0.0) == BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING

    def test_positive_current_is_charging(self, node):
        assert node._status(1.0) == BatteryState.POWER_SUPPLY_STATUS_CHARGING

    def test_negative_current_is_discharging(self, node):
        assert node._status(-1.0) == BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

    def test_idle_at_full_charge_is_full(self, node):
        node._coulomb_counter.seed(voltage=4.20 * node._cell_count)
        assert node._status(0.0) == BatteryState.POWER_SUPPLY_STATUS_FULL


class TestHealth:

    def test_normal_voltage_is_good(self, node):
        assert node._health(3.80 * node._cell_count) == BatteryState.POWER_SUPPLY_HEALTH_GOOD

    def test_below_cutoff_is_dead(self, node):
        assert node._health(2.5 * node._cell_count) == BatteryState.POWER_SUPPLY_HEALTH_DEAD

    def test_above_cutoff_is_overvoltage(self, node):
        assert (
            node._health(4.5 * node._cell_count)
            == BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        )


class TestOnTimer:

    def test_publishes_battery_state_with_expected_fields(self, node, monkeypatch):
        published = []
        monkeypatch.setattr(node._publisher, 'publish', published.append)
        _FakeSensor.last_instance.reading = (14.8, 0.5, 7.4)

        node._on_timer()

        assert len(published) == 1
        msg = published[0]
        assert msg.voltage == pytest.approx(14.8)
        assert msg.current == pytest.approx(0.5)
        assert msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
        assert msg.present is True
        assert msg.design_capacity == node._design_capacity_ah

    def test_current_polarity_inverted_flips_sign(self, tmp_path, monkeypatch):
        monkeypatch.setattr(node_module, 'INA260Sensor', _FakeSensor)
        n = node_module.BatteryMonitorNode(
            parameter_overrides=[
                rclpy.parameter.Parameter('state_file_path', value=str(tmp_path / 'state.yaml')),
                rclpy.parameter.Parameter('current_polarity_inverted', value=True),
            ]
        )
        try:
            published = []
            monkeypatch.setattr(n._publisher, 'publish', published.append)
            _FakeSensor.last_instance.reading = (14.8, 0.5, 7.4)

            n._on_timer()

            assert published[0].current == pytest.approx(-0.5)
        finally:
            n.destroy_node()

    def test_sensor_read_error_skips_publish(self, node, monkeypatch):
        published = []
        monkeypatch.setattr(node._publisher, 'publish', published.append)

        def _raise():
            raise node_module.INA260Error('simulated I2C failure')

        monkeypatch.setattr(_FakeSensor.last_instance, 'read', _raise)

        node._on_timer()

        assert published == []


class TestHardwareMissing:
    """No INA260 answering at startup must warn and shut down cleanly, not crash."""

    def test_sets_hardware_missing_flag(self, tmp_path, monkeypatch):
        monkeypatch.setattr(node_module, 'INA260Sensor', _MissingSensor)
        n = node_module.BatteryMonitorNode(
            parameter_overrides=[
                rclpy.parameter.Parameter('state_file_path', value=str(tmp_path / 'state.yaml')),
            ]
        )
        try:
            assert n.hardware_missing is True
        finally:
            n.destroy_node()

    def test_never_creates_publisher_or_timer(self, tmp_path, monkeypatch):
        monkeypatch.setattr(node_module, 'INA260Sensor', _MissingSensor)
        n = node_module.BatteryMonitorNode(
            parameter_overrides=[
                rclpy.parameter.Parameter('state_file_path', value=str(tmp_path / 'state.yaml')),
            ]
        )
        try:
            assert not hasattr(n, '_publisher')
            assert not hasattr(n, '_timer')
        finally:
            n.destroy_node()

    def test_destroy_node_does_not_touch_sensor_or_state(self, tmp_path, monkeypatch):
        # _MissingSensor.close() raises if called - destroy_node() must not call it, and
        # must not try to save coulomb-counter state that was never seeded from a reading.
        monkeypatch.setattr(node_module, 'INA260Sensor', _MissingSensor)
        n = node_module.BatteryMonitorNode(
            parameter_overrides=[
                rclpy.parameter.Parameter('state_file_path', value=str(tmp_path / 'state.yaml')),
            ]
        )
        n.destroy_node()  # must not raise
        assert not (tmp_path / 'state.yaml').exists()

    def test_healthy_sensor_does_not_set_hardware_missing(self, node):
        assert node.hardware_missing is False
