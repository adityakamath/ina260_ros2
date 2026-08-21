"""Publishes sensor_msgs/BatteryState from an INA260 current/voltage/power sensor.

Polls the INA260 on a timer, feeds each reading into a CoulombCounter for
charge/percentage estimation, and derives BatteryState's status/health fields from
the current sign and configured voltage cutoffs. See coulomb_counter.py for the
charge-estimation strategy and README.md for parameter documentation.

If no INA260 answers at startup (wrong I2C address, not fitted at all), the node logs
one warning and exits cleanly instead of raising - for a fleet where some robots don't
have this sensor, that's a much friendlier failure mode than a startup crash/traceback
if whoever launches it forgets to set battery_monitor:=false.
"""

import os
import time

import rclpy
from ina260_ros2.coulomb_counter import CoulombCounter
from ina260_ros2.ina260_sensor import INA260Error, INA260Sensor
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

_TECHNOLOGY_BY_NAME = {
    'LIPO': BatteryState.POWER_SUPPLY_TECHNOLOGY_LIPO,
    'LION': BatteryState.POWER_SUPPLY_TECHNOLOGY_LION,
}


class BatteryMonitorNode(Node):

    def __init__(self, **kwargs):
        super().__init__('battery_monitor_node', **kwargs)

        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('i2c_address', 0x40)
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('current_polarity_inverted', False)

        self.declare_parameter('chemistry', 'LION')
        self.declare_parameter('cell_count', 3)
        self.declare_parameter('design_capacity_ah', 2.55)
        self.declare_parameter('voltage_min_per_cell', 3.0)
        self.declare_parameter('voltage_max_per_cell', 4.2)

        self.declare_parameter('idle_current_threshold_a', 0.05)
        self.declare_parameter('rest_current_threshold_a', 0.05)
        self.declare_parameter('rest_settle_s', 60.0)
        self.declare_parameter('state_file_path', '~/.ros/ina260_ros2/state.yaml')
        self.declare_parameter('state_save_interval_s', 30.0)
        self.declare_parameter('state_max_stale_s', 3600.0)

        self.declare_parameter('serial_number', '')
        self.declare_parameter('location', '')

        chemistry = self.get_parameter('chemistry').value
        if chemistry not in _TECHNOLOGY_BY_NAME:
            raise ValueError(
                f'chemistry {chemistry!r} not supported, must be one of '
                f'{sorted(_TECHNOLOGY_BY_NAME)}'
            )

        self._cell_count = self.get_parameter('cell_count').value
        self._design_capacity_ah = self.get_parameter('design_capacity_ah').value
        self._voltage_min_per_cell = self.get_parameter('voltage_min_per_cell').value
        self._voltage_max_per_cell = self.get_parameter('voltage_max_per_cell').value
        self._idle_current_threshold_a = self.get_parameter('idle_current_threshold_a').value
        self._current_polarity_inverted = self.get_parameter('current_polarity_inverted').value
        self._technology = _TECHNOLOGY_BY_NAME[chemistry]
        self._frame_id = self.get_parameter('frame_id').value
        self._serial_number = self.get_parameter('serial_number').value
        self._location = self.get_parameter('location').value
        self._state_save_interval_s = self.get_parameter('state_save_interval_s').value

        state_path = os.path.expanduser(self.get_parameter('state_file_path').value)
        self._coulomb_counter = CoulombCounter(
            design_capacity_ah=self._design_capacity_ah,
            cell_count=self._cell_count,
            chemistry=chemistry,
            rest_current_threshold_a=self.get_parameter('rest_current_threshold_a').value,
            rest_settle_s=self.get_parameter('rest_settle_s').value,
            state_path=state_path,
            state_max_stale_s=self.get_parameter('state_max_stale_s').value,
        )

        self._sensor = None
        self._hardware_missing = False
        i2c_bus = self.get_parameter('i2c_bus').value
        i2c_address = self.get_parameter('i2c_address').value
        try:
            sensor = INA260Sensor(bus_number=i2c_bus, address=i2c_address)
            sensor.check_identity()
        except INA260Error as exc:
            self.get_logger().warning(
                f'No INA260 detected on I2C bus {i2c_bus} address 0x{i2c_address:02X} '
                f'({exc}). Shutting down without publishing battery_state - set '
                f'battery_monitor:=false if this robot has no INA260 fitted.'
            )
            self._hardware_missing = True
            return
        self._sensor = sensor

        self._publisher = self.create_publisher(BatteryState, 'battery_state', 10)

        self._last_read_monotonic = None
        self._last_save_monotonic = time.monotonic()

        publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self._timer = self.create_timer(1.0 / publish_rate_hz, self._on_timer)

        self.get_logger().info(
            f'ina260_ros2 started: bus={self.get_parameter("i2c_bus").value} '
            f'address=0x{self.get_parameter("i2c_address").value:02X} '
            f'chemistry={chemistry} cells={self._cell_count} '
            f'capacity={self._design_capacity_ah}Ah'
        )

    @property
    def hardware_missing(self) -> bool:
        return self._hardware_missing

    def _on_timer(self):
        try:
            voltage, current, _power = self._sensor.read()
        except INA260Error as exc:
            self.get_logger().error(f'INA260 read failed, skipping this cycle: {exc}')
            return

        if self._current_polarity_inverted:
            current = -current

        now = time.monotonic()
        dt_s = 0.0 if self._last_read_monotonic is None else now - self._last_read_monotonic
        self._last_read_monotonic = now

        self._coulomb_counter.update(voltage, current, dt_s)

        if now - self._last_save_monotonic >= self._state_save_interval_s:
            self._coulomb_counter.save_state()
            self._last_save_monotonic = now

        self._publisher.publish(self._to_battery_state(voltage, current))

    def _to_battery_state(self, voltage: float, current: float) -> BatteryState:
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.voltage = voltage
        msg.temperature = float('nan')
        msg.current = current
        msg.charge = self._coulomb_counter.charge_ah
        msg.capacity = self._design_capacity_ah
        msg.design_capacity = self._design_capacity_ah
        msg.percentage = self._coulomb_counter.percentage
        msg.power_supply_status = self._status(current)
        msg.power_supply_health = self._health(voltage)
        msg.power_supply_technology = self._technology
        msg.present = True
        msg.cell_voltage = []
        msg.cell_temperature = []
        msg.location = self._location
        msg.serial_number = self._serial_number
        return msg

    def _status(self, current: float) -> int:
        if abs(current) <= self._idle_current_threshold_a:
            if self._coulomb_counter.percentage >= 0.99:
                return BatteryState.POWER_SUPPLY_STATUS_FULL
            return BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
        if current > 0:
            return BatteryState.POWER_SUPPLY_STATUS_CHARGING
        return BatteryState.POWER_SUPPLY_STATUS_DISCHARGING

    def _health(self, voltage: float) -> int:
        cell_voltage = voltage / self._cell_count
        if cell_voltage < self._voltage_min_per_cell:
            return BatteryState.POWER_SUPPLY_HEALTH_DEAD
        if cell_voltage > self._voltage_max_per_cell:
            return BatteryState.POWER_SUPPLY_HEALTH_OVERVOLTAGE
        return BatteryState.POWER_SUPPLY_HEALTH_GOOD

    def destroy_node(self):
        if self._sensor is not None:
            self._coulomb_counter.save_state()
            self._sensor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    if node.hardware_missing:
        # No error, no traceback - the warning already logged in __init__ says why.
        # A clean exit here also means launch's respawn (if enabled) won't keep retrying
        # a condition that polling can't fix: the hardware is either there or it isn't.
        node.destroy_node()
        rclpy.try_shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
