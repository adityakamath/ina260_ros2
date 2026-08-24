"""Tests for INA260Sensor's register decoding, against the TI INA260 datasheet layout.

Stubs smbus2.SMBus so these run without a real I2C bus/device.
"""

import struct

import pytest
from ina260_battery_monitor import ina260_sensor as sensor_module
from ina260_battery_monitor.ina260_sensor import INA260Error, INA260Sensor


class _FakeBus:

    def __init__(self, registers, raise_oserror_on=None):
        self.registers = registers
        self.raise_oserror_on = raise_oserror_on or set()
        self.closed = False

    def read_i2c_block_data(self, address, register, length):
        if register in self.raise_oserror_on:
            raise OSError('simulated I2C bus error')
        return list(self.registers[register])

    def close(self):
        self.closed = True


def _make_sensor(monkeypatch, registers, raise_oserror_on=None):
    fake_bus = _FakeBus(registers, raise_oserror_on)
    monkeypatch.setattr(sensor_module, 'SMBus', lambda bus_number: fake_bus)
    return INA260Sensor(bus_number=1, address=0x40)


class TestCheckIdentity:

    def test_correct_manufacturer_id_passes(self, monkeypatch):
        sensor = _make_sensor(monkeypatch, {0xFE: struct.pack('>H', 0x5449)})
        sensor.check_identity()  # must not raise

    def test_wrong_manufacturer_id_raises(self, monkeypatch):
        sensor = _make_sensor(monkeypatch, {0xFE: struct.pack('>H', 0x1234)})
        with pytest.raises(INA260Error):
            sensor.check_identity()


class TestRead:

    def test_decodes_voltage_current_power(self, monkeypatch):
        # 14.8V bus voltage, +0.5A current, 7.4W power at the datasheet's documented LSBs.
        registers = {
            0x02: struct.pack('>H', round(14.8 / 1.25e-3)),
            0x01: struct.pack('>h', round(0.5 / 1.25e-3)),
            0x03: struct.pack('>H', round(7.4 / 10e-3)),
        }
        sensor = _make_sensor(monkeypatch, registers)
        voltage, current, power = sensor.read()
        assert voltage == pytest.approx(14.8, abs=1e-3)
        assert current == pytest.approx(0.5, abs=1e-3)
        assert power == pytest.approx(7.4, abs=1e-2)

    def test_decodes_negative_current(self, monkeypatch):
        registers = {
            0x02: struct.pack('>H', round(14.8 / 1.25e-3)),
            0x01: struct.pack('>h', round(-1.2 / 1.25e-3)),
            0x03: struct.pack('>H', round(17.76 / 10e-3)),
        }
        sensor = _make_sensor(monkeypatch, registers)
        _voltage, current, _power = sensor.read()
        assert current == pytest.approx(-1.2, abs=1e-3)

    def test_i2c_failure_raises_ina260_error(self, monkeypatch):
        sensor = _make_sensor(
            monkeypatch,
            {0x02: struct.pack('>H', 0), 0x01: struct.pack('>h', 0), 0x03: struct.pack('>H', 0)},
            raise_oserror_on={0x02},
        )
        with pytest.raises(INA260Error):
            sensor.read()
