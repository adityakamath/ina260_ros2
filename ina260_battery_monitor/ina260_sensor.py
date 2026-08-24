"""Register-level driver for the Adafruit INA260 current/power sensor (TI INA260 IC).

Talks directly to the chip over Linux i2c-dev via smbus2 rather than pulling in the
Adafruit CircuitPython/Blinka stack - the INA260 register map is small and stable, so a
thin driver here avoids Blinka's platform autodetection as a dependency and failure mode.
Register layout and LSB sizes are from the TI INA260 datasheet (SBOS866).
"""

import struct

from smbus2 import SMBus

_REG_CONFIG = 0x00
_REG_CURRENT = 0x01
_REG_VOLTAGE = 0x02
_REG_POWER = 0x03
_REG_MANUFACTURER_ID = 0xFE
_REG_DIE_ID = 0xFF

_MANUFACTURER_ID = 0x5449  # "TI"

_CURRENT_LSB = 1.25e-3  # A/bit, signed
_VOLTAGE_LSB = 1.25e-3  # V/bit, unsigned
_POWER_LSB = 10e-3  # W/bit, unsigned

_CONFIG_RESET = 0x8000


class INA260Error(RuntimeError):
    """Raised when the INA260 can't be read or doesn't identify itself correctly."""


class INA260Sensor:
    """Reads bus voltage, current and power from an INA260 over I2C."""

    def __init__(self, bus_number: int, address: int):
        self._address = address
        self._bus = SMBus(bus_number)

    def check_identity(self) -> None:
        manufacturer_id = self._read_register(_REG_MANUFACTURER_ID)
        if manufacturer_id != _MANUFACTURER_ID:
            raise INA260Error(
                f'unexpected manufacturer ID 0x{manufacturer_id:04X} at address '
                f'0x{self._address:02X} (expected 0x{_MANUFACTURER_ID:04X} - wrong device '
                f'or wrong I2C address?)'
            )

    def read(self) -> tuple[float, float, float]:
        """Return (voltage_v, current_a, power_w). current is positive as the INA260 sees it."""
        voltage = self._read_register(_REG_VOLTAGE) * _VOLTAGE_LSB
        current = self._read_signed_register(_REG_CURRENT) * _CURRENT_LSB
        power = self._read_register(_REG_POWER) * _POWER_LSB
        return voltage, current, power

    def close(self) -> None:
        self._bus.close()

    def _read_register(self, register: int) -> int:
        try:
            data = self._bus.read_i2c_block_data(self._address, register, 2)
        except OSError as exc:
            raise INA260Error(f'I2C read of register 0x{register:02X} failed: {exc}') from exc
        return struct.unpack('>H', bytes(data))[0]

    def _read_signed_register(self, register: int) -> int:
        try:
            data = self._bus.read_i2c_block_data(self._address, register, 2)
        except OSError as exc:
            raise INA260Error(f'I2C read of register 0x{register:02X} failed: {exc}') from exc
        return struct.unpack('>h', bytes(data))[0]
