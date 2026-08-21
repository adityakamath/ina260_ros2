"""Interactive calibration tool for ina260_ros2.

Run this after wiring the battery to the INA260 breakout and the INA260 to the RPi
over I2C, to work out cell_count, chemistry, voltage cutoffs, current polarity and
(optionally) design_capacity_ah for config/battery.yaml. Talks to the sensor directly
via INA260Sensor - no ROS graph, no node, just a live read/prompt loop over I2C, so it
also doubles as a standalone connectivity check.

    ros2 run ina260_ros2 ina260_calibrate            # guided wizard (default)
    ros2 run ina260_ros2 ina260_calibrate monitor    # just stream live readings
"""

import argparse
import os
import re
import sys
import time

from ina260_ros2.ina260_sensor import INA260Error, INA260Sensor
from ina260_ros2.ocv_tables import OCV_TABLES

# Covers both the LIPO and LION tables in ocv_tables.py with a little margin either side.
_PLAUSIBLE_CELL_VOLTAGE_RANGE = (2.5, 4.35)

_APPLIED_KEYS = (
    'frame_id', 'chemistry', 'cell_count', 'design_capacity_ah',
    'voltage_min_per_cell', 'voltage_max_per_cell', 'current_polarity_inverted',
)


def _prompt(text, default=None):
    suffix = f' [{default}]' if default is not None else ''
    response = input(f'{text}{suffix}: ').strip()
    return response or default


def _prompt_float(text, default=None):
    while True:
        raw = _prompt(text, default)
        try:
            return float(raw)
        except (TypeError, ValueError):
            print('  please enter a number')


def _prompt_yes_no(text, default=True):
    default_str = 'Y/n' if default else 'y/N'
    raw = input(f'{text} [{default_str}]: ').strip().lower()
    if not raw:
        return default
    return raw.startswith('y')


def read_live(sensor, duration_s=None):
    """Stream voltage/current/power at ~2 Hz until Ctrl+C or duration_s elapses."""
    print('Reading INA260 - Ctrl+C to stop' if duration_s is None else 'Reading INA260...')
    print(f'{"time (s)":>10} {"voltage (V)":>12} {"current (A)":>12} {"power (W)":>10}')
    start = time.monotonic()
    try:
        while duration_s is None or time.monotonic() - start < duration_s:
            voltage, current, power = sensor.read()
            elapsed = time.monotonic() - start
            print(f'{elapsed:10.1f} {voltage:12.3f} {current:12.3f} {power:10.3f}')
            time.sleep(0.5)
    except KeyboardInterrupt:
        print()


def measure_resting_voltage(sensor, settle_s=5.0):
    print(f'Measuring resting voltage (averaging over {settle_s:.0f}s - keep the pack idle)...')
    samples = []
    start = time.monotonic()
    while time.monotonic() - start < settle_s:
        voltage, _current, _power = sensor.read()
        samples.append(voltage)
        time.sleep(0.25)
    return sum(samples) / len(samples)


def suggest_cell_counts(pack_voltage):
    """Return [(cell_count, per_cell_voltage), ...] for counts giving a plausible cell voltage.

    Ambiguous by nature - e.g. 12.0V is plausible as both 3S (4.0V/cell) and 4S (3.0V/cell) -
    the wizard lists every plausible option and asks the user to confirm against the pack label.
    """
    suggestions = []
    for cell_count in range(1, 9):
        cell_voltage = pack_voltage / cell_count
        if _PLAUSIBLE_CELL_VOLTAGE_RANGE[0] <= cell_voltage <= _PLAUSIBLE_CELL_VOLTAGE_RANGE[1]:
            suggestions.append((cell_count, cell_voltage))
    return suggestions


def measure_capacity(sensor, cell_count, empty_voltage_per_cell, current_polarity_inverted):
    """Integrate discharge current from a full charge down to the empty cutoff."""
    print()
    print('Capacity measurement: fully charge the pack, then discharge it under a real load.')
    if not _prompt_yes_no('Is the pack now fully charged and discharging under load?'):
        print('Skipping capacity measurement.')
        return None

    print(
        f'Discharging until {empty_voltage_per_cell:.2f} V/cell '
        f'(~{empty_voltage_per_cell * cell_count:.2f} V pack), or Ctrl+C to stop early '
        f'and use what has been measured so far.'
    )
    charge_ah = 0.0
    last_time = time.monotonic()
    try:
        while True:
            voltage, current, _power = sensor.read()
            if current_polarity_inverted:
                current = -current
            now = time.monotonic()
            dt_s = now - last_time
            last_time = now
            # BatteryState convention: discharge current is negative; accumulate its magnitude.
            charge_ah += abs(min(current, 0.0)) * dt_s / 3600.0
            print(
                f'\rmeasured so far: {charge_ah:6.3f} Ah   '
                f'voltage: {voltage:6.3f} V   current: {current:6.3f} A',
                end='',
            )
            if voltage / cell_count <= empty_voltage_per_cell:
                print()
                print('Empty-voltage cutoff reached.')
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        print()
        print('Stopped early by user.')
    return charge_ah


def _default_config_path():
    # Resolve the source-tree config/battery.yaml - works when built with --symlink-install,
    # which is what makes __file__ point back at the source tree instead of a build-dir copy.
    package_root = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    return os.path.join(package_root, 'config', 'battery.yaml')


def _format_yaml_value(value):
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def apply_to_config(path, values):
    """Rewrite known keys' values in-place in a battery.yaml, preserving comments/formatting."""
    with open(path) as f:
        text = f.read()

    for key, value in values.items():
        if value is None:
            continue
        pattern = re.compile(rf'^(\s*{re.escape(key)}:\s*)\S+(.*)$', re.MULTILINE)
        if not pattern.search(text):
            print(f'  warning: key {key!r} not found in {path}, skipping')
            continue
        formatted = _format_yaml_value(value)
        text = pattern.sub(lambda m: f'{m.group(1)}{formatted}{m.group(2)}', text, count=1)

    with open(path, 'w') as f:
        f.write(text)


def _run_wizard(sensor):
    print()
    print('=== Step 1: connectivity check ===')
    read_live(sensor, duration_s=3.0)

    print()
    print('=== Step 2: cell count ===')
    pack_voltage = measure_resting_voltage(sensor)
    print(f'Resting pack voltage: {pack_voltage:.3f} V')
    suggestions = suggest_cell_counts(pack_voltage)
    default_cell_count = suggestions[0][0] if suggestions else 4
    if suggestions:
        print('Plausible cell counts (per-cell voltage in a typical LiPo/Li-ion range):')
        for cell_count, cell_voltage in suggestions:
            print(f'  {cell_count}S -> {cell_voltage:.3f} V/cell')
    else:
        print('No plausible cell count found automatically for that voltage.')
    print("Check the pack's label to disambiguate if more than one option looks plausible.")
    cell_count = int(_prompt_float('Cell count to use', default=default_cell_count))

    print()
    print('=== Step 3: chemistry ===')
    print("Voltage alone can't reliably distinguish LIPO from LION - check the pack label.")
    chemistry = _prompt('Chemistry (LIPO/LION)', default='LIPO').upper()
    while chemistry not in OCV_TABLES:
        chemistry = _prompt(f'Must be one of {sorted(OCV_TABLES)}', default='LIPO').upper()

    print()
    print('=== Step 4: voltage cutoffs ===')
    voltage_min_per_cell = _prompt_float(
        'Minimum safe voltage per cell (health=DEAD below this)', default=3.0
    )
    voltage_max_per_cell = _prompt_float(
        'Maximum safe voltage per cell (health=OVERVOLTAGE above this)', default=4.25
    )

    print()
    print('=== Step 5: current polarity ===')
    print('Apply a known charge or discharge now and watch the sign below.')
    read_live(sensor, duration_s=5.0)
    matches = _prompt_yes_no(
        'Did positive current correctly mean "charging" and negative "discharging"?',
        default=True,
    )
    current_polarity_inverted = not matches

    print()
    print('=== Step 6: design capacity (optional - takes as long as a full discharge) ===')
    design_capacity_ah = None
    if _prompt_yes_no('Measure design_capacity_ah now via a full discharge?', default=False):
        design_capacity_ah = measure_capacity(
            sensor, cell_count, voltage_min_per_cell, current_polarity_inverted
        )

    values = {
        'frame_id': 'base_link',
        'chemistry': chemistry,
        'cell_count': cell_count,
        'design_capacity_ah': design_capacity_ah,
        'voltage_min_per_cell': voltage_min_per_cell,
        'voltage_max_per_cell': voltage_max_per_cell,
        'current_polarity_inverted': current_polarity_inverted,
    }

    print()
    print('=== Recommended config/battery.yaml values ===')
    for key, value in values.items():
        if value is not None:
            print(f'    {key}: {_format_yaml_value(value)}')

    config_path = _default_config_path()
    if os.path.exists(config_path) and _prompt_yes_no(
        f'Write these into {config_path} now?', default=False
    ):
        apply_to_config(config_path, values)
        print(f'Updated {config_path}.')
    else:
        print('Not written - paste the values above into config/battery.yaml by hand.')

    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--i2c-bus', type=int, default=1)
    parser.add_argument('--i2c-address', type=lambda x: int(x, 0), default=0x40)
    subparsers = parser.add_subparsers(dest='command')

    monitor_parser = subparsers.add_parser('monitor', help='Live voltage/current/power readout')
    monitor_parser.add_argument(
        '--duration', type=float, default=None, help='Stop after this many seconds'
    )

    subparsers.add_parser(
        'wizard', help='Guided cell-count/chemistry/polarity/capacity calibration (default)'
    )

    args = parser.parse_args(argv)
    command = args.command or 'wizard'

    try:
        sensor = INA260Sensor(bus_number=args.i2c_bus, address=args.i2c_address)
        sensor.check_identity()
    except INA260Error as exc:
        print(f'Could not talk to the INA260: {exc}', file=sys.stderr)
        return 1

    print(f'INA260 found on bus {args.i2c_bus}, address 0x{args.i2c_address:02X}.')

    try:
        if command == 'monitor':
            read_live(sensor, duration_s=args.duration)
            return 0
        return _run_wizard(sensor)
    finally:
        sensor.close()


if __name__ == '__main__':
    raise SystemExit(main())
