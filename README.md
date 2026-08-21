# INA260 ROS 2 Battery Monitor

![Project Status](https://img.shields.io/badge/Status-Active-green)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
![Python](https://img.shields.io/badge/Python-3-blue?style=flat&logo=python&logoColor=white)
[![CI](https://github.com/adityakamath/ina260_ros2/actions/workflows/ci.yml/badge.svg)](https://github.com/adityakamath/ina260_ros2/actions/workflows/ci.yml)
![License](https://img.shields.io/github/license/adityakamath/ina260_ros2?label=License)

Publishes [`sensor_msgs/BatteryState`](https://docs.ros.org/en/kilted/p/sensor_msgs/interfaces/msg/BatteryState.html)
from an [Adafruit INA260](https://www.adafruit.com/product/4226) current/voltage/power
sensor over I2C, with software coulomb counting for charge and percentage estimation.

**⚠️ Status:** Built for Raspberry Pi 5 running ROS 2 Kilted (Ubuntu 24.04, aarch64).
Talks to the INA260 directly over Linux `i2c-dev` via `smbus2` - no Adafruit
CircuitPython/Blinka dependency.

## Features

- **Direct register access**: reads bus voltage, current and power straight off the
  INA260's registers (`smbus2`), decoded per the TI INA260 datasheet (SBOS866) -
  no Blinka platform-detection layer in the loop
- **Software coulomb counting**: integrates current over time for a continuous
  charge/percentage estimate (`coulomb_counter.py`)
- **Rest-triggered OCV recalibration**: bounds coulomb-counting drift by snapping the
  accumulator back to an open-circuit-voltage-derived estimate whenever the pack has
  been idle past `rest_settle_s` - see [Design notes](#design-notes)
- **State persistence**: accumulated charge is periodically written to disk and
  reloaded on startup, so a restart doesn't lose track of SoC mid-discharge; state
  older than `state_max_stale_s` is distrusted and the estimate reseeds from OCV instead
- **Configurable pack model**: cell count, chemistry (`LIPO`/`LION`), design capacity,
  and voltage cutoffs for `power_supply_health`/`power_supply_status` are all parameters

## Quick Start

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/ina260_ros2.git
cd ~/ros2_ws
colcon build --packages-select ina260_ros2
source install/setup.bash
ros2 launch ina260_ros2 battery_monitor.launch.py
```

Edit [`config/battery.yaml`](config/battery.yaml) for your actual pack (cell count,
capacity, voltage cutoffs, I2C address) before relying on the published values -
the checked-in defaults are placeholders.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `i2c_bus` | `1` | Linux I2C bus number (`/dev/i2c-N`) |
| `i2c_address` | `0x40` | INA260 I2C address (`0x40`-`0x4F` via `ADDR` pins) |
| `publish_rate_hz` | `2.0` | `BatteryState` publish rate |
| `frame_id` | `battery_link` | Header frame ID on published messages |
| `current_polarity_inverted` | `false` | Flip current sign if wiring reads charging as negative |
| `chemistry` | `LIPO` | `LIPO` or `LION` - selects the OCV table in `ocv_tables.py` |
| `cell_count` | `4` | Series cell count |
| `design_capacity_ah` | `5.0` | Pack design capacity |
| `voltage_min_per_cell` | `3.0` | Below this → `power_supply_health = DEAD` |
| `voltage_max_per_cell` | `4.25` | Above this → `power_supply_health = OVERVOLTAGE` |
| `idle_current_threshold_a` | `0.05` | `\|current\|` below this → `NOT_CHARGING`/`FULL` status |
| `rest_current_threshold_a` | `0.05` | `\|current\|` below this counts as "at rest" for OCV recalibration |
| `rest_settle_s` | `60.0` | Must be at rest this long before recalibrating from OCV |
| `state_file_path` | `~/.ros/ina260_ros2/state.yaml` | Where coulomb-counter state is persisted |
| `state_save_interval_s` | `30.0` | How often state is written to disk |
| `state_max_stale_s` | `3600.0` | Discard saved state older than this; reseed from OCV instead |
| `serial_number` | `""` | `BatteryState.serial_number` |
| `location` | `""` | `BatteryState.location` |

## Design notes

Plain current-integration coulomb counting drifts without bound - ADC offset error and
timer jitter accumulate forever with nothing to correct them. This package bounds that
drift the same way cheap fuel-gauge ICs do it: keep integrating current for the
continuous estimate, but whenever the pack has been at rest (`\|current\| <=
rest_current_threshold_a`) for `rest_settle_s`, snap the accumulator back to what an
open-circuit-voltage lookup table says for the resting voltage.

`capacity` is currently always reported equal to `design_capacity` - there's no
capacity-fade tracking (no estimate of a "last full capacity" that degrades with pack
age/cycles).

### Future work

- **Equivalent-circuit Kalman filter**: real fuel-gauge ICs (BQ series, MAX17048, etc.)
  fuse the coulomb-counting prediction with an OCV(SoC) + internal-resistance model in
  an EKF, giving an accurate SoC estimate continuously - even under load, not just at
  rest. This needs a real discharge-test characterization of the pack (OCV-SoC curve,
  internal resistance) that this package doesn't have yet; worth revisiting if
  coulomb-counting drift turns out to be a real problem in practice.
- **Capacity fade tracking**: distinguish `capacity` (last full capacity) from
  `design_capacity` as the pack ages.
- **Time-to-full / time-to-empty estimates**: `sensor_msgs/BatteryState` has no field
  for this, so it's out of scope for this package's core publisher; deferred pending a
  decision on how to expose it (`/diagnostics` KeyValue pairs vs. a custom message).

## License

Apache-2.0
