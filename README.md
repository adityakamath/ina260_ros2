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
- **Service-backed battery events** (`battery_events_node`): turns `/battery_state` into
  discrete `std_srvs/SetBool` events - low/critical/full battery - for anything that
  watches `_service_event` introspection, like a spoken indicator (see
  [Battery events](#battery-events))

## Wiring

Insert the INA260 in series with only the battery's `+` line, upstream of any Y-split
to a charge port and the robot's load - i.e. `VIN+`/`VIN-` between the battery and the
split, `-`/ground running straight through as a continuous common bus. That placement
makes the sensor see **net battery current** (charge current in minus load current out)
as a single signed value, which is exactly what the coulomb counter needs and handles
correctly even while the pack is being charged and driving the robot at the same time -
see [Limitations](#limitations) for the one case this placement can't distinguish.
The INA260's logic-side `GND` (its I2C/STEMMA connector, separate from the `VIN+`/`VIN-`
sense path) needs to share a ground reference with whatever it talks I2C to - normal on a
single-battery robot where that host is powered from the same pack, but worth a
continuity check (battery `-` to INA260 `GND`) if voltage readings look off.

## Quick Start

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/ina260_ros2.git
cd ~/ros2_ws
colcon build --packages-select ina260_ros2
source install/setup.bash
ros2 launch ina260_ros2 battery_monitor.launch.py
```

The checked-in [`config/battery.yaml`](config/battery.yaml) defaults match Seeed
Studio's LeKiwi kit stock battery (E326S, 3S1P Li-ion, 2550 mAh). Running a different
pack? Edit the file (cell count, chemistry, capacity, voltage cutoffs, I2C address) or
run the [calibration wizard](#calibration) below before relying on the published values.

## Calibration

`ina260_calibrate` is a standalone CLI (no ROS graph needed) for working out
`config/battery.yaml`'s pack-specific values after wiring the battery to the INA260
and the INA260 to the RPi over I2C:

```bash
ros2 run ina260_ros2 ina260_calibrate            # guided wizard
ros2 run ina260_ros2 ina260_calibrate monitor    # just stream live voltage/current/power
```

The wizard walks through connectivity, cell-count suggestion (from resting voltage),
chemistry selection, voltage cutoffs, a current-polarity check, and an optional
full-discharge capacity measurement, then offers to write the results straight into
`config/battery.yaml`. Voltage alone can't disambiguate everything automatically
(e.g. 12.0V is plausible as both 3S and 4S, and LIPO/LION look identical on voltage) -
it prompts for confirmation against the pack's label at each ambiguous step rather than
guessing.

The resting-voltage step actively checks current during sampling and warns (offering a
retry) if it wasn't near-zero the whole time - important with the [wiring](#wiring) above,
since "at rest" means disconnecting both the charger and any load, not just the load. The
capacity measurement tracks net current, so a charger left connected during the test is
netted out correctly rather than ignored.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `i2c_bus` | `1` | Linux I2C bus number (`/dev/i2c-N`) |
| `i2c_address` | `0x40` | INA260 I2C address (`0x40`-`0x4F` via `ADDR` pins) |
| `publish_rate_hz` | `2.0` | `BatteryState` publish rate |
| `frame_id` | `base_link` | Header frame ID on published messages |
| `current_polarity_inverted` | `false` | Flip current sign if wiring reads charging as negative |
| `chemistry` | `LION` | `LIPO` or `LION` - selects the OCV table in `ocv_tables.py` |
| `cell_count` | `3` | Series cell count |
| `design_capacity_ah` | `2.55` | Pack design capacity |
| `voltage_min_per_cell` | `3.0` | Below this → `power_supply_health = DEAD` |
| `voltage_max_per_cell` | `4.2` | Above this → `power_supply_health = OVERVOLTAGE` |
| `idle_current_threshold_a` | `0.05` | `\|current\|` below this → `NOT_CHARGING`/`FULL` status |
| `rest_current_threshold_a` | `0.05` | `\|current\|` below this counts as "at rest" for OCV recalibration |
| `rest_settle_s` | `60.0` | Must be at rest this long before recalibrating from OCV |
| `state_file_path` | `~/.ros/ina260_ros2/state.yaml` | Where coulomb-counter state is persisted |
| `state_save_interval_s` | `30.0` | How often state is written to disk |
| `state_max_stale_s` | `3600.0` | Discard saved state older than this; reseed from OCV instead |
| `serial_number` | `""` | `BatteryState.serial_number` |
| `location` | `""` | `BatteryState.location` |

## Battery events

`battery_events_node` subscribes to `/battery_state` and turns it into discrete
`std_srvs/SetBool` events - the same service-plus-introspection pattern this workspace's
audio indicator already narrates for `/emergency_stop`, `/twist_switch`, etc. (see
`twist_switch_node.py` in `lekiwi_control` for the pattern this follows). It hosts each
service and calls it as its own client whenever a genuine edge fires - the only way to
produce the request/response traffic a passive `_service_event` observer needs to see,
since nothing external "calls" a battery threshold the way a person calls `/emergency_stop`.

It's launched together with `battery_monitor_node` from `battery_monitor.launch.py` (it
depends on `battery_monitor_node`'s `/battery_state`, so there's no meaningful way to run
one without the other) - both nodes read their own top-level key out of the same
`params_file`:

```bash
ros2 launch ina260_ros2 battery_monitor.launch.py
```

| Service | Fires `true` when | Fires `false` when |
|---|---|---|
| `/battery_low` | `percentage` falls to/below `low_battery_threshold` | `percentage` climbs back above `low_battery_threshold + low_battery_hysteresis` |
| `/battery_critical` | `percentage` falls to/below `critical_battery_threshold` | `percentage` climbs back above `critical_battery_threshold + critical_battery_hysteresis` |
| `/battery_full` | `percentage` climbs to/above `full_battery_threshold` | `percentage` falls back below `full_battery_threshold - full_battery_hysteresis` |

Each is hysteresis-debounced so a value oscillating near a threshold doesn't spam repeat
calls; `/battery_low` and `/battery_critical` can both fire on the same reading if a drop
jumps straight past both.

**No charging/discharging indicator.** With the INA260 wired between the battery and a
Y-split (charge port + robot load - see [Wiring](#wiring)), net current can't tell
"charger connected but outpaced by the robot's load" apart from "no charger at all" - both
produce the same negative net reading (confirmed empirically: net current stayed negative
both with and without a charger connected while the robot was running, the difference
between the two being exactly the charger's - insufficient - contribution). A real fix
needs an independent signal, like a charger-presence GPIO decoupled from net current
direction, not a smarter algorithm on this one sensor.

| Parameter | Default |
|---|---|
| `battery_state_topic` | `/battery_state` |
| `low_battery_threshold` / `low_battery_hysteresis` | `0.2` / `0.05` |
| `critical_battery_threshold` / `critical_battery_hysteresis` | `0.1` / `0.03` |
| `full_battery_threshold` / `full_battery_hysteresis` | `0.98` / `0.02` |

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

## Limitations

**Rest detection can be fooled by a balanced charge + load.** With the [wiring](#wiring)
above, "net current near zero" is how `coulomb_counter.py` decides the pack is at rest
and safe to recalibrate against the open-circuit-voltage table. But a charger and the
robot's load can also cancel to a near-zero *net* reading while both are actually
flowing - current is genuinely passing through the pack's internal resistance, so the
terminal voltage isn't the true open-circuit voltage that recalibration assumes. A single
net-current sensor at this location can't tell that state apart from genuine rest - there
just isn't a second measurement to disambiguate them with. In practice this only matters
if charge and load current happen to stay closely matched for the full `rest_settle_s`
window, which is a narrow condition rather than typical operation, but it's a real gap
worth knowing about rather than a solved case.

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
