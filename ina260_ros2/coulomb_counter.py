"""Software coulomb counting with rest-triggered OCV recalibration and disk persistence.

Plain current-integration coulomb counting drifts without bound - ADC offset error and
timer jitter accumulate forever with nothing to correct them. This counter bounds that
drift the same way real fuel-gauge ICs do it cheaply: keep integrating current for the
continuous estimate, but whenever the pack has been at rest (near-zero current) for
`rest_settle_s`, snap the accumulator back to what the open-circuit-voltage curve says.
That's a good tradeoff for a mobile-robot gauge without characterizing the pack's
internal resistance for a full equivalent-circuit model (see README's "Future work").

Known limitation: "near-zero net current" isn't proof the pack is truly at rest if the
INA260 sits between the battery and a Y-split (charge port + load) - a charger and a load
current can cancel to a near-zero net reading while both are actually flowing, in which
case the terminal voltage isn't the true open-circuit voltage this relies on. There's no
way to tell that apart from genuine rest using a single net-current sensor at that
location (see README's "Limitations").
"""

import os
import tempfile
import time

import yaml

from ina260_ros2.ocv_tables import voltage_to_soc


class CoulombCounter:

    def __init__(
        self,
        design_capacity_ah: float,
        cell_count: int,
        chemistry: str,
        rest_current_threshold_a: float,
        rest_settle_s: float,
        state_path: str,
        state_max_stale_s: float,
    ):
        self._design_capacity_ah = design_capacity_ah
        self._cell_count = cell_count
        self._chemistry = chemistry
        self._rest_current_threshold_a = rest_current_threshold_a
        self._rest_settle_s = rest_settle_s
        self._state_path = state_path
        self._state_max_stale_s = state_max_stale_s

        self._charge_ah = 0.0
        self._rest_since_monotonic = None
        self._seeded = False

    @property
    def charge_ah(self) -> float:
        return self._charge_ah

    @property
    def percentage(self) -> float:
        return max(0.0, min(1.0, self._charge_ah / self._design_capacity_ah))

    def seed(self, voltage: float) -> None:
        """Seed the counter from persisted state if fresh enough, else from resting OCV."""
        loaded = self._load_state()
        if loaded is not None:
            self._charge_ah = loaded
        else:
            self._charge_ah = self._design_capacity_ah * voltage_to_soc(
                voltage, self._cell_count, self._chemistry
            )
        self._seeded = True

    def update(self, voltage: float, current: float, dt_s: float) -> None:
        if not self._seeded:
            self.seed(voltage)
            return

        self._charge_ah += current * dt_s / 3600.0
        self._charge_ah = max(0.0, min(self._design_capacity_ah, self._charge_ah))

        now = time.monotonic()
        if abs(current) <= self._rest_current_threshold_a:
            if self._rest_since_monotonic is None:
                self._rest_since_monotonic = now
            elif now - self._rest_since_monotonic >= self._rest_settle_s:
                self._charge_ah = self._design_capacity_ah * voltage_to_soc(
                    voltage, self._cell_count, self._chemistry
                )
        else:
            self._rest_since_monotonic = None

    def save_state(self) -> None:
        """Atomically write charge + wall-clock timestamp so a restart can resume from it."""
        os.makedirs(os.path.dirname(self._state_path), exist_ok=True)
        state = {'charge_ah': self._charge_ah, 'timestamp': time.time()}

        fd, tmp_path = tempfile.mkstemp(
            dir=os.path.dirname(self._state_path), prefix='.state-', suffix='.yaml'
        )
        try:
            with os.fdopen(fd, 'w') as f:
                yaml.safe_dump(state, f)
            os.replace(tmp_path, self._state_path)
        except BaseException:
            os.unlink(tmp_path)
            raise

    def _load_state(self):
        try:
            with open(self._state_path) as f:
                state = yaml.safe_load(f)
        except (FileNotFoundError, yaml.YAMLError):
            return None

        if not isinstance(state, dict) or 'charge_ah' not in state or 'timestamp' not in state:
            return None

        age_s = time.time() - state['timestamp']
        if age_s < 0 or age_s > self._state_max_stale_s:
            return None

        return max(0.0, min(self._design_capacity_ah, float(state['charge_ah'])))
