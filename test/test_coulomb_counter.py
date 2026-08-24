"""Tests for CoulombCounter: integration, clamping, rest recalibration, persistence."""

import ina260_battery_monitor.coulomb_counter as coulomb_counter_module
import pytest
from ina260_battery_monitor.coulomb_counter import CoulombCounter

_DESIGN_CAPACITY_AH = 5.0
_CELL_COUNT = 4


def _counter(tmp_path, **overrides):
    kwargs = dict(
        design_capacity_ah=_DESIGN_CAPACITY_AH,
        cell_count=_CELL_COUNT,
        chemistry='LIPO',
        rest_current_threshold_a=0.05,
        rest_settle_s=60.0,
        state_path=str(tmp_path / 'state.yaml'),
        state_max_stale_s=3600.0,
    )
    kwargs.update(overrides)
    return CoulombCounter(**kwargs)


class TestSeeding:

    def test_seeds_from_ocv_when_no_state_file(self, tmp_path):
        counter = _counter(tmp_path)
        counter.seed(voltage=3.70 * _CELL_COUNT)  # -> 0.20 SoC on the LIPO table
        assert abs(counter.percentage - 0.20) < 1e-9

    def test_update_seeds_lazily_on_first_call(self, tmp_path):
        counter = _counter(tmp_path)
        counter.update(voltage=3.70 * _CELL_COUNT, current=0.0, dt_s=1.0)
        assert abs(counter.percentage - 0.20) < 1e-9


class TestIntegration:

    def test_charging_current_increases_charge(self, tmp_path):
        counter = _counter(tmp_path)
        counter.seed(voltage=3.70 * _CELL_COUNT)
        before = counter.charge_ah
        counter.update(voltage=3.70 * _CELL_COUNT, current=1.0, dt_s=3600.0)  # 1 Ah in
        assert abs(counter.charge_ah - (before + 1.0)) < 1e-9

    def test_discharging_current_decreases_charge(self, tmp_path):
        counter = _counter(tmp_path)
        counter.seed(voltage=3.90 * _CELL_COUNT)
        before = counter.charge_ah
        counter.update(voltage=3.90 * _CELL_COUNT, current=-1.0, dt_s=3600.0)
        assert abs(counter.charge_ah - (before - 1.0)) < 1e-9

    def test_charge_clamps_to_design_capacity(self, tmp_path):
        counter = _counter(tmp_path)
        counter.seed(voltage=4.20 * _CELL_COUNT)  # full
        counter.update(voltage=4.20 * _CELL_COUNT, current=10.0, dt_s=3600.0)
        assert counter.charge_ah == _DESIGN_CAPACITY_AH

    def test_charge_clamps_to_zero(self, tmp_path):
        counter = _counter(tmp_path)
        counter.seed(voltage=3.00 * _CELL_COUNT)  # empty
        counter.update(voltage=3.00 * _CELL_COUNT, current=-10.0, dt_s=3600.0)
        assert counter.charge_ah == 0.0


class TestRestRecalibration:

    def test_recalibrates_after_settling_at_rest(self, tmp_path, monkeypatch):
        fake_time = [0.0]
        monkeypatch.setattr(coulomb_counter_module.time, 'monotonic', lambda: fake_time[0])

        counter = _counter(tmp_path, rest_current_threshold_a=0.05, rest_settle_s=60.0)
        # Seed away from truth by draining current, as if the integrator has drifted.
        counter.seed(voltage=3.70 * _CELL_COUNT)  # 0.20 SoC = 1.0 Ah
        counter.update(voltage=3.70 * _CELL_COUNT, current=-0.5, dt_s=3600.0)  # now 0.5 Ah
        assert abs(counter.charge_ah - 0.5) < 1e-9

        # Go idle at a voltage the OCV table maps to a different SoC (0.90 -> 4.5 Ah)
        # and hold there past rest_settle_s.
        fake_time[0] = 10.0
        counter.update(voltage=4.10 * _CELL_COUNT, current=0.0, dt_s=10.0)
        assert abs(counter.charge_ah - 0.5) < 1e-9, 'must not recalibrate before settling'

        fake_time[0] = 71.0  # 61s after going idle - past rest_settle_s=60
        counter.update(voltage=4.10 * _CELL_COUNT, current=0.0, dt_s=61.0)
        assert abs(counter.charge_ah - 4.5) < 1e-9, 'should snap to OCV-derived charge at rest'

    def test_current_above_threshold_resets_rest_timer(self, tmp_path, monkeypatch):
        fake_time = [0.0]
        monkeypatch.setattr(coulomb_counter_module.time, 'monotonic', lambda: fake_time[0])

        counter = _counter(tmp_path, rest_current_threshold_a=0.05, rest_settle_s=60.0)
        counter.seed(voltage=3.70 * _CELL_COUNT)  # 0.20 SoC = 1.0 Ah

        fake_time[0] = 10.0
        counter.update(voltage=3.70 * _CELL_COUNT, current=0.0, dt_s=10.0)  # rest starts at t=10

        fake_time[0] = 50.0
        # Active again before the 60s settle (would land at t=70) elapses - resets the timer.
        counter.update(voltage=3.70 * _CELL_COUNT, current=2.0, dt_s=40.0)

        fake_time[0] = 115.0
        # t=70 (old, un-reset deadline) has passed, but rest only restarted at t=50 here.
        counter.update(voltage=4.10 * _CELL_COUNT, current=0.0, dt_s=65.0)
        assert counter.percentage != pytest.approx(0.90), 'must not have recalibrated yet'

        fake_time[0] = 176.0  # 61s after the t=115 rest restart - now past rest_settle_s
        counter.update(voltage=4.10 * _CELL_COUNT, current=0.0, dt_s=61.0)
        assert counter.percentage == pytest.approx(0.90), 'should recalibrate to OCV at 4.10V/cell'


class TestPersistence:

    def test_save_and_load_round_trip(self, tmp_path):
        state_path = str(tmp_path / 'state.yaml')
        counter = _counter(tmp_path, state_path=state_path)
        counter.seed(voltage=3.90 * _CELL_COUNT)
        counter.update(voltage=3.90 * _CELL_COUNT, current=0.2, dt_s=3600.0)
        counter.save_state()

        reloaded = _counter(tmp_path, state_path=state_path)
        reloaded.seed(voltage=0.0)  # voltage unused when a fresh state file is found
        assert abs(reloaded.charge_ah - counter.charge_ah) < 1e-9

    def test_stale_state_is_ignored(self, tmp_path, monkeypatch):
        state_path = str(tmp_path / 'state.yaml')
        counter = _counter(tmp_path, state_path=state_path, state_max_stale_s=3600.0)
        counter.seed(voltage=3.90 * _CELL_COUNT)
        counter.save_state()

        # Simulate the saved timestamp being far in the past by advancing wall time.
        future_time = coulomb_counter_module.time.time() + 7200
        monkeypatch.setattr(coulomb_counter_module.time, 'time', lambda: future_time)
        reloaded = _counter(tmp_path, state_path=state_path, state_max_stale_s=3600.0)
        reloaded.seed(voltage=3.70 * _CELL_COUNT)  # should fall back to this OCV
        assert abs(reloaded.percentage - 0.20) < 1e-9

    def test_missing_state_file_returns_none(self, tmp_path):
        counter = _counter(tmp_path, state_path=str(tmp_path / 'does_not_exist.yaml'))
        counter.seed(voltage=3.70 * _CELL_COUNT)
        assert abs(counter.percentage - 0.20) < 1e-9
