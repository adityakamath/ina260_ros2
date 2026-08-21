"""Tests for calibrate.py's pure logic: cell-count suggestion and config rewriting.

Most interactive wizard prompts aren't tested here, except measure_resting_voltage's
current-check retry and measure_capacity's net-current integration - both fixed bugs
worth a regression test, exercised with a fake sensor and a fake monotonic clock so
they don't need real hardware or real elapsed time.
"""

import ina260_ros2.calibrate as calibrate_module
from ina260_ros2.calibrate import (
    apply_to_config,
    measure_capacity,
    measure_resting_voltage,
    suggest_cell_counts,
)


class _SequenceSensor:
    """Returns fixed (voltage, current, power) readings from a list, one per read() call.

    Advances a shared fake clock on every read() so time-bounded sampling loops (which
    poll time.monotonic()) terminate deterministically without any real elapsed time.
    """

    def __init__(self, readings, fake_time, step_s=0.3):
        self._readings = readings
        self._fake_time = fake_time
        self._step_s = step_s
        self._index = 0

    def read(self):
        voltage, current = self._readings[min(self._index, len(self._readings) - 1)]
        self._index += 1
        self._fake_time[0] += self._step_s
        return voltage, current, 0.0


class TestSuggestCellCounts:

    def test_single_plausible_cell_count(self):
        # 14.8V / 4 = 3.70V/cell (plausible); /3 = 4.93 (implausible); /5 = 2.96 (borderline low
        # but still inside the wide plausibility band used to avoid false negatives).
        suggestions = suggest_cell_counts(14.8)
        cell_counts = [c for c, _v in suggestions]
        assert 4 in cell_counts

    def test_low_voltage_suggests_no_cell_counts(self):
        assert suggest_cell_counts(0.5) == []

    def test_ambiguous_voltage_lists_multiple_candidates(self):
        # 12.0V is plausible both as 3S (4.0V/cell) and 4S (3.0V/cell).
        suggestions = suggest_cell_counts(12.0)
        cell_counts = [c for c, _v in suggestions]
        assert 3 in cell_counts
        assert 4 in cell_counts

    def test_returned_voltages_match_cell_count_division(self):
        for cell_count, cell_voltage in suggest_cell_counts(14.8):
            assert abs(cell_voltage - 14.8 / cell_count) < 1e-9


class TestApplyToConfig:

    def _write_config(self, tmp_path):
        path = tmp_path / 'battery.yaml'
        path.write_text(
            'battery_monitor_node:\n'
            '  ros__parameters:\n'
            '    i2c_bus: 1\n'
            '    frame_id: battery_link\n'
            '    chemistry: LIPO             # LIPO or LION\n'
            '    cell_count: 4\n'
            '    design_capacity_ah: 5.0\n'
            '    current_polarity_inverted: false  # flip if wiring is reversed\n'
        )
        return path

    def test_updates_known_keys(self, tmp_path):
        path = self._write_config(tmp_path)
        apply_to_config(str(path), {'cell_count': 6, 'frame_id': 'base_link'})
        text = path.read_text()
        assert 'cell_count: 6\n' in text
        assert 'frame_id: base_link\n' in text

    def test_preserves_trailing_comments(self, tmp_path):
        path = self._write_config(tmp_path)
        apply_to_config(str(path), {'chemistry': 'LION'})
        text = path.read_text()
        assert 'chemistry: LION             # LIPO or LION\n' in text

    def test_formats_bool_lowercase(self, tmp_path):
        path = self._write_config(tmp_path)
        apply_to_config(str(path), {'current_polarity_inverted': True})
        text = path.read_text()
        assert 'current_polarity_inverted: true' in text

    def test_none_values_are_skipped(self, tmp_path):
        path = self._write_config(tmp_path)
        before = path.read_text()
        apply_to_config(str(path), {'design_capacity_ah': None})
        assert path.read_text() == before

    def test_unknown_key_is_skipped_without_error(self, tmp_path, capsys):
        path = self._write_config(tmp_path)
        apply_to_config(str(path), {'not_a_real_key': 'x'})
        assert 'not_a_real_key' in capsys.readouterr().out


class TestMeasureRestingVoltage:

    def test_averages_voltage_when_current_stays_below_threshold(self, monkeypatch):
        fake_time = [0.0]
        monkeypatch.setattr(calibrate_module.time, 'monotonic', lambda: fake_time[0])
        monkeypatch.setattr(calibrate_module.time, 'sleep', lambda s: None)
        sensor = _SequenceSensor([(4.0, 0.0)], fake_time)

        voltage = measure_resting_voltage(sensor, settle_s=1.0, current_threshold_a=0.05)

        assert voltage == 4.0

    def test_retries_when_current_exceeds_threshold(self, monkeypatch):
        fake_time = [0.0]
        monkeypatch.setattr(calibrate_module.time, 'monotonic', lambda: fake_time[0])
        monkeypatch.setattr(calibrate_module.time, 'sleep', lambda s: None)
        # First settle_s window sees 1.0A (well above threshold) - triggers a retry prompt.
        # Second window is quiet, so the retry should succeed and return its average.
        readings = [(4.0, 1.0)] * 4 + [(4.0, 0.0)] * 4
        sensor = _SequenceSensor(readings, fake_time)

        prompts = []

        def _fake_prompt_yes_no(text, default=True):
            prompts.append(text)
            return True  # accept the retry

        monkeypatch.setattr(calibrate_module, '_prompt_yes_no', _fake_prompt_yes_no)

        voltage = measure_resting_voltage(sensor, settle_s=1.0, current_threshold_a=0.05)

        assert voltage == 4.0
        assert len(prompts) == 1, 'should have prompted exactly once, for the noisy window'


class TestMeasureCapacity:

    def test_nets_out_a_charging_excursion_instead_of_ignoring_it(self, monkeypatch):
        fake_time = [0.0]
        monkeypatch.setattr(calibrate_module.time, 'monotonic', lambda: fake_time[0])
        monkeypatch.setattr(calibrate_module.time, 'sleep', lambda s: None)
        monkeypatch.setattr(calibrate_module, '_prompt_yes_no', lambda text, default=True: True)

        # cell_count=1, empty_voltage_per_cell=3.0 for simplicity.
        # 1A discharge, then 2A charge (e.g. the charger kicks in), then 1A discharge below
        # cutoff. Net Ah = (-(-1) + -(2) + -(-1)) * 1s/3600 = (1 - 2 + 1)/3600 = 0.
        # The old one-sided accumulator (abs(min(current, 0))) would have given (1+0+1)/3600
        # instead, silently dropping the charging step rather than netting it out.
        readings = [(4.0, -1.0), (3.9, 2.0), (2.9, -1.0)]
        sensor = _SequenceSensor(readings, fake_time, step_s=1.0)

        charge_ah = measure_capacity(
            sensor, cell_count=1, empty_voltage_per_cell=3.0, current_polarity_inverted=False
        )

        assert abs(charge_ah - 0.0) < 1e-9
