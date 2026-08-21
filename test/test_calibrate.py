"""Tests for calibrate.py's pure logic: cell-count suggestion and config rewriting.

The interactive wizard prompts aren't tested here - only the deterministic pieces that
don't require stubbing input()/stdout.
"""

from ina260_ros2.calibrate import apply_to_config, suggest_cell_counts


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
