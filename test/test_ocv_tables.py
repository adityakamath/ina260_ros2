"""Tests for the OCV-to-state-of-charge lookup/interpolation."""

from ina260_battery_monitor.ocv_tables import OCV_TABLES, voltage_to_soc


class TestVoltageToSoc:

    def test_below_table_clamps_to_zero(self):
        assert voltage_to_soc(2.0 * 4, cell_count=4, chemistry='LIPO') == 0.0

    def test_above_table_clamps_to_one(self):
        assert voltage_to_soc(5.0 * 4, cell_count=4, chemistry='LIPO') == 1.0

    def test_exact_breakpoint_matches_table(self):
        assert voltage_to_soc(3.70 * 4, cell_count=4, chemistry='LIPO') == 0.20

    def test_interpolates_between_breakpoints(self):
        # Midpoint between (3.70, 0.20) and (3.75, 0.30) -> soc 0.25.
        soc = voltage_to_soc(3.725 * 4, cell_count=4, chemistry='LIPO')
        assert abs(soc - 0.25) < 1e-9

    def test_cell_count_scales_pack_voltage(self):
        # Same per-cell voltage at different cell counts should give the same SoC.
        soc_4s = voltage_to_soc(3.80 * 4, cell_count=4, chemistry='LIPO')
        soc_2s = voltage_to_soc(3.80 * 2, cell_count=2, chemistry='LIPO')
        assert soc_4s == soc_2s

    def test_all_tables_are_sorted_ascending_by_voltage(self):
        for chemistry, table in OCV_TABLES.items():
            voltages = [v for v, _soc in table]
            assert voltages == sorted(voltages), f'{chemistry} table not sorted by voltage'

    def test_all_tables_are_sorted_ascending_by_soc(self):
        for chemistry, table in OCV_TABLES.items():
            socs = [soc for _v, soc in table]
            assert socs == sorted(socs), f'{chemistry} table not sorted by SoC'
