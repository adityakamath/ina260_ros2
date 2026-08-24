"""Per-cell open-circuit-voltage (OCV) to state-of-charge lookup tables.

Generic curves for common RC/robotics chemistries, not characterized against any
specific pack. They're only used to seed and periodically re-anchor the coulomb
counter while the pack is at rest (see coulomb_counter.py) - accurate enough for
that, but replace with a real discharge-test curve for your pack if drift correction
turns out to matter more precisely.
"""

# Each table is a list of (voltage_per_cell, state_of_charge) pairs, ascending by voltage.
OCV_TABLES = {
    'LIPO': [
        (3.00, 0.00),
        (3.30, 0.02),
        (3.50, 0.05),
        (3.60, 0.10),
        (3.70, 0.20),
        (3.75, 0.30),
        (3.80, 0.40),
        (3.85, 0.50),
        (3.90, 0.60),
        (3.95, 0.70),
        (4.00, 0.80),
        (4.10, 0.90),
        (4.20, 1.00),
    ],
    'LION': [
        (2.80, 0.00),
        (3.20, 0.05),
        (3.45, 0.10),
        (3.55, 0.20),
        (3.62, 0.30),
        (3.68, 0.40),
        (3.74, 0.50),
        (3.80, 0.60),
        (3.87, 0.70),
        (3.95, 0.80),
        (4.05, 0.90),
        (4.20, 1.00),
    ],
}


def voltage_to_soc(pack_voltage: float, cell_count: int, chemistry: str) -> float:
    """Map a resting pack voltage to a 0-1 state-of-charge estimate via linear interpolation."""
    table = OCV_TABLES[chemistry]
    cell_voltage = pack_voltage / cell_count

    if cell_voltage <= table[0][0]:
        return table[0][1]
    if cell_voltage >= table[-1][0]:
        return table[-1][1]

    for (v_lo, soc_lo), (v_hi, soc_hi) in zip(table, table[1:]):
        if v_lo <= cell_voltage <= v_hi:
            fraction = (cell_voltage - v_lo) / (v_hi - v_lo)
            return soc_lo + fraction * (soc_hi - soc_lo)

    raise AssertionError('unreachable - table bounds checked above')
