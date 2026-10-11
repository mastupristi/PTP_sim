# SPDX-License-Identifier: Apache-2.0
"""Oscilloscope model: edges of the slave square wave from the exact slave-clock breakpoints."""
import numpy as np

from ptpsim.clock import SlaveClock
from pathlib import Path

from ptpsim.config import SimConfig
from ptpsim.engine import Simulation
from ptpsim.scope import ScopeModel, pick_timebase_ns, square_xy

P = 1_000_000                      # ns
PS_S = 10**12


def model(clock):
    return ScopeModel.from_clock(clock)


def test_identical_clocks_edges_coincide():
    sw = model(SlaveClock(0.0, 0.0)).sweep(1234)
    assert sw.edge_ns == 0.0 and sw.phi_ns == 0.0 and not sw.ambiguous
    assert sw.half_ns == P / 2


def test_constant_offset_sign_slave_ahead_is_drawn_to_the_left():
    # phi > 0: the slave clock is ahead, so it reaches k * P *before* the GM does
    assert model(SlaveClock(+200.0, 0.0)).sweep(7).edge_ns == -200.0
    assert model(SlaveClock(-350.0, 0.0)).sweep(7).edge_ns == +350.0


def test_frequency_error_drifts_linearly():
    slope = 1e-6                                   # slave 1 ppm fast, no control
    m = model(SlaveClock(0.0, slope))
    # at the sweep k the offset is slope * k * P = k ns; the edge is where t + phi(t) = k P -> d = -phi / (1 + slope)
    for k in (1, 10, 1000, 100_000):
        sw = m.sweep(k)
        assert abs(sw.phi_ns - k) < 1e-6
        assert abs(sw.edge_ns - (-k / (1 + slope))) < 1e-6
    assert abs(m.sweep(2000).edge_ns - 2 * m.sweep(1000).edge_ns) < 0.01       # linear in the sweep number


def test_exact_solve_matches_the_clock_within_1ps():
    c = SlaveClock(100_000.0, 20e-6)               # 100 us ahead and 20 ppm fast
    for t_ps, slope in ((50 * PS_S, -35e-6), (120 * PS_S, 5e-6)):
        c.set_slope(t_ps, slope)
    m = model(c)
    for k in (3, 49_999, 50_000, 50_001, 119_999, 120_005, 250_000):
        sw = m.sweep(k)
        t_edge_ps = int(round(k * P * 1000 + sw.edge_ns * 1000))
        # the slave clock must read k * P at the edge instant
        reading_ns = t_edge_ps / 1000 + c.phi_hist_ns(t_edge_ps)
        assert abs((reading_ns + P / 2) % P - P / 2) < 0.002


def test_phase_step_moves_the_edge():
    c = SlaveClock(0.0, 0.0)
    c.step(10 * PS_S, 5_000.0)                     # clock set 5 us forward
    m = model(c)
    assert m.sweep(9_999).edge_ns == 0.0
    assert m.sweep(10_001).edge_ns == -5_000.0


def test_huge_offset_is_wrapped_with_integer_arithmetic():
    # PHC at 0 against a 1.7e18 ns epoch: float64 cannot hold phi to the ns, the integer reduction can
    big = -1_700_000_000_123_456_789
    c = SlaveClock(0.0, 0.0)
    c.step(1, big)                                 # an int delta is exact (a float phi0 would already have lost ~256 ns)
    m = model(c)
    sw = m.sweep(5)
    expected = -(((big % P) + P / 2) % P - P / 2)                   # -(phi mod P) in [-P/2, P/2)
    assert sw.ambiguous
    assert sw.edge_ns == expected
    assert abs(sw.edge_ns) <= P / 2


def test_offset_larger_than_half_a_period_is_flagged_and_wrapped():
    sw = model(SlaveClock(700_000.0, 0.0)).sweep(3)                 # 0.7 ms ahead == 0.3 ms behind
    assert sw.ambiguous
    assert abs(sw.edge_ns - 300_000.0) < 1e-6
    assert not model(SlaveClock(499_999.0, 0.0)).sweep(3).ambiguous


def test_from_simulation_result_tracks_the_true_offset():
    cfg = SimConfig.load(Path(__file__).parent.parent / "configs" / "default_deterministic.json").with_overrides(
        duration_s=60)
    sim = Simulation(cfg)
    sim.run_until(60.0)
    res = sim.result()
    m = ScopeModel.from_clock(res.clock)
    for t in (1.0, 10.0, 30.0, 59.999):
        sw = m.sweep_at(t)
        assert abs(sw.phi_ns - res.true_offset_ns(np.array([sw.t_s]))[0]) < 1e-3
        # the slave clock reads k * P at the edge instant (the edge is -phi / (1 + slope), not exactly -phi)
        t_edge_ps = int(round(sw.k * P * 1000 + sw.edge_ns * 1000))
        reading_ns = t_edge_ps / 1000 + float(res.true_offset_ns(np.array([t_edge_ps / 1e12]))[0])
        assert abs(reading_ns - sw.k * P) < 0.01


def test_square_xy_levels_and_edges():
    x, y = square_xy(0.0, 500.0, -1000.0, 1000.0)
    # rising edges at -1000, 0, 1000 (period 1000), falling at -500, 500
    edges = x[1:-1:2]
    assert list(edges) == [-1000.0, -500.0, 0.0, 500.0, 1000.0]
    assert y[0] == 0.0 and y[1] == 0.0 and y[2] == 1.0              # low, then the rising edge at -1000
    # just right of the trigger the wave is high, just left of it low
    i = int(np.where(x == 0.0)[0][0])
    assert y[i] == 0.0 and y[i + 1] == 1.0
    # rising edge at +300, falling at -200: the window [-100, 100] sits entirely in a low half period
    x2, y2 = square_xy(300.0, 500.0, -100.0, 100.0)
    assert list(np.unique(y2)) == [0.0]


def test_pick_timebase():
    tbs = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000]
    assert pick_timebase_ns(0.0, tbs) == 1
    assert pick_timebase_ns(100.0, tbs) == 50                       # 5 div * 50 = 250 >= 125
    assert pick_timebase_ns(-100.0, tbs) == 50
    assert pick_timebase_ns(1e9, tbs) == 1000                       # larger than everything: the coarsest
