# SPDX-License-Identifier: Apache-2.0
"""FSYNC: architecture B (shared Audio PLL, one servo) and A (cascade: second servo on the PTP clock)."""
import numpy as np
import pytest

from ptpsim.config import default_scenario
from ptpsim.engine import Simulation, simulate
from ptpsim.pll import AudioPll

P48 = 1e9 / 48_000


def _wrap(x, p=P48):
    return (np.asarray(x) + p / 2) % p - p / 2


# --------------------------------------------------------------------------- B: shared PLL
def shared(**kw):
    return default_scenario().with_overrides(**{"fsync.mode": "shared", **kw})


def test_shared_pll_disciplines_the_phc_and_fsync_keeps_the_timer_residual():
    r = simulate(shared())
    info = r.actuator_info
    assert info["kind"] == "audio_pll" and info["clock_hz"] == 196_608_000
    assert info["timer_delta_ppb"] == pytest.approx(-204.47, abs=0.01)     # frozen ATINC/ATCOR at 196.608 MHz
    assert abs(r.true_offset_ns(np.array([299.0]))[0]) < 20.0              # the PHC is locked
    # the PHC rate error is ~0, FSYNC runs at -timer_delta: the PLL compensates the timer's residual
    t = 299 * 10**12
    phc_rate = r.clock._slope[-1] * 1e9
    fs_rate = r.fsync_clock._slope[-1] * 1e9
    assert abs(phc_rate) < 5.0 and fs_rate == pytest.approx(-info["timer_delta_ppb"], abs=5.0)
    assert r.fsync_clock.phi_hist_ns(t) > 40_000.0                         # ~ 204 ppb x 300 s of phase walked off
    # every NUM written is an integer: the realised FSYNC deviation is a multiple of one LSB
    p = AudioPll()
    d = (r.fsync_clock._slope[-1] - r.eps_ppb[-1] * 1e-9) / (1 + r.eps_ppb[-1] * 1e-9)
    assert abs(d * p.n0 - round(d * p.n0)) < 1e-3


def test_shared_pll_fsync_is_not_stepped_with_the_phc():
    r = simulate(shared(**{"oscillator.initial_offset_ns": 5e6, "firmware.step_threshold_ns": 1_000_000}))
    assert r.counters["steps"] >= 1 and r.clock._step_t
    assert not r.fsync_clock._step_t                                       # the PLL divider is never set


def test_shared_pll_rejects_a_ratio_beyond_the_30_bit_numerator():
    # the PLL reaches -9 304 / +23 464 ppm: a 30 000 ppm slow oscillator needs more than that
    r = simulate(shared(**{"oscillator.freq_error_ppb": -30_000_000.0, "duration_s": 30.0,
                           "firmware.cmd_clamp_ppm": 0.0}))
    assert r.counters["range_resets"] > 0


def test_shared_pll_large_commands_are_written_in_200_khz_steps_one_lock_time_apart():
    # 5 ms -> ~5 000 ppm commands (10 ms would ask -10 000 ppm, beyond the -9 304 ppm the NUM can reach)
    r = simulate(shared(**{"oscillator.initial_offset_ns": 5e6, "oscillator.freq_error_ppb": 0.0,
                           "duration_s": 5.0}))
    t0, _, sl = r.fsync_clock.breakpoints()
    p = AudioPll()
    step_ppm = p.max_step_num / p.n0 * 1e6                                  # 254.3 ppm
    jumps = np.abs(np.diff(sl)) * 1e6
    assert jumps.max() <= step_ppm + 1e-6
    ramp = np.flatnonzero(np.isclose(np.diff(t0), round(p.lock_time_s * 1e12), atol=1))
    assert ramp.size > 10                                                   # a ramp of several writes happened


def test_shared_pll_policies_cap_and_off():
    kw = {"oscillator.initial_offset_ns": 5e6, "oscillator.freq_error_ppb": 0.0, "duration_s": 5.0}
    off = simulate(shared(**{**kw, "fsync.pll.step_policy": "off"}))
    cap = simulate(shared(**{**kw, "fsync.pll.step_policy": "cap"}))
    p = AudioPll()
    assert np.abs(np.diff(off.fsync_clock._slope)).max() * 1e6 > 1000.0      # one write, whatever the size
    assert np.abs(np.diff(cap.fsync_clock._slope)).max() * 1e6 <= p.max_step_num / p.n0 * 1e6 + 1e-6
    assert len(cap.fsync_clock._t0) < len(simulate(shared(**kw)).fsync_clock._t0)


# --------------------------------------------------------------------------- A: cascade
def cascade(**kw):
    return default_scenario().with_overrides(**{"fsync.mode": "cascade", **kw})


def test_cascade_fsync_follows_the_phc_which_follows_the_gm():
    r = simulate(cascade(**{"duration_s": 60.0}))
    f = r.fsync
    assert f["freq_hz"] == 48_000 and f["n_updates"] == pytest.approx(60 * 1000, abs=2)   # one update per ms
    late = f["t_s"] > 40.0
    assert np.abs(_wrap(f["psi_ns"][late] - f["phc_offset_ns"][late])).max() < 3.0     # FSYNC on the PHC grid
    # ... at the PHC rate (which still moves with every PTP update), and so close to the GM grid
    t_ps = np.round(f["t_s"][late] * 1e12).astype(np.int64)
    t0, _, sl = r.clock.breakpoints()
    phc_rate = sl[np.searchsorted(t0, t_ps, side="right") - 1] * 1e9
    assert np.median(np.abs(f["rate_ppb"][late] - phc_rate)) < 2.0
    assert np.abs(_wrap(f["psi_ns"][late])).max() < 50.0


def test_cascade_decimation_sets_the_update_rate():
    r = simulate(cascade(**{"duration_s": 20.0, "fsync.decimation": 480}))
    assert r.fsync["n_updates"] == pytest.approx(20 * 100, abs=2)
    assert np.median(np.diff(r.fsync["t_s"])) == pytest.approx(0.01, abs=1e-8)          # 480 edges of 48 kHz (1 ppm)


def test_cascade_wrapped_detector_realigns_modulo_the_period_after_a_phc_step():
    kw = {"oscillator.initial_offset_ns": 5e6, "firmware.step_threshold_ns": 1_000_000, "duration_s": 60.0}
    w = simulate(cascade(**{**kw, "fsync.phase_detector": "wrapped"}))
    u = simulate(cascade(**kw))                                          # the default: unwrapped
    assert w.counters["steps"] >= 1
    late = w.fsync["t_s"] > 50.0
    # wrapped: on the PHC grid modulo P, but the full 5 ms was not slewed
    assert np.abs(_wrap(w.fsync["psi_ns"][late] - w.fsync["phc_offset_ns"][late])).max() < 3.0
    assert np.abs(_wrap(w.fsync["psi_ns"][late])).max() < 50.0          # on the GM grid modulo the period
    assert np.abs(w.fsync["psi_ns"][late]).min() > 1e6                  # ... but 243 periods (5 ms) away from it
    # unwrapped: follows the PHC phase including the step (anchored on the first edge)
    ul = u.fsync["t_s"] > 50.0
    assert np.abs(u.fsync["psi_ns"][ul] - u.fsync["phc_offset_ns"][ul]).max() < 3.0       # on the PHC itself
    assert np.abs(u.fsync["psi_ns"][ul]).max() < 50.0                   # the whole 5 ms was slewed


def test_cascade_runs_incrementally_with_the_same_result():
    cfg = cascade(**{"duration_s": 30.0})
    whole = simulate(cfg)
    sim = Simulation(cfg)
    for t in (7.3, 15.0, 22.2, 30.0):
        sim.run_until(t)
        part = sim.result()
    for k in ("t_s", "offset_ns", "cmd_ppb", "num", "psi_ns", "rate_ppb"):
        assert np.array_equal(whole.fsync[k], part.fsync[k]), k


def test_cascade_leaves_the_ptp_loop_bit_identical_and_noise_has_its_own_stream():
    base = simulate(default_scenario().with_overrides(duration_s=30.0))
    a = simulate(cascade(duration_s=30.0))
    b = simulate(cascade(**{"duration_s": 30.0, "fsync.capture_noise_ns": 20.0}))
    for name in ("servo_offset_est_ns", "servo_cmd_ppb", "servo_offset_true_ns"):
        assert np.array_equal(getattr(base, name), getattr(a, name)) and np.array_equal(getattr(a, name),
                                                                                         getattr(b, name)), name
    assert not np.array_equal(a.fsync["offset_ns"], b.fsync["offset_ns"])


def test_cascade_timestamps_are_quantised_to_the_timer_tick():
    # NXP timer at 100 MHz: 10 ns tick, so the observed offsets of a locked FSYNC move in steps of the tick
    r = simulate(cascade(**{"actuator.kind": "nxp", "duration_s": 30.0}))
    late = r.fsync["t_s"] > 20.0
    off = r.fsync["offset_ns"][late]
    # the offset is the distance of a 10 ns-aligned timestamp from the grid: its fractional part repeats
    assert len(np.unique(off)) <= 12


def test_cascade_defaults_follow_the_steps_without_winding_up_against_the_num_range():
    """Default detector unwrapped (FSYNC also follows a PTP step); the servo-2 command limit (9 000 ppm) sits inside
    the NUM range (-9 304 ppm), so the integrator is frozen while it saturates: no overshoot beyond a few us."""
    kw = {"oscillator.initial_offset_ns": 5e6, "firmware.step_threshold_ns": 1_000_000, "duration_s": 30.0}
    r = simulate(cascade(**kw))
    assert r.cfg.fsync.phase_detector == "unwrapped" and r.counters["steps"] >= 1
    f = r.fsync
    d = f["psi_ns"] - f["phc_offset_ns"]
    after = f["t_s"] > 4.0                                                # the 5 ms step is slewed at <= 9 000 ppm
    p = AudioPll()
    assert f["num"].min() >= p.num_min and (f["num"] == p.num_for_ppb(-9_000_000.0)).any()   # saturated, inside the range
    assert np.abs(d[after]).max() < 5_000.0                               # overshoot below 5 us
    assert np.abs(d[f["t_s"] > 20.0]).max() < 3.0


def test_overlay_of_a_cascade_run_skips_the_fsync_servo():
    from ptpsim.live import overlay_config
    c = cascade(**{"controller.name": "pi_time_aware", "controller.params": {}})
    assert overlay_config(c).fsync.mode == "off"
    s = shared(**{"controller.name": "pi_time_aware", "controller.params": {}})
    assert overlay_config(s).fsync.mode == "shared"                       # B changes the PTP hardware: kept


def test_export_writes_the_fsync_series(tmp_path):
    from ptpsim.export import export_result
    out = export_result(simulate(cascade(duration_s=5.0)), tmp_path)
    for name in ("fsync_phase.csv", "fsync_rate.csv", "fsync_servo.csv"):
        assert (out / name).exists() and (out / name).stat().st_size > 100
    rows = (out / "fsync_servo.csv").read_text().splitlines()
    assert rows[0].startswith("t_edge_s,offset_seen_ns") and len(rows) == 5 * 1000 + 1
