# SPDX-License-Identifier: Apache-2.0
"""Unit tests: slave clock, disturbance streams (jitter distributions, reproducibility),
controllers (law, gain-change policy, anti-windup), metrics, config round-trip."""
import json

import numpy as np
import pytest

from ptpsim import fwport
from ptpsim.clock import SlaveClock
from ptpsim.config import JitterSpec, SimConfig, noisy_preset
from ptpsim.controllers import BaselinePI, PITimeAware, ServoSample, make_controller
from ptpsim.engine import simulate
from ptpsim.metrics import MetricsConfig, compute_metrics
from ptpsim.rng import Disturbances

from .helpers import no_control, quiet_cfg


# ----------------------------------------------------------------------------- clock

def test_clock_phase_continuity_on_rate_change_and_step_is_explicit():
    c = SlaveClock(100.0, 20e-6, 0)
    c.set_slope(10_000_000_000_000, -5e-6)          # t = 10 s
    assert c.phi_ns(10_000_000_000_000) == pytest.approx(100.0 + 20e-6 * 10e9)   # unchanged
    assert c.phi_ns(20_000_000_000_000) == pytest.approx(100.0 + 20e-6 * 10e9 - 5e-6 * 10e9)
    c.step(20_000_000_000_000, -1000.0)
    assert c.phi_ns(20_000_000_000_000) == pytest.approx(100.0 + 200e3 - 50e3 - 1000.0)
    assert len(c._step_t) == 1                      # the only discontinuity is recorded


def test_clock_read_quantises_and_keeps_integer_precision():
    c = SlaveClock(0.0, 0.0, 0)
    assert c.read_ps(1_234_567, quantum_ps=10_000) == 1_230_000        # floor to 10 ns tick
    assert c.read_ps(1_234_567) == 1_234_567
    c2 = SlaveClock(0.4, 0.0, 0)
    assert c2.read_ps(1000) == 1400


def test_clock_history_lookup_matches_vectorised_samples():
    c = SlaveClock(5.0, 1e-6, 0)
    for k in range(1, 50):
        c.set_slope(k * 1_000_000_000, ((-1) ** k) * k * 1e-7)
    ts = np.array([0, 5, 1_000_000_000, 33_333_333_333, 49_000_000_000], dtype=np.int64)
    assert np.allclose(c.sample_ns(ts), [c.phi_hist_ns(int(t)) for t in ts])


# ------------------------------------------------------------------------ disturbances

def test_stream_value_depends_only_on_index_not_on_draw_order():
    a, b = Disturbances(7), Disturbances(7)
    spec = JitterSpec("normal", 20_000.0)
    fwd = [a.jitter_ns("sync.tx", i, spec) for i in range(100)]
    rev = {i: b.jitter_ns("sync.tx", i, spec) for i in reversed(range(100))}
    assert fwd == [rev[i] for i in range(100)]
    c = Disturbances(8)
    assert [c.jitter_ns("sync.tx", i, spec) for i in range(100)] != fwd
    # streams are independent of each other
    assert [a.jitter_ns("fup.tx", i, spec) for i in range(100)] != fwd


def test_jitter_distributions():
    d = Disturbances(3)
    n = 200_000
    z = np.array([d.jitter_ns("x", i, JitterSpec("normal", 20_000.0)) for i in range(n)])
    assert abs(z.mean()) < 300 and z.std() == pytest.approx(20_000.0, rel=0.01)
    assert z.max() <= 4 * 20_000.0 and z.min() >= -4 * 20_000.0         # clipped at 4 sigma
    u = np.array([d.jitter_ns("y", i, JitterSpec("uniform", 50_000.0)) for i in range(n)])
    assert u.min() >= -50_000.0 and u.max() < 50_000.0 and abs(u.mean()) < 300
    assert u.std() == pytest.approx(50_000.0 / np.sqrt(3), rel=0.01)
    e = np.array([d.jitter_ns("w", i, JitterSpec("exponential", 200.0)) for i in range(n)])
    assert e.min() >= 0 and e.mean() == pytest.approx(200.0, rel=0.02)
    assert all(d.jitter_ns("n", i, JitterSpec("none", 5.0)) == 0.0 for i in range(10))


def test_tx_jitter_applied_to_all_message_types_end_to_end():
    cfg = noisy_preset()
    cfg.duration_s = 120.0
    cfg.network.jitter_ms = JitterSpec()
    cfg.network.jitter_sm = JitterSpec()
    r = simulate(cfg)
    ideal_sync = (np.arange(r.sync_tx_s.size) + 1) * 0.25
    dev = (r.sync_tx_s - ideal_sync) * 1e9                                    # ns
    assert dev.std() == pytest.approx(20_000.0, rel=0.1) and abs(dev.mean()) < 3_000
    assert np.max(np.abs(dev)) <= 4 * 20_000.0 + 1                            # clipped, both signs
    assert dev.min() < 0 < dev.max()
    dq = np.diff(r.dreq_tx_s)
    assert dq.std() > 1e-6                                                    # Delay_Req jitters too
    # Follow_Up and Delay_Resp latencies show up in the processing times
    lat = r.servo_t_proc_s - r.servo_t_sample_s
    assert lat.std() > 1e-6


# ----------------------------------------------------------------------- controllers

def sample(offset, dt=0.25):
    return ServoSample(offset_ns=offset, sync_interval_s=dt, nominal_interval_s=0.25, index=0)


def test_baseline_is_the_firmware_pi_law():
    c = BaselinePI(kp=0.7, ki=0.3)
    # error = -offset; integral updated first; absolute output
    assert c.update(sample(1000)) == pytest.approx(0.7 * -1000 + 0.3 * -1000)
    assert c.update(sample(1000)) == pytest.approx(0.7 * -1000 + 0.3 * -2000)
    assert c.integral == pytest.approx(-600.0)
    c.reset()
    assert c.integral == 0.0 and c.params["kp"] == 0.7
    # no dependence on the sampling interval
    assert BaselinePI().update(sample(500, 0.25)) == BaselinePI().update(sample(500, 4.0))


def test_gain_change_policies_are_explicit():
    c = BaselinePI(kp=0.7, ki=0.3)
    for _ in range(5):
        c.update(sample(2000))
    keep_int, last_out = c.integral, c.last_output
    c.set_params({"kp": 0.3}, "keep")
    assert c.integral == keep_int                                    # integrator untouched
    c.set_params({"kp": 0.7}, "keep")
    c.set_params({"kp": 0.2, "ki": 0.1}, "bumpless")
    assert c.last_error == -2000 and c.integral == pytest.approx(last_out - 0.2 * -2000)
    c.set_params({"kp": 0.5}, "reset")
    assert c.integral == 0.0
    c2 = BaselinePI()
    c2.update(sample(100))
    c2.set_params({"kp": 0.7, "ki": 0.3}, "reset")                  # no change -> no hidden reset
    assert c2.integral != 0.0
    with pytest.raises(ValueError):
        c2.set_params({"kp": 1.0}, "whatever")
    with pytest.raises(KeyError):
        c2.set_params({"nope": 1.0})


def test_time_aware_pi_gains_are_per_second_and_interval_independent():
    a = PITimeAware(wn=1.0, zeta=1.0)
    assert a.kp == pytest.approx(2.0) and a.ki == pytest.approx(1.0)
    # same continuous-time behaviour whatever the interval: integrator step = ki * dt * e
    a.update(sample(1000, 0.25))
    assert a.integral == pytest.approx(-1.0 * 0.25 * 1000)
    b = PITimeAware(wn=1.0, zeta=1.0, wn_ts_max=1.0)
    b.update(sample(1000, 1.0))
    assert b.integral == pytest.approx(-1000.0)
    # stability guard: kp * dt < 2 is enforced through wn * dt <= wn_ts_max
    g = PITimeAware(wn=1.0, zeta=1.0, wn_ts_max=0.35)
    g.update(sample(1000, 1.0))
    assert g.kp * 1.0 == pytest.approx(0.7) and g.integral == pytest.approx(-0.35 ** 2 * 1000)


def test_anti_windup_conditional_integration():
    c = PITimeAware(wn=1.0, zeta=1.0, sat_ppb=1000.0)
    for _ in range(200):
        out = c.update(sample(1e6))                                  # huge error: saturated
        assert out == -1000.0
    assert abs(c.integral) <= 1000.0 + 1.0                           # no windup
    # a naive integrator would be at -1e6 * 200 * 0.25; after the error flips the output leaves saturation at once
    out = c.update(sample(-1e6))
    assert out == 1000.0 and abs(c.integral) <= 1.1e6 * 0.25 + 1000.0
    assert c.saturated_count == 201


def test_registry_and_unknown_names():
    assert make_controller("baseline_pi").params == {"kp": 0.7, "ki": 0.3}
    with pytest.raises(KeyError):
        make_controller("nope")


# ---------------------------------------------------------------------------- metrics

def test_metrics_settling_overshoot_rms_bias_on_known_response():
    cfg = quiet_cfg()
    cfg.duration_s = 200.0
    cfg.oscillator.initial_offset_ns = 100_000.0
    r = simulate(cfg)
    m = compute_metrics(r, MetricsConfig(band_ns=1000.0, dwell_s=10.0, final_window_s=50.0))
    t = m["true_offset"]
    assert t["settling_s"] is not None and 5 < t["settling_s"] < 60
    assert 0.0 < t["overshoot_pct"] < 100.0
    assert t["peak_abs_ns"] >= 100_000.0
    assert t["rms_final_ns"] < 5.0 and not t["diverged"]
    assert m["estimated_offset"]["settling_s"] is not None
    assert m["sim_wall_ms"] > 0


def test_metrics_edge_cases_never_settles_zero_initial_offset_and_divergence():
    cfg = no_control(quiet_cfg())
    cfg.oscillator.freq_error_ppb = 50_000.0                          # uncontrolled drift 50 ppm
    cfg.duration_s = 100.0
    m = compute_metrics(simulate(cfg), MetricsConfig(band_ns=1000.0))
    t = m["true_offset"]
    assert t["settling_s"] is None and "outside" in t["settling_reason"]
    assert t["overshoot_pct"] == 0.0 and t["overshoot_vs_initial_pct"] is None   # no opposite excursion; x0 == 0
    assert t["peak_abs_ns"] == pytest.approx(5.0e6, rel=1e-6)         # exact vertex peak
    cfg = no_control(quiet_cfg())                                     # nothing happens at all
    cfg.duration_s = 30.0
    t = compute_metrics(simulate(cfg))["true_offset"]
    assert t["settling_s"] == 0.0 and t["overshoot_pct"] is None
    cfg = no_control(quiet_cfg())
    cfg.oscillator.freq_error_ppb = 2e7                               # 2 % : diverging offset
    cfg.duration_s = 200.0
    assert compute_metrics(simulate(cfg))["true_offset"]["diverged"]


def test_metrics_dwell_requirement():
    cfg = quiet_cfg()
    cfg.duration_s = 40.0
    cfg.oscillator.initial_offset_ns = 100_000.0
    m = compute_metrics(simulate(cfg), MetricsConfig(band_ns=1000.0, dwell_s=30.0))
    assert m["true_offset"]["settling_s"] is None
    assert "dwell" in m["true_offset"]["settling_reason"]


# ----------------------------------------------------------------------------- config

def test_config_roundtrip_and_overrides(tmp_path):
    cfg = noisy_preset()
    cfg.controller.params = {"kp": 0.5, "ki": 0.2}
    p = tmp_path / "cfg.json"
    cfg.save(p)
    back = SimConfig.load(p)
    assert back.to_dict() == cfg.to_dict()
    assert json.loads(p.read_text())["seed"] == 1
    c2 = cfg.with_overrides(**{"intervals.sync_log": -3, "controller.params.kp": 0.1})
    assert c2.intervals.sync_log == -3 and c2.controller.params["kp"] == 0.1 and cfg.intervals.sync_log == -2
    with pytest.raises(KeyError):
        cfg.with_overrides(**{"intervals.nope": 1})
    with pytest.raises(KeyError):
        SimConfig.from_dict({"bogus": 1})


def test_steady_state_statistics_and_transient_end():
    cfg = quiet_cfg()
    cfg.duration_s = 200.0
    cfg.oscillator.initial_offset_ns = 100_000.0
    cfg.oscillator.freq_error_ppb = 20_000.0
    r = simulate(cfg)
    m = compute_metrics(r, MetricsConfig(final_window_s=50.0))
    t = m["true_offset"]
    for k in ("median_ns", "min_ns", "max_ns", "median_abs_ns", "p2p_ns", "max_abs_ns"):
        assert np.isfinite(t[k])
    assert t["min_ns"] <= t["median_ns"] <= t["max_ns"] and t["p2p_ns"] == pytest.approx(t["max_ns"] - t["min_ns"])
    assert m["transient_end_s"] == t["settling_s"] and t["steady_start_s"] == pytest.approx(150.0)
    assert m["delay"]["n"] > 10 and abs(m["delay"]["median_ns"] - 1000.0) < 2.0
    m2 = compute_metrics(r, MetricsConfig(final_window_s=50.0, steady_from_settling=True))
    assert m2["steady_start_s"] == pytest.approx(t["settling_s"])
