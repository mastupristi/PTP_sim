# SPDX-License-Identifier: Apache-2.0
"""Experimental servo options: command clamp, configurable step threshold, anti-windup PI,
P/I terms recording and the unmodified-firmware overlay."""
from pathlib import Path

import numpy as np
import pytest

from ptpsim.config import FirmwareConfig
from ptpsim.controllers import BaselinePI
from ptpsim.engine import Simulation, simulate
from ptpsim.live import LiveSession, overlay_config, pack_result

from .helpers import quiet_cfg

ACT_LIMIT_PPB = 50_000_000.0


def _aw(cfg, i_max_ppm):
    return cfg.with_overrides(**{"controller.name": "pi_anti_windup",
                                 "controller.params": {"kp": 0.7, "ki": 0.3, "i_max_ppm": i_max_ppm}})


# --------------------------------------------------------------------------- command clamp

def test_command_clamp_is_off_by_default():
    assert FirmwareConfig().cmd_clamp_ppm == 0.0


def test_command_clamp_saturates_instead_of_resetting_and_the_baseline_winds_up():
    cfg = quiet_cfg(**{"oscillator.initial_offset_ns": 200e6, "firmware.cmd_clamp_ppm": 50_000.0})
    r = simulate(cfg)
    assert r.counters["range_resets"] == 0 and r.counters["resets"] == 0 and r.counters["saturated"] > 0
    upd = np.flatnonzero(r.servo_action == 0)
    first = upd[:5]
    assert (r.servo_cmd_ppb[first] < -ACT_LIMIT_PPB).all()             # the PI asks for more ...
    assert (r.servo_cmd_applied_ppb[first] == -ACT_LIMIT_PPB).all()    # ... the clamp applies the limit
    assert np.nanmax(np.abs(r.servo_cmd_applied_ppb)) == ACT_LIMIT_PPB
    # no anti-windup in the baseline: the integrator keeps growing while the command is clamped
    assert (np.diff(r.servo_integral[first]) < 0).all()


@pytest.mark.parametrize("kind", ["ideal", "nxp"])
def test_clamp_exactly_at_the_actuator_limit_is_accepted_both_signs(kind):
    sim = Simulation(quiet_cfg(**{"actuator.kind": kind, "firmware.cmd_clamp_ppm": 50_000.0}))
    sim.fw_mean_delay = 1000
    sim._clock_adjust_rate(200_000_000, 0.25, 0.25, 0)
    sim._clock_adjust_rate(-200_000_000, 0.25, 0.25, 0)
    assert sim.counters["range_resets"] == 0 and sim.counters["saturated"] == 2
    assert sim._s_app == [-ACT_LIMIT_PPB, ACT_LIMIT_PPB]


def test_clamp_does_not_turn_a_non_finite_command_into_full_scale():
    class NanPI(BaselinePI):
        def update(self, s):
            super().update(s)
            return float("nan")

    sim = Simulation(quiet_cfg(**{"firmware.cmd_clamp_ppm": 1000.0}), controller=NanPI())
    sim.fw_mean_delay = 1000
    sim._clock_adjust_rate(1000, 0.25, 0.25, 0)
    assert sim.counters["range_resets"] == 1 and sim.counters["saturated"] == 0
    assert sim._clamp_command(float("inf")) == (float("inf"), False)
    assert sim._clamp_command(-2.0e6) == (-1.0e6, True)
    assert sim._clamp_command(0.5e6) == (0.5e6, False)


# --------------------------------------------------------------------------- step threshold

def test_step_threshold_is_configurable():
    base = quiet_cfg(**{"oscillator.initial_offset_ns": 0.5e9})
    assert simulate(base).counters["steps"] == 0                       # firmware: 1 s
    assert simulate(base.with_overrides(**{"firmware.step_threshold_ns": 100_000_000})).counters["steps"] == 1
    above = quiet_cfg(**{"oscillator.initial_offset_ns": 1.5e9, "firmware.step_threshold_ns": 2_000_000_000})
    assert simulate(above).counters["steps"] == 0


def test_step_threshold_change_applies_live():
    sim = Simulation(quiet_cfg(**{"oscillator.initial_offset_ns": 0.5e9}))
    sim.run_until(5.0)
    assert sim.counters["steps"] == 0
    sim.update_config({"firmware.step_threshold_ns": 100_000_000})
    sim.run_until(10.0)
    assert sim.counters["steps"] == 1


# --------------------------------------------------------------------------- anti-windup PI

def test_anti_windup_with_zero_limit_is_identical_to_the_baseline():
    cfg = quiet_cfg(**{"oscillator.initial_offset_ns": 100e6, "oscillator.freq_error_ppb": 20_000.0,
                       "firmware.cmd_clamp_ppm": 1000.0})
    a, b = simulate(_aw(cfg, 0.0)), simulate(cfg)
    for name in ("servo_cmd_ppb", "servo_integral", "servo_p_ppb", "servo_offset_true_ns"):
        assert np.array_equal(getattr(a, name), getattr(b, name), equal_nan=True), name


def test_integrator_limit_bounds_the_windup_and_the_overshoot():
    cfg = quiet_cfg(**{"duration_s": 300.0, "oscillator.initial_offset_ns": 100e6,
                       "oscillator.freq_error_ppb": 20_000.0, "firmware.cmd_clamp_ppm": 1000.0})
    wound, limited = simulate(cfg), simulate(_aw(cfg, 100.0))
    assert np.max(np.abs(wound.servo_integral)) > 1e9                  # > 1e6 ppm stored by the baseline
    assert np.max(np.abs(limited.servo_integral)) <= 100_000.0
    assert wound.servo_offset_true_ns.min() < -50e6                    # baseline overshoots by > 50 ms
    assert limited.servo_offset_true_ns.min() > -100e3                 # limited: < 100 us
    assert abs(limited.true_offset_ns(np.array([299.0]))[0]) < 10.0


def test_integrator_limit_below_the_steady_correction_leaves_an_offset():
    """I can hold only i_max: P must supply the rest, so e = (20 - 10) ppm / kp = 14.286 us."""
    cfg = quiet_cfg(**{"duration_s": 120.0, "oscillator.initial_offset_ns": 100_000.0,
                       "oscillator.freq_error_ppb": 20_000.0})
    r = simulate(_aw(cfg, 10.0))
    assert r.true_offset_ns(np.array([119.0]))[0] == pytest.approx(10_000.0 / 0.7, rel=1e-3)


# --------------------------------------------------------------------------- P / I recording

def test_p_plus_i_is_the_baseline_output_and_steps_have_no_p_term():
    r = simulate(quiet_cfg(**{"duration_s": 30.0, "oscillator.initial_offset_ns": 3.0e9}))
    upd = r.servo_action == 0
    assert r.counters["steps"] == 1 and upd.sum() > 50
    assert np.array_equal(r.servo_p_ppb[upd] + r.servo_integral[upd], r.servo_cmd_ppb[upd])
    assert np.isnan(r.servo_p_ppb[r.servo_action == 1]).all()
    assert np.array_equal(r.servo_cmd_applied_ppb[upd], r.servo_cmd_ppb[upd])   # no clamp configured
    pk = pack_result(r)
    assert pk["pi_t"].size == upd.sum() and pk["limits"]["clamp"] == 0.0


def test_live_pi_terms_concatenate_to_the_exploration_ones():
    cfg = _aw(quiet_cfg(**{"oscillator.initial_offset_ns": 100e6, "firmware.cmd_clamp_ppm": 2000.0}), 50.0)
    ref = pack_result(simulate(cfg))
    live = LiveSession(cfg)
    parts = [live.delta()]
    for t in (7.3, 21.0, 44.4, 60.0):
        live.advance(t)
        parts.append(live.delta())
    for k in ("pi_t", "pi_p", "pi_i", "pi_out"):
        assert np.array_equal(np.concatenate([p["main"][k] for p in parts]), ref[k]), k
    assert parts[-1]["main"]["limits"] == {"actuator": 50000.0, "clamp": 2000.0, "i_max": 50.0, "sat": 0.0}


# --------------------------------------------------------------------------- overlay

def test_overlay_is_the_unmodified_firmware():
    cfg = quiet_cfg()
    assert overlay_config(cfg) is None                                 # main run already is the firmware
    mod = _aw(cfg.with_overrides(**{"firmware.cmd_clamp_ppm": 100.0, "firmware.step_threshold_ns": 5_000}), 20.0)
    b = overlay_config(mod)
    assert b.firmware == FirmwareConfig()
    assert b.controller.name == "baseline_pi" and b.controller.params == {"kp": 0.7, "ki": 0.3}
    assert b.oscillator == mod.oscillator and b.network == mod.network and b.seed == mod.seed
    clamped_baseline = cfg.with_overrides(**{"firmware.cmd_clamp_ppm": 100.0})
    assert overlay_config(clamped_baseline).firmware == FirmwareConfig()
    compensated = cfg.with_overrides(**{"firmware.delay_rate_comp": True})
    assert overlay_config(compensated).firmware == FirmwareConfig()    # the overlay never gets the experimental option
    assert not overlay_config(compensated).firmware.delay_rate_comp


def test_live_overlay_does_not_receive_firmware_or_controller_changes():
    cfg = quiet_cfg(**{"firmware.cmd_clamp_ppm": 100.0})
    live = LiveSession(cfg, overlay=True)
    assert live.overlay and live.sims["base"].cfg.firmware == FirmwareConfig()
    live.advance(5.0)
    live.update({"firmware.cmd_clamp_ppm": 10.0, "network.delay_ms_ns": 2000.0}, "keep")
    base = live.sims["base"].cfg
    assert base.firmware.cmd_clamp_ppm == 0.0 and base.network.delay_ms_ns == 2000.0
    assert live.sims["main"].cfg.firmware.cmd_clamp_ppm == 10.0
    assert live.delta()["main"]["limits"]["clamp"] == 10.0


# --------------------------------------------------------------------------- per-second PI

def _ps(cfg, **params):
    return cfg.with_overrides(**{"controller.name": "pi_per_second",
                                 "controller.params": {"kp": 0.7, "ki": 0.3, **params}})


def test_per_second_pi_is_bit_identical_to_anti_windup_at_the_reference_interval():
    cfg = quiet_cfg(**{"intervals.sync_log": 0, "oscillator.initial_offset_ns": 100e3,
                       "oscillator.freq_error_ppb": 20_000.0, "duration_s": 60.0})
    a, b = simulate(_ps(cfg, t_ref_s=1.0)), simulate(_aw(cfg, 0.0))
    for name in ("servo_cmd_ppb", "servo_integral", "servo_p_ppb", "servo_offset_true_ns"):
        assert np.array_equal(getattr(a, name), getattr(b, name), equal_nan=True), name


def test_per_second_pi_scales_only_the_integral_gain_with_the_measured_interval():
    from ptpsim.controllers import PIPerSecond, ServoSample
    c = PIPerSecond(kp=0.7, ki=0.3, t_ref_s=1.0)
    out = c.update(ServoSample(offset_ns=-1000, sync_interval_s=0.25, nominal_interval_s=0.25, index=0))
    assert c.ki_eff == pytest.approx(0.075)
    assert c.integral == pytest.approx(0.075 * 1000) and out == pytest.approx(0.7 * 1000 + 75.0)
    # a lost Sync (measured 0.5 s) doubles the step; a huge gap is clamped to the absolute dt_max_s
    c.update(ServoSample(offset_ns=-1000, sync_interval_s=0.5, nominal_interval_s=0.25, index=1))
    assert c.ki_eff == pytest.approx(0.15)
    c.update(ServoSample(offset_ns=-1000, sync_interval_s=900.0, nominal_interval_s=0.25, index=2))
    assert c.ki_eff == pytest.approx(0.3 * 10.0)
    c.set_params({"dt_max_s": 2.0})
    c.update(ServoSample(offset_ns=-1000, sync_interval_s=900.0, nominal_interval_s=0.25, index=3))
    assert c.ki_eff == pytest.approx(0.3 * 2.0)


def test_per_second_pi_caps_kp_so_that_kp_times_dt_stays_below_kp_dt_max():
    from ptpsim.controllers import PIPerSecond, ServoSample

    def kp_eff(dt, **kw):
        c = PIPerSecond(kp=1.6, ki=0.3, **kw)
        out = c.update(ServoSample(offset_ns=-1000, sync_interval_s=dt, nominal_interval_s=dt, index=0))
        assert out == pytest.approx(c.kp_eff * 1000 + c.integral)       # the output uses the applied gain
        return c.kp_eff

    assert kp_eff(0.25) == pytest.approx(1.6)                             # 0.4 <= 1: untouched
    assert kp_eff(1.0) == pytest.approx(1.0)                              # 1.6 > 1: capped
    assert kp_eff(2.0) == pytest.approx(0.5)
    assert kp_eff(0.625) == pytest.approx(1.6) and kp_eff(1.0, kp_dt_max=2.0) == pytest.approx(1.6)
    assert kp_eff(2.0, kp_dt_max=0.0) == pytest.approx(1.6)               # 0 = off
    # the guard acts on the clamped measured interval (dt_max_s), not on a huge gap
    assert kp_eff(900.0, dt_max_s=4.0) == pytest.approx(0.25)
    # kp <= kp_dt_max at dt == t_ref: the law is the firmware one (also covered by the bit-identity test)
    assert PIPerSecond(kp=0.7, ki=0.3).update(
        ServoSample(offset_ns=-1.0, sync_interval_s=1.0, nominal_interval_s=1.0, index=0)) == pytest.approx(0.7 + 0.3)


def test_per_second_pi_bumpless_transfer_uses_the_guarded_gain():
    from ptpsim.controllers import PIPerSecond, ServoSample
    c = PIPerSecond(kp=0.5, ki=0.3)
    c.update(ServoSample(offset_ns=-1000, sync_interval_s=2.0, nominal_interval_s=2.0, index=0))
    last_out = c.last_output
    c.set_params({"kp": 1.6}, "bumpless")                                 # guard: min(1.6, 1/2) = 0.5 at 2 s
    assert c.integral + c._proportional(c.last_error) == pytest.approx(last_out)
    out = c.update(ServoSample(offset_ns=-1000, sync_interval_s=2.0, nominal_interval_s=2.0, index=1))
    assert c.kp_eff == pytest.approx(0.5) and out == pytest.approx(c.kp_eff * 1000 + c.integral)


def test_kp_guard_helps_at_a_long_sync_interval():
    """49 ms / +20 ppm quiet scenario at Sync 1 s, kp 1.6 / ki 0.3: unguarded the loop rings for ~1 min, with the cap
    it settles about twice as fast."""
    from ptpsim.config import SimConfig
    from ptpsim.metrics import MetricsConfig, compute_metrics
    base = SimConfig.load(Path(__file__).parent.parent / "configs" / "default_deterministic.json")

    def settling(kp_dt_max):
        cfg = base.with_overrides(**{
            "duration_s": 300.0, "intervals.sync_log": 0, "oscillator.initial_offset_ns": 49e6,
            "oscillator.freq_error_ppb": 20_000.0, "firmware.cmd_clamp_ppm": 50_000.0,
            "firmware.step_threshold_ns": 50_000_000, "controller.name": "pi_per_second",
            "controller.params": {"kp": 1.6, "ki": 0.3, "i_max_ppm": 150.0, "kp_dt_max": kp_dt_max}})
        return compute_metrics(simulate(cfg), MetricsConfig(band_ns=1000.0))["true_offset"]["settling_s"]

    guarded, unguarded = settling(1.0), settling(0.0)
    assert guarded is not None and unguarded is not None
    assert guarded < 0.6 * unguarded


def test_integral_gain_per_second_is_independent_of_the_sync_interval():
    from ptpsim.controllers import PIPerSecond, ServoSample
    for n in (-4, -2, 0, 1):
        T = 2.0 ** n
        c = PIPerSecond(kp=0.7, ki=0.3, t_ref_s=1.0)
        c.update(ServoSample(offset_ns=-1.0, sync_interval_s=T, nominal_interval_s=T, index=0))
        assert c.ki_eff / T == pytest.approx(0.3)


def test_per_second_pi_keeps_the_damping_across_sync_intervals_where_the_baseline_does_not():
    """100 us step, no frequency error: the undershoot of the baseline grows as the interval shrinks
    (continuous zeta 0.64 -> 0.16 from 1 s to 62.5 ms), the per-second PI does not."""
    from ptpsim.metrics import true_offset_grid

    def undershoot(name, n):
        cfg = quiet_cfg(**{"duration_s": 120.0, "intervals.sync_log": n, "oscillator.initial_offset_ns": 100e3,
                           "controller.name": name, "controller.params": {"kp": 0.7, "ki": 0.3}})
        return -float(np.min(true_offset_grid(simulate(cfg), 0.01)[1]))

    base = {n: undershoot("baseline_pi", n) for n in (-4, -2, 0)}
    flat = {n: undershoot("pi_per_second", n) for n in (-4, -2, 0)}
    assert base[-4] > 1.8 * base[0]                                 # baseline: >= 1.8x worse at 62.5 ms than at 1 s
    assert max(flat.values()) < 1.5 * min(flat.values())             # per-second: within 1.5x across 16x of interval
    assert flat[-4] < 0.5 * base[-4]
