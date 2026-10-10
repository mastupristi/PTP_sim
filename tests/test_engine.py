# SPDX-License-Identifier: Apache-2.0
"""Closed-loop engine tests: clocks, timestamps, pairing, firmware servo logic, intervals."""
import heapq

import numpy as np
import pytest

from ptpsim import fwport
from ptpsim.analysis import baseline_poles, baseline_recursion
from ptpsim.config import SimConfig, noisy_preset
from ptpsim.engine import PS_PER_S, Simulation, simulate

from .helpers import no_control, quiet_cfg


# --------------------------------------------------------------------------- basic physics

def test_identical_clocks_symmetric_network_no_noise():
    r = simulate(quiet_cfg())
    t = np.linspace(0, 60, 6001)
    assert np.max(np.abs(r.true_offset_ns(t))) == 0.0
    assert r.counters["steps"] == 0 and r.counters["resets"] == 0
    assert r.counters["pairs"] > 200
    # the servo ran, and delay and offset estimates are exact (integer ns arithmetic)
    assert (r.delay_est_ns == 1000).all()
    assert (r.servo_offset_est_ns == 0).all()
    assert r.servo_offset_est_ns.size > 200


def test_zero_path_delay_never_runs_the_servo():
    """Firmware quirk: mean_delay == 0 means 'no delay yet', the servo is skipped (clock.c:786)."""
    cfg = quiet_cfg(**{"network.delay_ms_ns": 0.0, "network.delay_sm_ns": 0.0})
    cfg.oscillator.initial_offset_ns = 1000.0
    cfg.oscillator.freq_error_ppb = 5000.0
    # (delay exactly 0 requires phi == 0 at t2/t3 in the estimate; with an offset the estimate is
    #  not zero, so use identical clocks to reproduce the quirk)
    cfg.oscillator.initial_offset_ns = 0.0
    cfg.oscillator.freq_error_ppb = 0.0
    r = simulate(cfg)
    assert r.servo_t_proc_s.size == 0
    assert (r.delay_est_ns == 0).all() and r.delay_est_ns.size > 5


def test_known_initial_offset_same_rate_no_control():
    cfg = no_control(quiet_cfg())
    cfg.oscillator.initial_offset_ns = 123_456.0
    r = simulate(cfg)
    t = np.linspace(0, 60, 601)
    assert np.allclose(r.true_offset_ns(t), 123_456.0, atol=0, rtol=0)
    assert (r.servo_offset_est_ns == 123_456).all()   # constant offset: estimate exact
    assert (r.delay_est_ns == 1000).all()


def test_known_frequency_error_no_control():
    cfg = no_control(quiet_cfg())
    cfg.oscillator.freq_error_ppb = 20_000.0
    r = simulate(cfg)
    t = np.array([0.0, 10.0, 30.0, 60.0])
    assert np.allclose(r.true_offset_ns(t), 20_000.0 * t, atol=1e-6)


def test_rate_variable_between_t2_and_t3_biases_delay_estimate_on_constant_network():
    """delay_est = d + (phi(t2) - phi(t3))/2 even with a perfectly constant, symmetric network."""
    cfg = no_control(quiet_cfg())
    cfg.oscillator.freq_error_ppb = 20_000.0
    r = simulate(cfg)
    assert r.delay_est_ns.size > 10
    phi = lambda ts: r.true_offset_ns(ts)
    expected = 1000.0 + (phi(r.delay_t2_phys_s) - phi(r.delay_t3_phys_s)) / 2.0
    assert np.max(np.abs(r.delay_est_ns - expected)) <= 1.0     # ns truncation only
    # and it is a real bias: of the order of eps * (t3 - t2) / 2, up to 2.5 us here
    assert np.max(np.abs(r.delay_est_ns - 1000.0)) > 1000.0


def test_phase_is_continuous_when_rate_changes():
    cfg = quiet_cfg()
    cfg.oscillator.initial_offset_ns = 50_000.0
    cfg.oscillator.freq_error_ppb = 20_000.0
    r = simulate(cfg)
    assert r.counters["steps"] == 0
    t, phi0, slope = r.clock.breakpoints()
    assert t.size > 100                                    # many rate changes
    # phase before each anchor == phase at the anchor (no jump), checked on an exact grid
    before = phi0[:-1] + slope[:-1] * ((t[1:] - t[:-1]) / 1000.0)
    assert np.max(np.abs(before - phi0[1:])) < 1e-6


# ---------------------------------------------------------- timestamps vs message arrival

def test_t2_is_taken_at_sync_arrival_not_at_follow_up_processing():
    cfg = no_control(quiet_cfg())
    cfg.oscillator.freq_error_ppb = 20_000.0
    cfg.latency.follow_up_ns = 200e6                       # Follow_Up 200 ms after the Sync
    cfg.loss.delay_req = 1.0                               # keep the delay fixed (set below)
    sim = Simulation(cfg)
    sim.fw_mean_delay = 1000                               # d_ms == d_sm == 1000: exact compensation
    r = sim.run()
    assert r.servo_t_proc_s.size > 100
    # estimate == true offset at the t2 instant (to 1 ns); differs from the offset at processing
    assert np.max(np.abs(r.servo_offset_est_ns - r.servo_offset_true_ns)) <= 1.0
    gap = r.servo_offset_true_proc_ns - r.servo_offset_true_ns
    assert np.min(gap) > 3500.0                            # 200 ms * 20 ppm = 4000 ns
    # t2 acquisition instant precedes the processing by (follow_up latency + net) at least
    assert np.min(r.servo_t_proc_s - r.servo_t_sample_s) >= 0.2


def test_t3_is_taken_at_delay_req_transmission_not_at_delay_resp():
    cfg = no_control(quiet_cfg())
    cfg.oscillator.freq_error_ppb = 20_000.0
    cfg.latency.delay_resp_ns = 500e6                      # Delay_Resp 0.5 s after Delay_Req
    r = simulate(cfg)
    # the estimate uses phi at the Delay_Req TX instant (not at the Delay_Resp processing)
    phi = lambda ts: r.true_offset_ns(ts)
    expected = 1000.0 + (phi(r.delay_t2_phys_s) - phi(r.delay_t3_phys_s)) / 2.0
    assert np.max(np.abs(r.delay_est_ns - expected)) <= 1.0
    assert np.min(r.delay_t_proc_s - r.delay_t3_phys_s) >= 0.5


# --------------------------------------------------------------- pairing / stale data

def _recording_sim():
    sim = Simulation(quiet_cfg())
    calls = []
    sim._port_synchronize = lambda **kw: calls.append(kw)
    return sim, calls


def test_pairing_both_orders_and_stale_replacement():
    sim, calls = _recording_sim()
    ooo = sim._ooo_handle
    ooo(("S", 5, 1000, 0, 0)); ooo(("F", 5, 900, 0, 0))            # Sync then Follow_Up
    assert len(calls) == 1 and (calls[0]["t2"], calls[0]["t1"]) == (1000, 900)
    ooo(("F", 6, 2900, 0, 0)); ooo(("S", 6, 3000, 0, 0))           # Follow_Up first
    assert len(calls) == 2 and (calls[1]["t2"], calls[1]["t1"]) == (3000, 2900)
    ooo(("S", 7, 1, 0, 0)); ooo(("F", 8, 2, 0, 0))                 # mismatching: Sync 7 dropped
    assert len(calls) == 2 and sim.fw_slot[1] == 8
    ooo(("S", 8, 3, 0, 0))                                          # pairs with the stored F8
    assert len(calls) == 3
    ooo(("S", 9, 1, 0, 0)); ooo(("S", 10, 2, 0, 0)); ooo(("F", 10, 5, 0, 0))
    assert len(calls) == 4 and calls[3]["t2"] == 2                 # Sync 9 lost its Follow_Up
    assert sim.counters["stale_pairs"] >= 2
    ooo(("S", 11, 1, 0, 0)); ooo(("F", 9, 5, 0, 0)); ooo(("F", 11, 6, 0, 0))   # old Follow_Up 9 replaces S11
    assert len(calls) == 4                                          # F11 can't pair: slot held F9
    assert sim.fw_slot[:2] == ("F", 11)


def test_correction_field_is_floor_shifted():
    sim = Simulation(quiet_cfg())
    got = []
    sim._clock_synchronize = lambda ingress, egress, *a: got.append((ingress, egress))
    # corrections in ns * 2^16: -1 (=-2^-16 ns) >> 16 == -1 (arithmetic shift), 65536*3 -> 3
    sim._port_synchronize(t2=1000, t1=500, corr1=-1, corr2=65536 * 3, t_arr=0)
    assert got == [(1000, 500 - 1 + 3)]


def test_follow_up_loss_leaves_stale_syncs_and_system_keeps_running():
    cfg = quiet_cfg()
    cfg.loss.follow_up = 0.3
    cfg.oscillator.initial_offset_ns = 10_000.0
    r = simulate(cfg)
    assert r.counters["stale_pairs"] > 10
    assert r.counters["pairs"] < r.counters["sync_rx"]
    assert abs(r.true_offset_ns(np.array([59.0]))[0]) < 1000.0     # still converging


def test_delay_resp_for_unknown_request_is_ignored_and_list_cleanup_after_3s():
    cfg = quiet_cfg()
    cfg.loss.delay_resp = 1.0                                      # no response ever arrives
    sim = Simulation(cfg)
    r = sim.run()
    assert r.delay_est_ns.size == 0
    assert len(sim.fw_delay_list) <= 2          # entries older than 3 s were purged at each timer
    sim2, _ = _recording_sim()
    sim2._on_slave_dresp_proc((77, 1, 0, 1, 0.0))                  # unknown seq: nothing happens
    assert sim2.fw_mean_delay == 0


def test_step_clears_timestamps_and_delay_but_not_the_request_list():
    sim = Simulation(quiet_cfg())
    sim.fw_t1, sim.fw_t2, sim.fw_mean_delay = 5, 6, 1000
    sim.fw_delay_list[3] = {"t_phys": 0, "t3": 7, "t3_hw": 7, "hw": True}
    sim.now = 1_000_000_000
    sim._clock_step(1_500_000_000, 0)
    assert (sim.fw_t1, sim.fw_t2, sim.fw_mean_delay) == (0, 0, 0)
    assert 3 in sim.fw_delay_list                                   # NOT cleared (firmware behaviour)
    assert sim.clock.phi_ns(sim.now) == -1.5e9
    assert sim.counters["steps"] == 1
    # ptp_clock_delay is a no-op until a new Sync pair re-arms t1/t2
    sim._ptp_clock_delay(1, 2, 0.0, 0)
    assert sim.fw_mean_delay == 0


def test_large_initial_offset_is_stepped_once_then_converges():
    cfg = quiet_cfg()
    cfg.oscillator.initial_offset_ns = 1.5e9
    r = simulate(cfg)
    assert r.counters["steps"] == 1
    assert abs(r.true_offset_ns(np.array([59.9]))[0]) < 100.0


# ---------------------------------------------------- lock / outlier / saturation logic

def test_lock_after_three_samples_then_outlier_rejection_and_reset():
    sim = Simulation(quiet_cfg())
    sim.fw_mean_delay = 1000
    for off in (5000, 3000, 1000):
        sim._clock_adjust_rate(off, 0.25, 0.25, 0)
    assert sim.fw_locked
    n_cmd = len(sim._r_t)
    sim._clock_adjust_rate(150_000_000, 0.25, 0.25, 0)      # outlier 1/2: rejected, no command
    assert sim.counters["outliers"] == 1 and len(sim._r_t) == n_cmd and sim.fw_locked
    sim._clock_adjust_rate(150_000_000, 0.25, 0.25, 0)      # 2/2: servo reset
    assert not sim.fw_locked and sim.ctrl.integral == 0.0 and sim.act.effective_ratio == 1.0
    # before lock there is no rejection: a 150 ms offset goes straight to the PI
    sim._clock_adjust_rate(150_000_000, 0.25, 0.25, 0)
    assert sim.counters["outliers"] == 2


def test_out_of_window_command_resets_servo_like_the_driver():
    """kp*offset beyond +-50000 ppm: the firmware does not clamp, the driver rejects -> reset."""
    cfg = quiet_cfg()
    cfg.oscillator.initial_offset_ns = 200e6
    r = simulate(cfg)
    assert r.counters["range_resets"] > 50
    assert abs(r.true_offset_ns(np.array([59.0]))[0]) > 100e6      # never converges: reset loop


def test_variant_saturates_with_anti_windup_and_converges_from_large_offset():
    cfg = quiet_cfg()
    cfg.duration_s = 260.0
    cfg.oscillator.initial_offset_ns = 50e6     # 50 ms at 400 ppm max slew: ~125 s of saturation
    cfg.controller.name = "pi_time_aware"
    cfg.controller.params = {"wn": 1.0, "zeta": 1.0, "sat_ppb": 400_000.0}
    r = simulate(cfg)
    assert r.counters["saturated"] > 5
    assert r.counters["range_resets"] == 0
    assert np.nanmax(np.abs(r.servo_cmd_ppb)) <= 400_000.0 + 1e-9
    assert np.max(np.abs(r.servo_integral)) < 1.5e6
    assert abs(r.true_offset_ns(np.array([259.0]))[0]) < 50.0


def test_nominal_ppb_to_ratio_quantisation_is_applied():
    cfg = no_control(quiet_cfg())
    r = simulate(cfg)
    assert (r.rate_cmd_ppb == 0).all()


# ------------------------------------------------------------------------ asymmetry

def test_asymmetry_biases_true_offset_by_half_the_difference():
    cfg = quiet_cfg()
    cfg.duration_s = 120.0
    cfg.network.delay_ms_ns = 1500.0
    cfg.network.delay_sm_ns = 500.0
    r = simulate(cfg)
    final = r.true_offset_ns(np.linspace(100, 120, 200))
    # est offset = o + (dms - dsm)/2 -> servo drives it to 0, so true o = -(dms - dsm)/2 = -500 ns
    assert np.allclose(final, -500.0, atol=2.0)
    assert abs(np.mean(r.servo_offset_est_ns[-50:])) < 2.0         # the estimate looks perfect


# --------------------------------------------------------------------- intervals / timers

def test_delay_req_first_random_then_period_independent_of_servo_and_sync():
    cfg = quiet_cfg()
    cfg.duration_s = 40.0
    r = simulate(cfg)
    d = r.dreq_tx_s
    assert 0 < d[0] <= 4.0                                  # uniform in (0, 2 * 2^1]
    assert np.allclose(np.diff(d), 2.0, atol=1e-9)
    # changing the Sync interval does not change the independent Delay_Req period
    cfg2 = cfg.with_overrides(**{"intervals.sync_log": -4})
    r2 = simulate(cfg2)
    assert np.allclose(r2.dreq_tx_s, r.dreq_tx_s)


def test_every_n_sync_mode_follows_the_sync_count():
    cfg = quiet_cfg()
    cfg.intervals.delay_mode = "every_n_sync"
    cfg.intervals.delay_every_n = 8
    r = simulate(cfg)
    d = np.diff(r.dreq_tx_s)
    assert np.allclose(d, 8 * 0.25, atol=1e-3)
    cfg.intervals.sync_log = -3
    r = simulate(cfg)
    assert np.allclose(np.diff(r.dreq_tx_s), 8 * 0.125, atol=1e-3)     # N kept, period follows


def test_live_interval_changes():
    cfg = quiet_cfg()
    cfg.duration_s = 60.0
    sim = Simulation(cfg)
    sim.run_until(20.0)
    sim.update_config({"intervals.sync_log": -3})
    sim.run_until(30.0)
    sim.update_config({"intervals.delay_log": 0})
    sim.run_until(60.0)
    r = sim.result()
    sy = np.diff(r.sync_tx_s)
    t = r.sync_tx_s[1:]
    assert np.allclose(sy[t < 20.0], 0.25, atol=1e-9)
    assert np.allclose(sy[t > 21.0], 0.125, atol=1e-9)
    dq = np.diff(r.dreq_tx_s)
    tq = r.dreq_tx_s[1:]
    assert np.allclose(dq[tq < 30.0], 2.0, atol=1e-9)
    assert np.allclose(dq[tq > 36.0], 1.0, atol=1e-9)             # adopted from the next Delay_Resp
    assert [c[0] for c in r.changes] == [20.0, 30.0]


def test_delay_timer_error_independent_of_phc():
    cfg = quiet_cfg()
    cfg.oscillator.timer_error_ppb = 100_000.0                     # timer 1e-4 slow in physical time
    r = simulate(cfg)
    assert np.allclose(np.diff(r.dreq_tx_s), 2.0 / (1 + 1e-4), atol=1e-9)


# ---------------------------------------------------------- ordering and reproducibility

def test_event_order_is_deterministic_at_equal_times():
    sim = Simulation(quiet_cfg())
    order = []
    sim._heap.clear()
    for prio, name in [(3, "proc"), (1, "tx"), (2, "rx"), (0, "rate"), (2, "rx2")]:
        sim._push(1000, prio, lambda a, n=name: order.append(n))
    while sim._heap:
        t, _p, _c, fn, arg = heapq.heappop(sim._heap)
        fn(arg)
    assert order == ["rate", "tx", "rx", "rx2", "proc"]            # priority, then FIFO


def test_simultaneous_events_end_to_end_reproducible():
    cfg = quiet_cfg(**{"latency.follow_up_ns": 0.0, "latency.rx_processing_ns": 0.0,
                       "latency.delay_resp_ns": 0.0, "intervals.delay_mode": "every_n_sync",
                       "intervals.delay_every_n": 1, "oscillator.initial_offset_ns": 5000.0})
    a, b = simulate(cfg), simulate(cfg)
    assert np.array_equal(a.servo_offset_est_ns, b.servo_offset_est_ns)
    assert np.array_equal(a.delay_est_ns, b.delay_est_ns)
    assert a.counters == b.counters


def test_same_exogenous_disturbances_for_different_controllers():
    cfg = noisy_preset()
    cfg.duration_s = 60.0
    a = simulate(cfg)
    b = simulate(cfg.with_overrides(**{"controller.name": "pi_time_aware",
                                        "controller.params": {"wn": 0.5, "zeta": 1.0}}))
    assert np.array_equal(a.sync_tx_s, b.sync_tx_s)
    assert np.array_equal(a.dreq_tx_s, b.dreq_tx_s)
    assert not np.array_equal(a.servo_cmd_ppb, b.servo_cmd_ppb)
    c = simulate(cfg)
    assert np.array_equal(a.servo_offset_est_ns, c.servo_offset_est_ns)       # seed reproducible
    d = simulate(cfg.with_overrides(seed=2))
    assert not np.array_equal(a.sync_tx_s, d.sync_tx_s)


def test_command_latency_delays_the_rate_effect():
    cfg = quiet_cfg(**{"latency.command_ns": 100e6})
    cfg.oscillator.initial_offset_ns = 10_000.0
    sim = Simulation(cfg)
    sim.run_until(10.0)
    r = sim.result()
    proc = r.servo_t_proc_s[r.servo_action == 0][:5]
    eff = r.rate_t_s[1:6]
    assert np.allclose(eff - proc, 0.1, atol=1e-9)


# ---------------------------------------------------------------- closed-loop analytic model

def test_matches_discrete_closed_loop_recursion_exactly():
    """Ideal actuator, zero latencies, exact delay compensation: the simulated offset sampled at
    the Sync arrivals follows the analytic recursion of the firmware PI to < 1e-3 ns."""
    cfg = quiet_cfg(**{"latency.follow_up_ns": 0.0, "latency.rx_processing_ns": 0.0,
                       "latency.delay_resp_ns": 0.0})
    cfg.duration_s = 40.0
    cfg.oscillator.initial_offset_ns = 100_000.0
    cfg.oscillator.freq_error_ppb = 20_000.0
    cfg.loss.delay_req = 1.0
    sim = Simulation(cfg)
    sim.fw_mean_delay = 1000
    r = sim.run()
    n = r.servo_offset_true_ns.size
    ts = 0.25

    def ratio_of_ppb(u):
        return fwport.scaled_ppm_to_ratio(fwport.ppb_to_scaled_ppm(u))

    # first sample sits at tau_0 = Ts + d: phi(tau_0) = phi0 + eps * tau_0
    phi_first = 100_000.0 + 20_000.0 * (ts + 1e-6)
    ref = baseline_recursion(0.7, 0.3, ts, phi_first, 20_000.0, n, ratio_of_ppb)
    assert np.max(np.abs(ref - r.servo_offset_true_ns)) < 1e-3
    assert np.array_equal(r.servo_offset_est_ns, np.floor(ref))        # firmware works in integer ns


def test_poles_of_baseline_and_dependence_on_sync_interval():
    p1 = baseline_poles(0.7, 0.3, 1.0)
    p025 = baseline_poles(0.7, 0.3, 0.25)
    assert np.all(np.abs(p1) < 1) and np.all(np.abs(p025) < 1)
    assert np.iscomplexobj(p025) and np.abs(p025[0]) > np.abs(p1[0]) * 0.9     # slower per sample
    # damping (continuous equivalent) 0.64 at 1 s, 0.32 at 0.25 s
    assert baseline_poles(0.7, 0.3, 4.0).max() is not None
    assert not np.all(np.abs(baseline_poles(0.7, 0.3, 4.0)) < 1)               # kp*Ts = 2.8 > 2


# ------------------------------------------------------------------------- long runs

def test_time_precision_on_a_million_seconds():
    """1e6 s at the 1.7e18 ns epoch: ns-exact (a float64 epoch has 256 ns resolution there)."""
    assert float(1_700_000_000_000_000_000 + 100) == float(1_700_000_000_000_000_000)   # float64 fails
    cfg = no_control(quiet_cfg(**{"intervals.sync_log": 2, "intervals.delay_log": 4}))
    cfg.duration_s = 1.0e6
    r = simulate(cfg)
    assert r.counters["pairs"] > 249_000
    assert (r.delay_est_ns == 1000).all()
    assert (r.servo_offset_est_ns == 0).all()
    assert r.true_offset_ns(np.array([1.0e6]))[0] == 0.0
    cfg.oscillator.freq_error_ppb = 1.0
    cfg.duration_s = 1.0e6
    cfg.loss.delay_req = 1.0
    sim = Simulation(cfg)
    sim.fw_mean_delay = 1000
    r = sim.run()
    assert abs(r.true_offset_ns(np.array([1.0e6]))[0] - 1.0e6) < 1e-6      # 1 ppb * 1e6 s = 1e6 ns


def test_precision_of_integer_timestamps_at_large_epoch():
    cfg = no_control(quiet_cfg())
    cfg.epoch_ns = 1_700_000_000_123_456_789 // 1_000_000_000 * 1_000_000_000
    cfg.oscillator.initial_offset_ns = 7.0
    r = simulate(cfg)
    assert (r.servo_offset_est_ns == 7).all()


# ------------------------------------------------------------------------------ NXP

def test_nxp_actuator_closed_loop_close_to_ideal_at_100mhz():
    cfg = quiet_cfg()
    cfg.duration_s = 120.0
    cfg.oscillator.initial_offset_ns = 100_000.0
    cfg.oscillator.freq_error_ppb = 20_000.0
    ideal = simulate(cfg)
    nxp = simulate(cfg.with_overrides(**{"actuator.kind": "nxp"}))
    t = np.linspace(100, 120, 400)
    assert np.sqrt(np.mean(nxp.true_offset_ns(t) ** 2)) < 40.0     # quantisation of 10 ns tick
    assert np.sqrt(np.mean(ideal.true_offset_ns(t) ** 2)) < 2.0


def test_nxp_actuator_24mhz_effective_ratio_follows_the_register_model():
    cfg = quiet_cfg(**{"actuator.kind": "nxp", "actuator.clock_hz": 24_000_000})
    cfg.oscillator.freq_error_ppb = 20_000.0
    sim = Simulation(cfg)
    r = sim.run()
    t = sim.act.timer
    assert t.cor != 0
    # the realised average tick matches the register triple (inc, inc_corr, cor)
    avg = t.inc + (t.inc_corr - t.inc) / (t.cor + 1)
    assert abs(sim.act.effective_ratio - avg / (1e9 / 24e6)) < 1e-15
    assert abs(sim.act.effective_delta - (sim.act.effective_ratio - 1.0)) < 1e-15
    # nominal pair makes up the cut fractional tick exactly (41 + 2/3 ns)
    nominal = fwport.NxpTimer(24_000_000)
    assert (nominal.inc, nominal.inc_corr, nominal.cor) == (41, 43, 2) and nominal.effective_delta == 0.0
    assert np.max(np.abs(r.true_offset_ns(np.linspace(30, 59, 300)))) < 50_000.0   # bounded dither


def test_nxp_24mhz_rate_granularity_near_nominal():
    """Finding: with INC = 41 the reachable average rates near ratio 1.0 are ~60 ppm apart
    (INC_CORR - INC <= 86 limits the fractions p/q to p <= 86), so small corrections dither."""
    t = fwport.NxpTimer(24_000_000)
    reachable = set()
    for ppm in np.arange(-150.0, 150.0, 0.25):
        assert t.rate_adjust(1.0 + ppm * 1e-6) == 0
        reachable.add(round(t.effective_delta * 1e6, 3))
    r = np.array(sorted(reachable))
    assert 0.0 in r
    assert not any(0.0 < abs(x) < 60.0 for x in r)                 # dead zone around nominal
    t100 = fwport.NxpTimer(100_000_000)
    reachable100 = set()
    for ppm in np.arange(-150.0, 150.0, 0.25):
        t100.rate_adjust(1.0 + ppm * 1e-6)
        reachable100.add(round(t100.effective_delta * 1e6, 6))
    assert len(reachable100) > 250                                  # fine-grained at a whole tick


def test_ideal_and_nxp_share_the_range_limit():
    cfg = quiet_cfg()
    sim_i = Simulation(cfg)
    sim_n = Simulation(cfg.with_overrides(**{"actuator.kind": "nxp"}))
    for sim in (sim_i, sim_n):
        assert sim.act.adjust(1.0 + 60_000e-6) < 0
        assert sim.act.adjust(1.0 - 49_000e-6) == 0


def test_baseline_instability_at_long_sync_comes_from_delay_estimate_coupling():
    """Finding: with a 2 s Sync the firmware PI is stable if the delay is exactly known (poles of the
    ideal loop inside the unit circle) but diverges when the delay is estimated as the firmware does
    (t2 of the latest Sync with a later t3 under a moving clock)."""
    cfg = quiet_cfg(**{"intervals.sync_log": 1})
    cfg.duration_s = 600.0
    cfg.oscillator.initial_offset_ns = 100_000.0
    cfg.oscillator.freq_error_ppb = 20_000.0
    assert np.all(np.abs(baseline_poles(0.7, 0.3, 2.0)) < 1.0)
    exact = cfg.copy()
    exact.loss.delay_req = 1.0
    sim = Simulation(exact)
    sim.fw_mean_delay = 1000
    r = sim.run()
    assert np.max(np.abs(r.true_offset_ns(np.linspace(500, 600, 400)))) < 5.0
    r2 = simulate(cfg)
    assert r2.counters["range_resets"] > 20 or np.max(np.abs(r2.true_offset_ns(np.linspace(500, 600, 400)))) > 1e4


def _open_loop_delay_error(comp: bool) -> np.ndarray:
    """Delay estimate minus the true delay [ns] of an uncontrolled +20 ppm slave at Sync 2 s (symmetric 1 us)."""
    cfg = no_control(quiet_cfg(**{"intervals.sync_log": 1, "duration_s": 200.0,
                                  "oscillator.freq_error_ppb": 20_000.0, "oscillator.initial_offset_ns": 0.0,
                                  "firmware.delay_rate_comp": comp}))
    r = simulate(cfg)
    return (np.asarray(r.delay_est_ns) - np.asarray(r.delay_true_sample_ns))[3:]


def test_delay_rate_comp_removes_the_bias_of_a_moving_clock_open_loop():
    """The firmware pairs the latest (t1, t2) with a later t3: offset(t2) - offset(t3) = -rate * (t3 - t2) leaks into
    the delay (half of it).  +20 ppm and t3 - t2 up to 2 s: the sample is low by up to 20 us.  The compensation
    (rate from t2 - t1 of consecutive Syncs, no truth) brings it back to the true delay, which also fixes the sign."""
    plain, comp = _open_loop_delay_error(False), _open_loop_delay_error(True)
    assert plain.size > 50 and np.max(plain) <= 1.0 and np.min(plain) < -5_000.0     # low, by up to ~20 us
    assert np.max(np.abs(comp)) < 5.0


def test_delay_rate_comp_fixes_the_baseline_instability_at_sync_2s():
    """Counterpart of the test above: same scenario, flag on -> no range resets, converged."""
    cfg = quiet_cfg(**{"intervals.sync_log": 1, "duration_s": 600.0, "oscillator.initial_offset_ns": 100_000.0,
                       "oscillator.freq_error_ppb": 20_000.0, "firmware.delay_rate_comp": True})
    r = simulate(cfg)
    assert r.counters["range_resets"] == 0 and r.counters["resets"] == 0
    assert np.max(np.abs(r.true_offset_ns(np.linspace(300, 600, 1000)))) < 100.0


def test_delay_rate_comp_off_is_bit_identical_to_the_firmware_model():
    cfg = quiet_cfg(**{"intervals.sync_log": 0, "duration_s": 60.0, "oscillator.initial_offset_ns": 100_000.0,
                       "oscillator.freq_error_ppb": 20_000.0})
    a, b = simulate(cfg), simulate(cfg.with_overrides(**{"firmware.delay_rate_comp": False}))
    for name in ("servo_cmd_ppb", "servo_offset_est_ns", "delay_est_ns", "servo_offset_true_ns"):
        assert np.array_equal(getattr(a, name), getattr(b, name), equal_nan=True), name


def test_delay_rate_comp_can_be_switched_on_live_without_a_stale_history():
    cfg = quiet_cfg(**{"intervals.sync_log": 1, "duration_s": 400.0, "oscillator.initial_offset_ns": 100_000.0,
                       "oscillator.freq_error_ppb": 20_000.0})
    sim = Simulation(cfg)
    sim.run_until(50.0)
    assert sim.fw_prev_x is None and sim.fw_rate_est is None               # nothing kept while the option is off
    sim.update_config({"firmware.delay_rate_comp": True})
    sim.run_until(400.0)
    r = sim.result()
    assert np.max(np.abs(r.true_offset_ns(np.linspace(300, 400, 400)))) < 100.0


# ------------------------------------------------------------------ live parameter changes

@pytest.mark.parametrize("policy", ["keep", "reset", "bumpless"])
def test_live_controller_switch_both_directions(policy):
    cfg = quiet_cfg()
    cfg.duration_s = 80.0
    cfg.oscillator.initial_offset_ns = 100_000.0
    cfg.oscillator.freq_error_ppb = 20_000.0
    sim = Simulation(cfg)
    sim.run_until(30.0)
    pre = sim.ctrl.integral
    sim.update_config({"controller.name": "pi_time_aware", "controller.params.wn": 1.0,
                       "controller.params.zeta": 1.0, "controller.params.sat_ppb": 400000.0,
                       "controller.params.wn_ts_max": 0.35, "controller.params.dt_max_s": 10.0}, policy)
    assert sim.ctrl.name == "pi_time_aware" and set(sim.cfg.controller.params) == set(sim.ctrl.params)
    if policy == "keep":
        assert sim.ctrl.integral == pytest.approx(pre)
    elif policy == "reset":
        assert sim.ctrl.integral == 0.0
    sim.run_until(55.0)
    sim.update_config({"controller.name": "baseline_pi", "controller.params.kp": 0.7,
                       "controller.params.ki": 0.3}, policy)
    assert sim.ctrl.name == "baseline_pi" and sim.cfg.controller.params == {"kp": 0.7, "ki": 0.3}
    sim.run_until(80.0)
    r = sim.result()
    assert r.counters["range_resets"] == 0 and len(r.changes) == 2
    assert abs(r.true_offset_ns(np.array([79.0]))[0]) < 50.0         # kept converging through both switches


def test_live_switch_without_new_params_uses_defaults():
    sim = Simulation(quiet_cfg())
    sim.run_until(5.0)
    sim.update_config({"controller.name": "pi_time_aware"}, "reset")
    assert sim.ctrl.params["wn"] == 1.0
    sim.update_config({"controller.name": "baseline_pi"}, "reset")
    assert sim.ctrl.params == {"kp": 0.7, "ki": 0.3}


def test_live_drift_change_does_not_double_the_drift():
    cfg = no_control(quiet_cfg())
    cfg.duration_s = 100.0
    cfg.oscillator.drift_ppb_per_s = 10.0
    cfg.loss.delay_req = 1.0
    ref = Simulation(cfg)
    ref.fw_mean_delay = 1000
    ref.run_until(100.0)
    sim = Simulation(cfg)
    sim.fw_mean_delay = 1000
    sim.run_until(30.0)
    sim.update_config({"oscillator.drift_ppb_per_s": 10.0 + 1e-12})   # nonzero -> nonzero
    sim.update_config({"oscillator.drift_ppb_per_s": 10.0})
    sim.run_until(100.0)
    a, b = ref.result(), sim.result()
    t = np.array([50.0, 100.0 - 1e-6])
    assert np.allclose(a.true_offset_ns(t), b.true_offset_ns(t), rtol=1e-6)
    # drift 10 ppb/s for 100 s: eps(t) = 10 t ppb -> phi = 5 t^2 ns (1 s steps)
    assert b.true_offset_ns(np.array([99.0]))[0] == pytest.approx(5 * 99.0 ** 2, rel=0.02)


# --------------------------------------------------------- huge initial offsets / forced alignment

def test_phc_starting_at_zero_is_aligned_by_a_step_with_ns_accuracy():
    """Real PHCs start at ~0 while the GM is at the 1.7e18 ns epoch: clock_step must cancel that exactly."""
    cfg = quiet_cfg()
    cfg.duration_s = 120.0
    cfg.oscillator.initial_offset_ns = -float(cfg.epoch_ns)
    cfg.oscillator.freq_error_ppb = 20_000.0
    r = simulate(cfg)
    assert r.counters["steps"] == 1
    t = np.linspace(0, 120, 12001)
    x = r.true_offset_ns(t)
    assert abs(x[0]) > 1e18 and np.all(np.isfinite(x))
    assert np.max(np.abs(x[t > 60])) < 50.0 and abs(x[-1]) < 5.0
    # the servo only restarts after the step *and* a new Delay_Resp (mean_delay was cleared)
    step_t = [e[0] for e in r.events if e[1] == "step"][0]
    first_pi = r.servo_t_proc_s[r.servo_action == 0][0]
    assert first_pi > step_t
    m = __import__("ptpsim.metrics", fromlist=["x"]).compute_metrics(r)
    assert not m["true_offset"]["diverged"] and m["servo_start_s"] > step_t


def test_step_residual_is_exact_for_integer_offsets_and_includes_the_set_latency():
    cfg = quiet_cfg()
    cfg.duration_s = 40.0
    cfg.oscillator.initial_offset_ns = 3.0e9 + 123.0
    r = simulate(cfg)
    first_pi = r.servo_t_proc_s[r.servo_action == 0][0]
    assert r.counters["steps"] == 1
    cfg2 = cfg.with_overrides(**{"latency.step_ns": 5000.0})
    r2 = simulate(cfg2)
    # same scenario, 5 us read->set latency: the clock is 5 us behind right after the step
    step_t = [e[0] for e in r2.events if e[1] == "step"][0]
    assert r2.true_offset_ns(np.array([step_t + 1e-6]))[0] == pytest.approx(-5000.0, abs=1.5)
    assert abs(r.true_offset_ns(np.array([step_t + 1e-6]))[0]) < 1.5


def test_clock_big_offset_arithmetic():
    from ptpsim.clock import SlaveClock
    c = SlaveClock(-1.7e18, 20e-6, 0)
    assert c.read_ps(0) == int(-1.7e18) * 1000
    c.step(1_000_000_000_000, int(1.7e18) + 7)
    assert c.phi_ns(1_000_000_000_000) == pytest.approx(7.0 + 20e-6 * 1e9, abs=1e-9)
    assert c._big[-1] == 0
