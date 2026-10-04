# SPDX-License-Identifier: Apache-2.0
"""Discrete-event closed-loop simulation of the PTPv2 time receiver.

Physical simulation time = GM time, integer picoseconds.  No fixed time step, no sleeps.

Actors
------
* **GM** (rate constant, no drift): emits Sync/Follow_Up on a nominal grid with send jitter,
  answers Delay_Req with Delay_Resp.  ``t1``/``t4`` are GM hardware timestamps.
* **Network**: per-direction base delay (asymmetry), random path variation, loss.
* **Slave firmware** (``_Firmware`` section below): a line-by-line behavioural port of
  ``port.c``/``clock.c`` of the Zephyr PTP library (see docs/firmware_reconstruction.md):
  Sync/Follow_Up pairing slot, Delay_Req list, "latest t1/t2" delay estimate,
  lock/outlier/step logic, absolute-ppb PI command, actuator rejection => servo reset.
* **Slave clock**: PHC disciplined by an actuator (ideal or NXP ENET timer); ``t2``/``t3``
  are read from it at the physical instant of the event.

The controller only ever sees :class:`ptpsim.controllers.ServoSample`.
"""
from __future__ import annotations

import heapq
import math
import time as _time
from dataclasses import dataclass, field
from typing import Any, Callable

import numpy as np

from . import fwport
from .actuators import make_actuator
from .clock import PS_PER_NS, SlaveClock
from .config import SimConfig, set_path
from .controllers import REGISTRY, Controller, ServoSample, make_controller
from .rng import Disturbances

PS_PER_S = 1_000_000_000_000
# event priorities at identical timestamps (deterministic order, then FIFO by creation)
P_RATE, P_TX, P_RX, P_PROC = 0, 1, 2, 3

U16 = 0xFFFF


def _ps(ns: float) -> int:
    return int(round(ns * PS_PER_NS))


def _trunc_div2(n: int) -> int:
    """C ``int64 / 2LL`` (truncation toward zero)."""
    return n // 2 if n >= 0 else -((-n) // 2)


def interval_ps(log_n: int) -> int:
    """1 s * 2^n in ps (exact for n >= -12)."""
    return PS_PER_S >> -log_n if log_n < 0 else PS_PER_S << log_n


# --------------------------------------------------------------------------- results

@dataclass
class SimResult:
    cfg: SimConfig
    t_end_s: float
    wall_s: float
    n_events: int
    # servo samples (one per completed Sync/Follow_Up pair that reached the offset computation)
    servo_t_proc_s: np.ndarray
    servo_t_sample_s: np.ndarray          # physical instant of the t2 acquisition
    servo_offset_est_ns: np.ndarray
    servo_offset_true_ns: np.ndarray      # true offset at the t2 instant
    servo_offset_true_proc_ns: np.ndarray
    servo_cmd_ppb: np.ndarray             # NaN when no command was issued
    servo_integral: np.ndarray
    servo_action: np.ndarray              # 0 pi, 1 step, 2 outlier rejected, 3 reset (range/driver)
    # delay samples
    delay_t_proc_s: np.ndarray
    delay_est_ns: np.ndarray
    delay_true_sample_ns: np.ndarray      # realised (d_ms_resp + d_sm_req) / 2 of that exchange
    delay_true_nominal_ns: float
    delay_t2_phys_s: np.ndarray           # physical instant of the t2 used by that estimate
    delay_t3_phys_s: np.ndarray           # physical instant of the Delay_Req TX (t3)
    sync_tx_s: np.ndarray                 # physical instants of every Sync emission (all, incl. lost)
    dreq_tx_s: np.ndarray                 # physical instants of every Delay_Req emission
    # rate diagnostics: instants at which the clock rate changed
    rate_t_s: np.ndarray
    rate_cmd_ppb: np.ndarray              # last commanded ppb (what the controller asked)
    rate_eff_ppb: np.ndarray              # actual clock rate error vs GM (includes oscillator)
    # clock + events
    clock: SlaveClock
    events: list[tuple[float, str, str]]
    changes: list[tuple[float, str]]       # live parameter changes (t_s, text)
    counters: dict[str, int]

    def true_offset_ns(self, t_s: np.ndarray) -> np.ndarray:
        return self.clock.sample_ns(np.round(np.asarray(t_s) * PS_PER_S).astype(np.int64))


# --------------------------------------------------------------------------- engine

class Simulation:
    def __init__(self, cfg: SimConfig, controller: Controller | None = None):
        self.cfg = cfg.copy()
        c = self.cfg
        self.dist = Disturbances(c.seed)
        self.ctrl = controller or make_controller(c.controller.name, c.controller.params)
        self.act = make_actuator(c.actuator)
        self.E = int(c.epoch_ns)
        self.now = 0
        self._heap: list[tuple[int, int, int, Callable, Any]] = []
        self._cnt = 0
        self.n_events = 0
        self._wall = 0.0

        # slave clock: true oscillator error * actuator state
        self.eps_ppb = c.oscillator.freq_error_ppb
        self._applied_delta = self.act.effective_delta
        self.clock = SlaveClock(c.oscillator.initial_offset_ns, self._slope(), 0)

        # GM / sequences
        self.sync_k = 0
        self.sync_nominal = 0
        self.dreq_j = 0
        self.dreq_nominal = 0
        self._dreq_gen = 0       # generation counter to cancel a stale timer
        self.osc_i = 0
        self._osc_gen = 0        # generation counter: a live change of drift/walk restarts the chain

        # ---- slave firmware state (clock.c / port.c) -------------------------------------
        self.fw_slot: tuple | None = None        # port->last_sync_fup
        self.fw_t1 = 0                           # ptp_clk.timestamp.t1/t2 (ns, 0 = unset)
        self.fw_t2 = 0
        self.fw_t2_phys = 0                      # physical instant of that t2 (truth, for analysis only)
        self.fw_mean_delay = 0                   # ns (C stores ns<<16; 0 == "no delay yet")
        self.fw_delay_list: dict[int, dict] = {}  # port->delay_req_list (seq16 -> entry)
        self.fw_lock_samples = 0
        self.fw_outliers = 0
        self.fw_locked = False
        self.fw_delay_log = c.intervals.delay_log   # port_ds.log_min_delay_req_interval
        self.fw_pairs_since_dreq = 0
        self.fw_prev_t1c: int | None = None
        self.fw_servo_index = 0
        self._last_cmd_ppb = 0.0
        self._dreq_truth: dict[int, int] = {}     # realised slave->GM delay per Delay_Req (truth)

        # ---- recording ------------------------------------------------------------------
        self._s_t_proc: list[float] = []
        self._s_t_samp: list[float] = []
        self._s_off: list[int] = []
        self._s_true: list[float] = []
        self._s_true_p: list[float] = []
        self._s_cmd: list[float] = []
        self._s_int: list[float] = []
        self._s_act: list[int] = []
        self._d_t: list[float] = []
        self._d_est: list[int] = []
        self._d_true: list[float] = []
        self._d_t2: list[float] = []
        self._d_t3: list[float] = []
        self._sync_tx: list[float] = []
        self._dreq_tx: list[float] = []
        self._r_t: list[float] = [0.0]
        self._r_cmd: list[float] = [0.0]
        self._r_eff: list[float] = [self.clock.slope * 1e9]
        self.events: list[tuple[float, str, str]] = []
        self.changes: list[tuple[float, str]] = []
        self.counters = {"sync_tx": 0, "sync_rx": 0, "fup_rx": 0, "dreq_tx": 0, "dresp_rx": 0,
                         "lost": 0, "pairs": 0, "steps": 0, "outliers": 0, "resets": 0,
                         "range_resets": 0, "saturated": 0, "delay_samples": 0, "stale_pairs": 0}

        self._schedule_start()

    # ------------------------------------------------------------------ plumbing
    def _push(self, t_ps: int, prio: int, fn: Callable, arg: Any = None) -> None:
        self._cnt += 1
        heapq.heappush(self._heap, (t_ps, prio, self._cnt, fn, arg))

    def _slope(self) -> float:
        """Clock rate error vs physical time: (1 + eps)(1 + d) - 1 written without cancellation."""
        e = self.eps_ppb * 1e-9
        d = self._applied_delta
        return e + d + e * d

    def _log(self, kind: str, text: str = "") -> None:
        if len(self.events) < 200_000:
            self.events.append((self.now / PS_PER_S, kind, text))

    def run_until(self, t_end_s: float) -> None:
        t_end = int(round(t_end_s * PS_PER_S))
        t0 = _time.perf_counter()
        heap = self._heap
        pop = heapq.heappop
        n = 0
        while heap and heap[0][0] <= t_end:
            t, _p, _c, fn, arg = pop(heap)
            self.now = t
            fn(arg)
            n += 1
        self.now = max(self.now, t_end)
        self.n_events += n
        self._wall += _time.perf_counter() - t0

    def run(self) -> SimResult:
        self.run_until(self.cfg.duration_s)
        return self.result()

    # ------------------------------------------------------------------ timestamps
    def _gm_ts(self, t_ps: int, stream: str, idx: int) -> int:
        ts = self.cfg.timestamps
        x = t_ps + _ps(self.dist.normal_ns(stream, idx, ts.gm_noise_sigma_ns))
        q = _ps(ts.gm_quantum_ns)
        if q > 0:
            x = (x // q) * q
        return self.E + x // PS_PER_NS

    def _slave_quantum_ps(self) -> int:
        q = self.cfg.timestamps.slave_quantum_ns
        if q is None:
            q = self.act.timestamp_quantum_ns
        return _ps(q)

    def _slave_ts(self, t_ps: int, stream: str, idx: int) -> int:
        ts = self.cfg.timestamps
        x = self.clock.read_ps(t_ps, self.dist.normal_ns(stream, idx, ts.slave_noise_sigma_ns),
                               self._slave_quantum_ps())
        return self.E + x // PS_PER_NS

    def _phc_read_ns(self, t_ps: int) -> int:
        return self.E + self.clock.read_ps(t_ps) // PS_PER_NS

    # ------------------------------------------------------------------ network helpers
    def _net_ps(self, direction: str, stream: str, idx: int) -> int:
        n = self.cfg.network
        if direction == "ms":
            base, spec = n.delay_ms_ns, n.jitter_ms
        else:
            base, spec = n.delay_sm_ns, n.jitter_sm
        return max(0, _ps(base + self.dist.jitter_ns(stream, idx, spec)))

    def _lost(self, kind: str, idx: int) -> bool:
        p = getattr(self.cfg.loss, kind)
        if self.dist.lost("loss." + kind, idx, p):
            self.counters["lost"] += 1
            return True
        return False

    # ------------------------------------------------------------------ start
    def _schedule_start(self) -> None:
        c = self.cfg
        # GM: first Sync one interval after t = 0
        self.sync_nominal = interval_ps(c.intervals.sync_log)
        self._push_sync_tx()
        # slave Delay_Req timer (interval mode): first expiry uniform in (0, 2 * 2^n]
        if c.intervals.delay_mode == "interval":
            self._arm_first_delay_timer()
        if self._osc_dynamic():
            self._push(_ps(c.oscillator.update_period_s * 1e9), P_RATE, self._on_osc_update, self._osc_gen)

    def _osc_dynamic(self) -> bool:
        o = self.cfg.oscillator
        return o.drift_ppb_per_s != 0.0 or o.walk_ppb_per_sqrt_s != 0.0

    # ------------------------------------------------------------------ GM
    def _push_sync_tx(self) -> None:
        k = self.sync_k
        j = _ps(self.dist.jitter_ns("sync.tx", k, self.cfg.tx_jitter.sync))
        self._push(max(self.now, self.sync_nominal + j), P_TX, self._on_gm_sync_tx, (k, self.sync_nominal))

    def _on_gm_sync_tx(self, arg) -> None:
        k, nominal = arg
        t = self.now
        self.counters["sync_tx"] += 1
        self._sync_tx.append(t / PS_PER_S)
        t1_ts = self._gm_ts(t, "t1.noise", k)
        # next Sync on the nominal grid (interval read now: live changes apply from here)
        self.sync_k = k + 1
        self.sync_nominal = nominal + interval_ps(self.cfg.intervals.sync_log)
        self._push_sync_tx()
        # Sync on the wire
        if not self._lost("sync", k):
            self._push(t + self._net_ps("ms", "sync.net", k), P_RX, self._on_slave_sync_rx, (k, t))
        # Follow_Up carries t1 (TX timestamp is available only after the Sync was sent)
        lat = _ps(self.cfg.latency.follow_up_ns) + _ps(self.dist.jitter_ns("fup.tx", k, self.cfg.tx_jitter.follow_up))
        self._push(t + max(0, lat), P_TX, self._on_gm_fup_tx, (k, t1_ts))

    def _on_gm_fup_tx(self, arg) -> None:
        k, t1_ts = arg
        if self._lost("follow_up", k):
            return
        n = self.cfg.network
        t_arr = self.now + self._net_ps("ms", "fup.net", k)
        self._push(t_arr + _ps(self.cfg.latency.rx_processing_ns), P_PROC, self._on_slave_fup_proc,
                   (k, t1_ts, int(round(n.correction_follow_up_ns * 65536))))

    def _on_gm_dreq_rx(self, arg) -> None:
        j, = arg
        t4_ts = self._gm_ts(self.now, "t4.noise", j)
        lat = _ps(self.cfg.latency.delay_resp_ns) + _ps(self.dist.jitter_ns("dresp.tx", j, self.cfg.tx_jitter.delay_resp))
        self._push(self.now + max(0, lat), P_TX, self._on_gm_dresp_tx, (j, t4_ts))

    def _on_gm_dresp_tx(self, arg) -> None:
        j, t4_ts = arg
        if self._lost("delay_resp", j):
            return
        d_ms = self._net_ps("ms", "dresp.net", j)
        d_sm = self._dreq_truth.pop(j, 0)
        t_arr = self.now + d_ms
        n = self.cfg.network
        self._push(t_arr + _ps(self.cfg.latency.rx_processing_ns), P_PROC, self._on_slave_dresp_proc,
                   (j, t4_ts, int(round(n.correction_delay_resp_ns * 65536)),
                    self.cfg.intervals.delay_log, (d_ms + d_sm) / 2.0 / PS_PER_NS))

    # ------------------------------------------------------------------ slave RX (hardware stamp)
    def _on_slave_sync_rx(self, arg) -> None:
        k, _t_tx = arg
        self.counters["sync_rx"] += 1
        t2_ts = self._slave_ts(self.now, "t2.noise", k)     # HW RX timestamp at the arrival instant
        n = self.cfg.network
        corr = int(round(n.correction_sync_ns * 65536))
        self._push(self.now + _ps(self.cfg.latency.rx_processing_ns), P_PROC, self._on_slave_sync_proc,
                   (k, t2_ts, True, corr, self.now))

    # ------------------------------------------------------------------ firmware: Sync / Follow_Up
    def _on_slave_sync_proc(self, arg) -> None:
        k, t2_ts, valid, corr, t_arr = arg
        if not valid:                      # port_sync_rx_timestamp_valid: dropped
            return
        # msg->header.correction += port_ds.delay_asymmetry
        corr += int(round(self.cfg.network.delay_asymmetry_ns * 65536))
        self._ooo_handle(("S", k & U16, t2_ts, corr, t_arr))

    def _on_slave_fup_proc(self, arg) -> None:
        k, t1_ts, corr = arg
        self.counters["fup_rx"] += 1
        self._ooo_handle(("F", k & U16, t1_ts, corr, 0))

    def _ooo_handle(self, msg: tuple) -> None:
        """port_sync_fup_ooo_handle: single slot, pairs by sequence id in either order."""
        last = self.fw_slot
        if last is None:
            self.fw_slot = msg
            return
        if last[0] == "S" and msg[0] == "F" and msg[1] == last[1]:
            self._port_synchronize(t2=last[2], t1=msg[2], corr1=last[3], corr2=msg[3], t_arr=last[4])
            self.fw_slot = None
        elif last[0] == "F" and msg[0] == "S" and msg[1] == last[1]:
            self._port_synchronize(t2=msg[2], t1=last[2], corr1=msg[3], corr2=last[3], t_arr=msg[4])
            self.fw_slot = None
        else:
            if last[0] == "S":
                self.counters["stale_pairs"] += 1   # an unpaired Sync was overwritten
            self.fw_slot = msg

    def _port_synchronize(self, t2: int, t1: int, corr1: int, corr2: int, t_arr: int) -> None:
        t1c = t1 + (corr1 >> 16) + (corr2 >> 16)
        self.counters["pairs"] += 1
        # one servo-period bookkeeping for the controller (data the firmware has: t1 values)
        nominal = interval_ps(self.cfg.intervals.sync_log) / PS_PER_S
        dt = nominal if self.fw_prev_t1c is None else (t1c - self.fw_prev_t1c) / 1e9
        self.fw_prev_t1c = t1c
        self._clock_synchronize(t2, t1c, dt, nominal, t_arr)
        # non-firmware "every N Sync" trigger of the Delay_Req
        c = self.cfg.intervals
        if c.delay_mode == "every_n_sync":
            self.fw_pairs_since_dreq += 1
            if self.fw_pairs_since_dreq >= max(1, int(c.delay_every_n)):
                self.fw_pairs_since_dreq = 0
                self._dreq_trigger_now()

    # clock.c: clock_synchronize_with_delay -----------------------------------------------
    def _clock_synchronize(self, ingress: int, egress: int, dt: float, nominal: float, t_arr: int) -> None:
        self.fw_t1 = egress
        self.fw_t2 = ingress
        self.fw_t2_phys = t_arr
        if self.fw_mean_delay == 0:
            return
        delay = self.fw_mean_delay
        offset = (ingress - egress) - delay
        fwc = self.cfg.firmware
        if offset > fwc.step_threshold_ns or offset < -fwc.step_threshold_ns:
            self._clock_step(offset, t_arr)
            return
        self._clock_adjust_rate(offset, dt, nominal, t_arr)

    def _true_phi(self, t_ps: int) -> float:
        return self.clock.phi_ns(t_ps)

    def _record_sample(self, t_arr: int, offset: int, cmd: float, action: int) -> None:
        self._s_t_proc.append(self.now / PS_PER_S)
        self._s_t_samp.append(t_arr / PS_PER_S)
        self._s_off.append(offset)
        self._s_true.append(self.clock.phi_hist_ns(t_arr))
        self._s_true_p.append(self.clock.phi_ns(self.now))
        self._s_cmd.append(cmd)
        self._s_int.append(self.ctrl.integral)
        self._s_act.append(action)

    def _clock_step(self, offset: int, t_arr: int) -> None:
        """clock_step: target = phc_now - offset; clears timestamps, mean_delay; resets servo."""
        self._record_sample(t_arr, offset, float("nan"), 1)
        self.counters["steps"] += 1
        self._log("step", f"offset={offset} ns")
        self.clock.step(self.now, -int(offset))                 # exact: cancels a huge offset to the ns
        lat = self.cfg.latency.step_ns
        if lat:
            self.clock.step(self.now, -float(lat))              # target was read `lat` before the set took effect
        self.fw_t1 = self.fw_t2 = 0           # memset(&ptp_clk.timestamp, 0)
        self.fw_mean_delay = 0
        self._servo_reset("step")
        # NOTE: fw_delay_list is NOT cleared (as in the firmware): a Delay_Resp of a request sent
        # before the step will pair a pre-step t3 with a post-step t2.

    def _servo_reset(self, why: str) -> None:
        """clock_servo_reset: PI integral = 0, lock cleared, rate back to nominal (adjust_rate(0))."""
        self.counters["resets"] += 1
        self.ctrl.reset()
        self.fw_servo_index = 0
        self.fw_lock_samples = 0
        self.fw_outliers = 0
        self.fw_locked = False
        self._last_cmd_ppb = 0.0
        if self.act.adjust(1.0) == 0:
            self._schedule_rate_effect()
        self._log("servo_reset", why)

    def _clock_adjust_rate(self, offset: int, dt: float, nominal: float, t_arr: int) -> None:
        fwc = self.cfg.firmware
        if self.fw_locked and abs(offset) > fwc.outlier_ns:
            self.fw_outliers += 1
            self.counters["outliers"] += 1
            self._record_sample(t_arr, offset, float("nan"), 2)
            self._log("outlier_rejected", f"offset={offset} ns ({self.fw_outliers}/{fwc.outlier_samples})")
            if self.fw_outliers >= fwc.outlier_samples:
                self._servo_reset("outliers")
            return
        self.fw_outliers = 0
        sample = ServoSample(offset_ns=offset, sync_interval_s=dt, nominal_interval_s=nominal,
                             index=self.fw_servo_index)
        self.fw_servo_index += 1
        sat_before = getattr(self.ctrl, "saturated_count", 0)
        ppb = self.ctrl.update(sample)
        if getattr(self.ctrl, "saturated_count", 0) != sat_before:
            self.counters["saturated"] += 1
        scaled = fwport.ppb_to_scaled_ppm(ppb)
        if scaled is None:
            self._record_sample(t_arr, offset, ppb, 3)
            self.counters["range_resets"] += 1
            self._servo_reset("ppb out of range")
            return
        ratio = fwport.scaled_ppm_to_ratio(scaled)
        if self.act.adjust(ratio) < 0:
            self._record_sample(t_arr, offset, ppb, 3)
            self.counters["range_resets"] += 1
            self._servo_reset("actuator rejected ratio")
            return
        self._last_cmd_ppb = ppb
        self._record_sample(t_arr, offset, ppb, 0)
        self._schedule_rate_effect()
        # clock_servo_update_lock
        if abs(offset) > fwc.lock_offset_ns:
            self.fw_lock_samples = 0
            return
        if self.fw_lock_samples < fwc.lock_samples:
            self.fw_lock_samples += 1
        if not self.fw_locked and self.fw_lock_samples >= fwc.lock_samples:
            self.fw_locked = True
            self._log("locked", "")

    # ------------------------------------------------------------------ rate effect / oscillator
    def _schedule_rate_effect(self) -> None:
        lat = _ps(self.cfg.latency.command_ns)
        delta = self.act.effective_delta
        if lat <= 0:
            self._on_rate_effect(delta)
        else:
            self._push(self.now + lat, P_RATE, self._on_rate_effect, delta)

    def _on_rate_effect(self, delta: float) -> None:
        self._applied_delta = delta
        self._apply_slope()

    def _apply_slope(self) -> None:
        slope = self._slope()
        self.clock.set_slope(self.now, slope)
        self._r_t.append(self.now / PS_PER_S)
        self._r_cmd.append(self._last_cmd_ppb)
        self._r_eff.append(slope * 1e9)

    def _on_osc_update(self, gen) -> None:
        if gen != self._osc_gen:
            return                                    # superseded by a live change
        o = self.cfg.oscillator
        dt = o.update_period_s
        self.eps_ppb += o.drift_ppb_per_s * dt
        if o.walk_ppb_per_sqrt_s != 0.0:
            self.eps_ppb += o.walk_ppb_per_sqrt_s * math.sqrt(dt) * self.dist.stream("osc.walk").z(self.osc_i)
        self.osc_i += 1
        self._apply_slope()
        if self._osc_dynamic():
            self._push(self.now + _ps(dt * 1e9), P_RATE, self._on_osc_update, self._osc_gen)

    # ------------------------------------------------------------------ Delay_Req (slave)
    def _timer_interval_ps(self, log_n: int) -> int:
        """Physical duration of a k_timer period: the kernel clock is independent of the PHC."""
        nominal = interval_ps(log_n)
        return int(round(nominal / (1.0 + self.cfg.oscillator.timer_error_ppb * 1e-9)))

    def _arm_first_delay_timer(self) -> None:
        # port_timer_set_timeout_random(timer, 0, 2, log): uniform in (0, 2 * 2^n] s
        u = self.dist.uniform("dreq.first", 0)
        frac = (int(u * 32768) + 1) / 32768.0
        first = int(round(2 * self._timer_interval_ps(self.fw_delay_log) * frac))
        self.dreq_nominal = first
        self._push_dreq_timer()

    def _push_dreq_timer(self) -> None:
        j = self.dreq_j
        jit = _ps(self.dist.jitter_ns("dreq.tx", j, self.cfg.tx_jitter.delay_req))
        self._push(max(self.now, self.dreq_nominal + jit), P_TX, self._on_dreq_timer,
                   (self._dreq_gen, self.dreq_nominal))

    def _on_dreq_timer(self, arg) -> None:
        gen, nominal = arg
        if gen != self._dreq_gen or self.cfg.intervals.delay_mode != "interval":
            return                                    # cancelled by a live mode change
        # port_delay_req_cleanup: drop requests older than PORT_DELAY_REQ_CLEAR_TO
        limit = self.cfg.firmware.delay_req_clear_ns * PS_PER_NS
        for seq in [s for s, e in self.fw_delay_list.items() if self.now - e["t_phys"] >= limit]:
            del self.fw_delay_list[seq]
        # re-arm (k_timer restarted at handling time with the CURRENT interval)
        base = self.now if self.cfg.intervals.delay_rearm_from_handling else nominal
        self.dreq_nominal = base + self._timer_interval_ps(self.fw_delay_log)
        self._dreq_tx_now()
        self._push_dreq_timer()

    def _dreq_trigger_now(self) -> None:
        j = self.dreq_j
        jit = _ps(self.dist.jitter_ns("dreq.tx", j, self.cfg.tx_jitter.delay_req))
        self._push(max(self.now, self.now + jit), P_TX, self._on_dreq_event, (self._dreq_gen,))

    def _on_dreq_event(self, arg) -> None:
        gen, = arg
        if gen != self._dreq_gen or self.cfg.intervals.delay_mode != "every_n_sync":
            return
        limit = self.cfg.firmware.delay_req_clear_ns * PS_PER_NS
        for seq in [s for s, e in self.fw_delay_list.items() if self.now - e["t_phys"] >= limit]:
            del self.fw_delay_list[seq]
        self._dreq_tx_now()

    def _dreq_tx_now(self) -> None:
        """port_delay_req_msg_transmit: HW TX timestamp t3 from the (disciplined) PHC."""
        j = self.dreq_j
        self.dreq_j += 1
        self.counters["dreq_tx"] += 1
        t = self.now
        self._dreq_tx.append(t / PS_PER_S)
        seq = j & U16
        entry = {"t_phys": t, "t3": self._phc_read_ns(t),    # software fallback read at creation
                 "t3_hw": self._slave_ts(t, "t3.noise", j), "hw": False}
        self.fw_delay_list[seq] = entry
        cb = _ps(self.cfg.latency.tx_timestamp_cb_ns)
        if cb <= 0:
            entry["t3"], entry["hw"] = entry["t3_hw"], True
        else:
            self._push(t + cb, P_PROC, self._on_dreq_ts_cb, seq)
        if not self._lost("delay_req", j):
            d = self._net_ps("sm", "dreq.net", j)
            self._dreq_truth[j] = d
            self._push(t + d, P_RX, self._on_gm_dreq_rx, (j,))

    def _on_dreq_ts_cb(self, seq: int) -> None:
        e = self.fw_delay_list.get(seq)
        if e is not None:
            e["t3"], e["hw"] = e["t3_hw"], True

    def _on_slave_dresp_proc(self, arg) -> None:
        j, t4_ts, corr, log_iv, true_delay_sample = arg
        self.counters["dresp_rx"] += 1
        e = self.fw_delay_list.get(j & U16)
        if e is None:
            return                                    # not for a known request: ignored
        t3 = e["t3"]
        t4c = t4_ts - (corr >> 16)
        self._ptp_clock_delay(t3, t4c, true_delay_sample, e["t_phys"])
        del self.fw_delay_list[j & U16]
        # logMessageInterval of the Delay_Resp overrides log_min_delay_req_interval
        if -10 <= log_iv <= 22:
            self.fw_delay_log = log_iv

    def _ptp_clock_delay(self, egress: int, ingress: int, truth: float, t3_phys: int) -> None:
        """ptp_clock_delay: uses the LATEST t1/t2 held by the clock, no filtering."""
        if self.fw_t1 == 0 or self.fw_t2 == 0:
            return
        delay = _trunc_div2((self.fw_t2 - egress) + (ingress - self.fw_t1))
        if abs(delay) > 1_000_000_000:
            self._log("delay_ignored", f"{delay} ns")
            return
        self.fw_mean_delay = delay
        self.counters["delay_samples"] += 1
        self._d_t.append(self.now / PS_PER_S)
        self._d_est.append(delay)
        self._d_true.append(truth)
        self._d_t2.append(self.fw_t2_phys / PS_PER_S)
        self._d_t3.append(t3_phys / PS_PER_S)

    # ------------------------------------------------------------------ live parameter changes
    def update_config(self, overrides: dict[str, Any], controller_policy: str = "keep") -> None:
        """Apply parameter changes from *now* on (live mode).  Dotted keys, e.g.
        ``{"controller.params.kp": 0.5, "intervals.sync_log": -3}``.  The integrator policy for
        gain changes is explicit (``keep`` | ``reset`` | ``bumpless``); nothing is reset silently."""
        c = self.cfg
        old_mode = c.intervals.delay_mode
        old_name = c.controller.name
        new_name = overrides.get("controller.name", old_name)
        if new_name != old_name:
            c.controller.params = {}                  # parameters of the old class do not apply
        ctrl_params: dict[str, float] = {}
        for k, v in overrides.items():
            if k.startswith("controller.params."):
                ctrl_params[k.split(".", 2)[2]] = v
            set_path(c, k, v)
        notes = []
        if new_name != old_name:
            old = self.ctrl
            cls = REGISTRY[new_name]
            params = {k: v for k, v in c.controller.params.items() if k in cls.PARAMS}
            c.controller.params = dict(params)
            self.ctrl = make_controller(new_name, params)
            self.ctrl.last_error, self.ctrl.last_output = old.last_error, old.last_output
            if controller_policy == "keep":               # same physical quantity (ppb): carry it over
                self.ctrl._set_integral(old.integral)
            elif controller_policy == "bumpless":         # output continuous w.r.t. the last error
                self.ctrl._set_integral(old.last_output - self.ctrl._proportional(old.last_error))
            notes.append(f"controller {old_name} -> {new_name} ({controller_policy})")
        elif ctrl_params:
            self.ctrl.set_params(ctrl_params, controller_policy)
            notes.append(f"gains {ctrl_params} ({controller_policy})")
        if "actuator.kind" in overrides or "actuator.clock_hz" in overrides or "actuator.max_ratio_ppm" in overrides:
            self.act = make_actuator(c.actuator)
            self._applied_delta = self.act.effective_delta
            self._apply_slope()
            notes.append("actuator re-initialised at its nominal state")
        if "oscillator.freq_error_ppb" in overrides:
            self.eps_ppb = c.oscillator.freq_error_ppb
            self._apply_slope()
        if any(k.startswith("oscillator.") and k.split(".")[1] in ("drift_ppb_per_s", "walk_ppb_per_sqrt_s",
                                                                    "update_period_s") for k in overrides):
            self._osc_gen += 1                        # cancel the running chain, start a new one if needed
            if self._osc_dynamic():
                self._push(self.now + _ps(c.oscillator.update_period_s * 1e9), P_RATE, self._on_osc_update,
                           self._osc_gen)
        if "intervals.delay_log" in overrides:
            notes.append(f"GM advertises Delay_Req 2^{c.intervals.delay_log}")
        if c.intervals.delay_mode != old_mode:
            self._dreq_gen += 1                       # cancel the pending timer / trigger
            self.fw_pairs_since_dreq = 0
            if c.intervals.delay_mode == "interval":
                self.dreq_nominal = self.now + self._timer_interval_ps(self.fw_delay_log)
                self._push_dreq_timer()
            notes.append(f"delay mode -> {c.intervals.delay_mode}")
        self.changes.append((self.now / PS_PER_S, "; ".join(notes) or ", ".join(f"{k}={v}" for k, v in overrides.items())))

    # ------------------------------------------------------------------ results
    def result(self) -> SimResult:
        n = self.cfg.network
        arr = np.array
        return SimResult(
            cfg=self.cfg.copy(), t_end_s=self.now / PS_PER_S, wall_s=self._wall, n_events=self.n_events,
            servo_t_proc_s=arr(self._s_t_proc), servo_t_sample_s=arr(self._s_t_samp),
            servo_offset_est_ns=arr(self._s_off, dtype=np.float64),
            servo_offset_true_ns=arr(self._s_true), servo_offset_true_proc_ns=arr(self._s_true_p),
            servo_cmd_ppb=arr(self._s_cmd), servo_integral=arr(self._s_int),
            servo_action=arr(self._s_act, dtype=np.int8),
            delay_t_proc_s=arr(self._d_t), delay_est_ns=arr(self._d_est, dtype=np.float64),
            delay_true_sample_ns=arr(self._d_true),
            delay_true_nominal_ns=(n.delay_ms_ns + n.delay_sm_ns) / 2.0,
            delay_t2_phys_s=arr(self._d_t2), delay_t3_phys_s=arr(self._d_t3),
            sync_tx_s=arr(self._sync_tx), dreq_tx_s=arr(self._dreq_tx),
            rate_t_s=arr(self._r_t), rate_cmd_ppb=arr(self._r_cmd), rate_eff_ppb=arr(self._r_eff),
            clock=self.clock, events=list(self.events), changes=list(self.changes),
            counters=dict(self.counters))


def simulate(cfg: SimConfig, controller: Controller | None = None) -> SimResult:
    """Run a full trajectory from the initial conditions (exploration mode)."""
    return Simulation(cfg, controller).run()
