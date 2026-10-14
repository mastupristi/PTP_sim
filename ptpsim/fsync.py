# SPDX-License-Identifier: Apache-2.0
"""Architecture A (``fsync.mode = cascade``): FSYNC from the Audio PLL, disciplined by a second servo on the PTP clock.

The follower's FSYNC is ``VCO / fsync_div`` of the Audio PLL (48 kHz).  Its rising edges are timestamped by the PTP
timer (the PHC, disciplined by the PTP servo); every ``decimation`` edges (48 = 1 ms, 480 = 10 ms) a second servo compares
the captured timestamp with the PHC grid and writes a new ``NUM`` (``ptpsim/pll.py``).  FSYNC follows the PHC, which
follows the GM.

The coupling is one-way -- the PHC does not depend on FSYNC -- so this runs *after* the PTP loop on the PHC history
(``advance`` is incremental: the live mode calls it chunk by chunk).  It never enters the event heap: one update is a
few arithmetic operations on the current segments, so 300 000 updates (300 s at 1 ms) stay a plain loop.

Model
-----
* FSYNC is a :class:`SlaveClock` reading ``t + psi(t)``: an edge whenever it reads ``k * P`` (``P = 1e12 / f`` ps, an exact
  fraction).  Its rate error is ``(1 + eps(t)) (1 + d_pll) - 1``: ``eps`` is the crystal error (the same crystal feeds
  every PLL, so it is the PHC's oscillator error too), ``d_pll = (NUM - NUM0) / n0``.  It is never stepped.
* Capture: the PHC reading at the edge instant, floored to the timer tick (as every slave timestamp), plus optional
  noise (``capture_noise_ns``, own indexed random stream ``fsync.cap``), in whole ns.
* Phase detector: ``wrapped`` (default) -- distance of the timestamp from the nearest point of the PHC grid ``k * P``,
  in [-P/2, P/2): after a PHC step the servo re-aligns within half a period instead of slewing the whole step;
  ``unwrapped`` -- distance from the grid anchored on the first captured edge (follows the full step).
  Offset convention as for the PTP servo: ``offset = FSYNC - PHC`` (> 0: FSYNC early), the controller outputs an
  absolute ppb correction.
* Command: ``NUM = NUM0 + round(ppb * n0 / 1e9)`` clamped to the 30-bit range, written ``cmd_latency_ns`` after the
  capture, split in writes of at most 200 kHz of VCO one lock time apart (``step_policy``, as in the shared mode).
* Lock lag after a ``NUM`` write (<= 8.3 us) is not modelled as a slow transition: the write takes effect at once.
"""
from __future__ import annotations

import bisect
import math
from fractions import Fraction

import numpy as np

from .clock import PS_PER_NS, SlaveClock
from .controllers import ServoSample, make_controller
from .pll import AudioPll

PS_PER_S = 10**12


class FsyncServo:
    def __init__(self, cfg, phc_initial_offset_ns: float, quantum_ps: int, dist):
        f = cfg.fsync
        pc = f.pll
        self.pll = AudioPll(pc.fref_hz, pc.vco_hz, pc.den, pc.step_limit_hz)
        if pc.vco_hz % pc.fsync_div:
            raise ValueError("VCO / fsync_div is not a whole number of Hz")
        self.freq_hz = pc.vco_hz // pc.fsync_div
        per = Fraction(PS_PER_S, self.freq_hz)                       # period in ps, exact
        self._pn, self._pd = per.numerator, per.denominator
        self.period_ns = 1e9 / self.freq_hz
        self.M = int(f.decimation)
        if self.M < 1:
            raise ValueError("fsync.decimation must be >= 1")
        self.nominal_s = self.M / self.freq_hz
        self.policy = pc.step_policy
        self.detector = f.phase_detector
        if self.detector not in ("wrapped", "unwrapped"):
            raise ValueError(f"unknown phase detector {self.detector!r}")
        self.lat_ps = int(round(f.cmd_latency_ns * PS_PER_NS))
        self.noise_ns = float(f.capture_noise_ns)
        self.quantum_ps = int(quantum_ps)
        self.dist = dist
        self.ctrl = make_controller(f.controller.name, f.controller.params)
        self.gap_ps = int(round(self.pll.lock_time_s * PS_PER_S))
        psi0 = phc_initial_offset_ns if f.initial_offset_ns is None else f.initial_offset_ns
        self.num = self.pll.num0                  # NUM in the hardware
        self.num_target = self.num                # last NUM computed by the servo
        self._eps_i = 0                           # index of the crystal error in force
        self._eps = None
        self.clock = SlaveClock(psi0, 0.0, 0)     # slope set at the first advance (needs the crystal history)
        # first rising edge after t = 0: F(0) = psi0 (ns) -> k = floor(psi0 / P) + 1
        self.k_next = int(math.floor(psi0 * 1000 * self._pd / self._pn)) + 1
        self._pending: list[tuple[int, int]] = []  # (t_ps, NUM) writes not yet in force
        self._anchor_ps: int | None = None
        self._prev_ts: int | None = None
        self.n_updates = 0
        self.index = 0
        self.t_done = 0
        # records (one per update)
        self.r_t: list[float] = []
        self.r_off: list[int] = []                # offset seen by the servo (ns)
        self.r_cmd: list[float] = []              # controller output (ppb)
        self.r_num: list[int] = []                # NUM requested
        self.r_psi: list[float] = []              # true FSYNC phase vs the GM grid at the edge (ns, unwrapped)
        self.r_rate: list[float] = []             # true FSYNC rate error vs GM at the edge (ppb)
        self.r_phc: list[float] = []              # true PHC offset at the edge (ns)

    # ------------------------------------------------------------------ helpers
    def _slope(self) -> float:
        e = self._eps * 1e-9
        d = self.pll.delta_of(self.num)
        return e + d + e * d

    # ------------------------------------------------------------------ main loop
    def advance(self, phc: SlaveClock, eps_t: list[int], eps_v: list[float], t_end_ps: int) -> None:
        """Run every edge capture, servo update and NUM write up to ``t_end_ps`` (physical, ps).

        Written as one flat loop (locals, no per-update objects besides the controller sample): 300 000 updates per
        300 s at 1 ms decimation.  The FSYNC segments are appended to ``self.clock`` directly (same layout as
        ``SlaveClock.set_slope``: a rate change at the instant of the previous one replaces it)."""
        clock = self.clock
        ct0, cphi, cslope, cbig = clock._t0, clock._phi0, clock._slope, clock._big
        if self._eps is None:
            self._eps = eps_v[0]
            cslope[-1] = self._slope()
        pending = self._pending
        ctrl = self.ctrl
        pll = self.pll
        num0, n0 = pll.num0, pll.n0
        num_min, num_max, max_step = pll.num_min, pll.num_max, pll.max_step_num
        num_per_ppb = n0 / 1e9
        policy = self.policy
        P_ns = self.period_ns
        half = P_ns / 2
        pn, pd = self._pn, self._pd
        q = self.quantum_ps
        M = self.M
        wrapped = self.detector == "wrapped"
        noise_ns = self.noise_ns
        lat, gap = self.lat_ps, self.gap_ps
        nominal = self.nominal_s
        # PHC cursor
        pt0, pbig, pphi, pslope = phc._t0, phc._big, phc._phi0, phc._slope
        j = max(0, bisect.bisect_right(pt0, self.t_done) - 1)
        npt = len(pt0)
        n_eps = len(eps_t)
        r_t, r_off, r_cmd, r_num = self.r_t, self.r_off, self.r_cmd, self.r_num
        r_psi, r_rate, r_phc = self.r_psi, self.r_rate, self.r_phc
        eps = self._eps * 1e-9

        def new_segment(t: int) -> None:
            # FSYNC rate after a crystal change or a NUM write at t (phase continuous)
            d = (self.num - num0) / n0
            sl = eps + d + eps * d
            if t == ct0[-1]:
                cslope[-1] = sl
                return
            ph = cphi[-1] + cslope[-1] * ((t - ct0[-1]) / PS_PER_NS)
            ct0.append(t)
            cphi.append(ph)
            cslope.append(sl)
            cbig.append(0)

        while True:
            k = self.k_next
            t0 = ct0[-1]
            u = ((k * pn - t0 * pd) / pd - cphi[-1] * PS_PER_NS) / (1.0 + cslope[-1])
            t_e = t0 + int(round(u))
            t_ch = t_e
            t_eps = eps_t[self._eps_i + 1] if self._eps_i + 1 < n_eps else None
            if t_eps is not None and t_eps < t_ch:
                t_ch = t_eps
            if pending and pending[0][0] < t_ch:
                t_ch = pending[0][0]
            if t_ch > t_end_ps:
                break
            if t_ch != t_e:
                if t_eps == t_ch:
                    self._eps_i += 1
                    self._eps = eps_v[self._eps_i]
                    eps = self._eps * 1e-9
                else:
                    self.num = pending.pop(0)[1]
                new_segment(t_ch)
                continue
            # ---- capture of edge k at t_e
            while j + 1 < npt and pt0[j + 1] <= t_e:
                j += 1
            phi_phc = pphi[j] + pslope[j] * ((t_e - pt0[j]) / PS_PER_NS)
            noise = self.dist.normal_ns("fsync.cap", self.n_updates, noise_ns) if noise_ns else 0.0
            x = t_e + pbig[j] * PS_PER_NS + int(round((phi_phc + noise) * PS_PER_NS))
            if q > 0:
                x = (x // q) * q
            ts_ns = x // PS_PER_NS                                         # whole-ns timestamp (relative to the epoch)
            if wrapped:
                # distance from the nearest grid point k' * P, in [-P/2, P/2) (exact: P = pn / pd ps)
                r = ((ts_ns * PS_PER_NS * pd) % pn) / pd / PS_PER_NS
                if r >= half:
                    r -= P_ns
            else:
                if self._anchor_ps is None:
                    self._anchor_ps = ts_ns * PS_PER_NS * pd - k * pn
                r = (ts_ns * PS_PER_NS * pd - k * pn - self._anchor_ps) / pd / PS_PER_NS
            offset = -int(round(r))                                        # FSYNC - PHC (> 0: FSYNC early)
            prev = self._prev_ts
            dt = (ts_ns - prev) * 1e-9 if prev is not None else nominal
            self._prev_ts = ts_ns
            ppb = ctrl.update(ServoSample(offset, dt, nominal, self.index))
            self.index += 1
            if ppb == ppb and abs(ppb) != math.inf:
                target = num0 + int(round(ppb * num_per_ppb))
            else:
                target = num0
            if target > num_max:
                target = num_max
            elif target < num_min:
                target = num_min
            cur = self.num_target
            if max_step <= 0 or policy == "off" or abs(target - cur) <= max_step:
                writes = (target,)
            else:
                writes = pll.ramp(cur, target, policy)
            self.num_target = writes[-1]
            pending.clear()                                                # a newer command replaces an older ramp
            t_w0 = t_e + lat
            for i, n in enumerate(writes):
                pending.append((t_w0 + i * gap, n))
            # records
            psi = cphi[-1] + cslope[-1] * ((t_e - ct0[-1]) / PS_PER_NS)
            r_t.append(t_e / PS_PER_S)
            r_off.append(offset)
            r_cmd.append(ppb)
            r_num.append(self.num_target)
            r_psi.append(psi)
            r_rate.append(cslope[-1] * 1e9)
            r_phc.append(pbig[j] + phi_phc)
            self.n_updates += 1
            self.k_next = k + M
            if pending[0][0] <= t_e:                                       # zero latency: the write is in force now
                self.num = pending.pop(0)[1]
                new_segment(t_e)
        self.t_done = t_end_ps

    def result(self) -> dict:
        f = np.array
        return {"t_s": f(self.r_t), "offset_ns": f(self.r_off, dtype=np.float64), "cmd_ppb": f(self.r_cmd),
                "num": f(self.r_num, dtype=np.int64), "psi_ns": f(self.r_psi), "rate_ppb": f(self.r_rate),
                "phc_offset_ns": f(self.r_phc), "freq_hz": self.freq_hz, "decimation": self.M,
                "num0": self.pll.num0, "lsb_ppb": self.pll.lsb_ppb, "n_updates": self.n_updates}
