# SPDX-License-Identifier: Apache-2.0
"""Slave clock: piecewise-linear phase with exact (integer) segment anchors.

Physical time is the GM time, an integer number of picoseconds.  The slave clock reads
``t + phi(t)`` where ``phi`` (ns, float) is the *true* offset slave - GM.  Within a segment
``phi(t) = phi0 + slope * (t - t0)`` with ``slope = (1 + eps_osc) * ratio_eff - 1``.
A rate change anchors a new segment at the exact instant, so phase is continuous; a phase
step (``step``) is the only discontinuity and is recorded explicitly.

Absolute timestamps are never put in a float: ``phi`` is bounded by the clock error while the
epoch (1.7e18 ns) only lives in Python ints.
"""
from __future__ import annotations

import bisect

import numpy as np

PS_PER_NS = 1000


class SlaveClock:
    def __init__(self, phi0_ns: float, slope: float, t0_ps: int = 0):
        self._t0 = [int(t0_ps)]
        self._phi0 = [float(phi0_ns)]
        self._slope = [float(slope)]
        self._step_t: list[int] = []          # instants of phase steps
        self._step_size: list[float] = []     # ns

    # ---- state ------------------------------------------------------------
    @property
    def slope(self) -> float:
        return self._slope[-1]

    @property
    def last_anchor_ps(self) -> int:
        return self._t0[-1]

    def phi_ns(self, t_ps: int) -> float:
        """True offset slave - GM at physical time ``t_ps`` (>= last anchor)."""
        return self._phi0[-1] + self._slope[-1] * ((t_ps - self._t0[-1]) / PS_PER_NS)

    def phi_hist_ns(self, t_ps: int) -> float:
        """True offset at any past/present instant (looks the segment up)."""
        i = bisect.bisect_right(self._t0, t_ps) - 1
        if i < 0:
            i = 0
        return self._phi0[i] + self._slope[i] * ((t_ps - self._t0[i]) / PS_PER_NS)

    def set_slope(self, t_ps: int, slope: float) -> None:
        """Change the rate at ``t_ps`` keeping phase continuity."""
        phi = self.phi_ns(t_ps)
        self._anchor(t_ps, phi, slope)

    def step(self, t_ps: int, delta_ns: float) -> None:
        """Phase jump of ``delta_ns`` (clock set / adjust) at ``t_ps``."""
        phi = self.phi_ns(t_ps)
        self._step_t.append(t_ps)
        self._step_size.append(delta_ns)
        self._anchor(t_ps, phi + delta_ns, self._slope[-1])

    def _anchor(self, t_ps: int, phi: float, slope: float) -> None:
        if t_ps == self._t0[-1]:
            # same instant: replace the (zero-length) segment
            self._phi0[-1], self._slope[-1] = phi, slope
        else:
            self._t0.append(int(t_ps))
            self._phi0.append(phi)
            self._slope.append(slope)

    # ---- timestamping ------------------------------------------------------
    def read_ps(self, t_ps: int, noise_ns: float = 0.0, quantum_ps: int = 0) -> int:
        """Slave clock reading relative to the epoch, in integer ps, optionally noisy and
        floored to a multiple of ``quantum_ps`` (the timer tick)."""
        x = t_ps + int(round((self.phi_ns(t_ps) + noise_ns) * PS_PER_NS))
        if quantum_ps > 0:
            x = (x // quantum_ps) * quantum_ps
        return x

    # ---- analysis ----------------------------------------------------------
    def breakpoints(self):
        return (np.array(self._t0, dtype=np.int64), np.array(self._phi0), np.array(self._slope))

    def sample_ns(self, t_ps: np.ndarray) -> np.ndarray:
        """True offset (ns) on an array of physical times (ps, any order).  A time that
        coincides with a phase step returns the value after the step."""
        t0, phi0, slope = self.breakpoints()
        t_ps = np.asarray(t_ps, dtype=np.int64)
        idx = np.clip(np.searchsorted(t0, t_ps, side="right") - 1, 0, t0.size - 1)
        return phi0[idx] + slope[idx] * ((t_ps - t0[idx]) / PS_PER_NS)

    def segment_end_values(self, t_end_ps: int):
        """Exact piecewise-linear vertices (time_ps, phi before, phi after), for peak search."""
        t0, phi0, slope = self.breakpoints()
        ends = np.append(t0[1:], t_end_ps)
        phi_end = phi0 + slope * ((ends - t0) / PS_PER_NS)
        return t0, phi0, ends, phi_end
