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
    """Phase = ``big`` (exact integer ns, only non-zero while the offset is huge, e.g. a PHC that starts at 0
    against a 1.7e18 ns epoch) + ``small`` (float ns).  A forced alignment (``step``) with an integer delta
    cancels the big part exactly, so the residual offset keeps ns accuracy after the step."""

    _BIG = 1 << 40

    def __init__(self, phi0_ns: float, slope: float, t0_ps: int = 0):
        self._t0 = [int(t0_ps)]
        big, small = self._split(phi0_ns)
        self._big = [big]
        self._phi0 = [small]
        self._slope = [float(slope)]
        self._step_t: list[int] = []          # instants of phase steps
        self._step_size: list[float] = []     # ns

    @classmethod
    def _split(cls, phi_ns):
        if abs(phi_ns) >= cls._BIG:
            return int(round(phi_ns)), 0.0
        return 0, float(phi_ns)

    # ---- state ------------------------------------------------------------
    @property
    def slope(self) -> float:
        return self._slope[-1]

    @property
    def last_anchor_ps(self) -> int:
        return self._t0[-1]

    def _small_at(self, i: int, t_ps: int) -> float:
        return self._phi0[i] + self._slope[i] * ((t_ps - self._t0[i]) / PS_PER_NS)

    def phi_ns(self, t_ps: int) -> float:
        """True offset slave - GM at physical time ``t_ps`` (>= last anchor), as a float."""
        return self._big[-1] + self._small_at(-1, t_ps)

    def phi_hist_ns(self, t_ps: int) -> float:
        """True offset at any past/present instant (looks the segment up)."""
        i = max(0, bisect.bisect_right(self._t0, t_ps) - 1)
        return self._big[i] + self._small_at(i, t_ps)

    def set_slope(self, t_ps: int, slope: float) -> None:
        """Change the rate at ``t_ps`` keeping phase continuity."""
        self._anchor(t_ps, self._big[-1], self._small_at(-1, t_ps), slope)

    def step(self, t_ps: int, delta_ns) -> None:
        """Phase jump of ``delta_ns`` (clock set / adjust) at ``t_ps``.  An ``int`` delta is exact."""
        small = self._small_at(-1, t_ps)
        big = self._big[-1]
        if isinstance(delta_ns, int):
            big += delta_ns
        else:
            small += delta_ns
        if abs(big) < self._BIG:                      # merge a small residual into the float part
            small += big
            big = 0
        self._step_t.append(t_ps)
        self._step_size.append(float(delta_ns))
        self._anchor(t_ps, big, small, self._slope[-1])

    def _anchor(self, t_ps: int, big: int, small: float, slope: float) -> None:
        if t_ps == self._t0[-1]:
            # same instant: replace the (zero-length) segment
            self._big[-1], self._phi0[-1], self._slope[-1] = big, small, slope
        else:
            self._t0.append(int(t_ps))
            self._big.append(big)
            self._phi0.append(small)
            self._slope.append(slope)

    # ---- timestamping ------------------------------------------------------
    def read_ps(self, t_ps: int, noise_ns: float = 0.0, quantum_ps: int = 0) -> int:
        """Slave clock reading relative to the epoch, in integer ps, optionally noisy and
        floored to a multiple of ``quantum_ps`` (the timer tick).  Exact for huge offsets."""
        x = t_ps + self._big[-1] * PS_PER_NS + int(round((self._small_at(-1, t_ps) + noise_ns) * PS_PER_NS))
        if quantum_ps > 0:
            x = (x // quantum_ps) * quantum_ps
        return x

    # ---- analysis ----------------------------------------------------------
    def breakpoints(self):
        """(anchor times in ps, total phase at the anchors (float), slopes)."""
        phi = np.array([float(b) for b in self._big]) + np.array(self._phi0)
        return (np.array(self._t0, dtype=np.int64), phi, np.array(self._slope))

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
