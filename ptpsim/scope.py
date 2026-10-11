# SPDX-License-Identifier: Apache-2.0
"""Oscilloscope view of the two clocks: a square wave locked to the Grandmaster and one locked to the slave clock.

Both are ideal outputs at ``1 / period`` (1 kHz by default): the GM wave has its rising edges at ``k * period`` of the
physical (= GM) time, the slave wave has its rising edges whenever the *slave clock reads* ``k * period``.  The scope
triggers on the GM rising edge, so the GM trace is always the same and the slave trace is shifted by the true offset:
with ``phi = slave - GM > 0`` the slave is ahead and its edge comes *before* the trigger (drawn at ``-phi``).

Nothing is stored per edge: the slave clock is piecewise linear (``SlaveClock``), so the edge of any sweep is found
analytically from the segment breakpoints (a ``bisect`` per sweep).  A square wave cannot show an offset larger than half
a period unambiguously: the picture shows the offset wrapped to ``[-P/2, P/2)`` (nearest slave edge) and ``Sweep.ambiguous``
says that the real offset is larger.

Approximations (documented in docs/model.md): the slave wave is derived from the mean-rate clock model, i.e. it ignores
the NXP timer tick; the edges are ideal (no output jitter).
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass

import numpy as np

PS_PER_NS = 1000
DEFAULT_PERIOD_NS = 1_000_000          # 1 kHz


@dataclass(frozen=True)
class Sweep:
    """One scope sweep, triggered on the GM rising edge number ``k``."""
    k: int
    t_s: float            # GM time of the trigger
    edge_ns: float        # slave rising edge relative to the trigger (nearest one, in [-P/2, P/2))
    half_ns: float        # slave half period (P / 2 / (1 + slope), the slave clock runs at 1 + slope)
    phi_ns: float         # true offset slave - GM at the trigger (float; huge values lose ns resolution)
    ambiguous: bool       # |phi| > P / 2: the wave shows the offset modulo the period


class ScopeModel:
    """Edges of the slave wave from the exact breakpoints of the slave clock (``SlaveClock.exact_breakpoints``)."""

    def __init__(self, t0_ps, big_ns, small_ns, slope, period_ns: int = DEFAULT_PERIOD_NS):
        self.period_ns = int(period_ns)
        self._p_ps = self.period_ns * PS_PER_NS
        self.t0 = np.asarray(t0_ps, dtype=np.int64)
        self.big = np.asarray(big_ns, dtype=np.int64)
        self.small = np.asarray(small_ns, dtype=np.float64)
        self.slope = np.asarray(slope, dtype=np.float64)
        # the integer part is reduced modulo the period *as an integer*; only then is it added to the float part
        self._big_mod = (self.big % self.period_ns).astype(np.float64)
        self._t0_list = self.t0.tolist()

    @classmethod
    def from_clock(cls, clock, period_ns: int = DEFAULT_PERIOD_NS) -> "ScopeModel":
        return cls(*clock.exact_breakpoints(), period_ns=period_ns)

    @classmethod
    def from_dict(cls, d: dict, period_ns: int = DEFAULT_PERIOD_NS) -> "ScopeModel":
        return cls(d["t0_ps"], d["big_ns"], d["small_ns"], d["slope"], period_ns=period_ns)

    # ------------------------------------------------------------------ phase
    def _seg(self, t_ps: int) -> int:
        return max(0, bisect.bisect_right(self._t0_list, t_ps) - 1)

    def _wrapped_a(self, j: int, t_ps: int) -> float:
        """Phase of segment ``j`` extrapolated to ``t_ps`` (ns), reduced modulo the period to [-P/2, P/2)."""
        a = self._big_mod[j] + self.small[j] + self.slope[j] * ((t_ps - self._t0_list[j]) / PS_PER_NS)
        p = self.period_ns
        return a - p * np.floor((a + p / 2) / p)

    def phi_ns(self, t_ps: int) -> float:
        """True offset slave - GM at ``t_ps`` (ns, float)."""
        j = self._seg(t_ps)
        return float(self.big[j]) + self.small[j] + self.slope[j] * ((t_ps - self._t0_list[j]) / PS_PER_NS)

    # ------------------------------------------------------------------ edges
    def slave_edge_ns(self, t_trig_ps: int) -> tuple[float, int]:
        """Offset (ns) of the slave rising edge nearest to the trigger ``t_trig_ps`` (a multiple of the period), and
        the index of the segment it falls in.  Solves ``t + phi(t) = k * P`` exactly on the segment that contains the
        edge (``d * (1 + slope) = -A`` with ``A`` the phase of that segment extrapolated to the trigger); when the edge
        falls in another segment than the trigger the solve is repeated there."""
        j = self._seg(t_trig_ps)
        d = 0.0
        for _ in range(4):
            d = -self._wrapped_a(j, t_trig_ps) / (1.0 + self.slope[j])
            j2 = self._seg(t_trig_ps + int(round(d * PS_PER_NS)))
            if j2 == j:
                break
            j = j2
        return float(d), j

    @property
    def n_sweeps_per_s(self) -> float:
        return 1e9 / self.period_ns

    def sweep(self, k: int) -> Sweep:
        """Sweep triggered on the GM rising edge ``k`` (``t = k * period``)."""
        t_ps = int(k) * self._p_ps
        d, j = self.slave_edge_ns(t_ps)
        phi = self.phi_ns(t_ps)
        return Sweep(k=int(k), t_s=t_ps / 1e12, edge_ns=d, half_ns=self.period_ns / 2.0 / (1.0 + float(self.slope[j])),
                     phi_ns=phi, ambiguous=abs(phi) > self.period_ns / 2.0)

    def sweep_at(self, t_s: float) -> Sweep:
        """Sweep whose trigger is the GM edge nearest to ``t_s``."""
        return self.sweep(int(round(t_s * 1e12 / self._p_ps)))


def square_xy(rise_ns: float, half_ns: float, x0_ns: float, x1_ns: float, low: float = 0.0, high: float = 1.0):
    """Polyline (ns, level) of a 50 % square wave over [x0, x1], with rising edges at ``rise + 2 m half``."""
    m_lo = int(np.ceil((x0_ns - rise_ns) / half_ns))
    m_hi = int(np.floor((x1_ns - rise_ns) / half_ns))
    m = np.arange(m_lo, m_hi + 1)
    edges = rise_ns + m * half_ns
    level_after = np.where(m % 2 == 0, high, low)                 # even m: rising edge
    level_start = high if (m_lo - 1) % 2 == 0 else low            # level at x0, before the first edge
    xs = [x0_ns]
    ys = [level_start]
    for e, la in zip(edges, level_after):
        xs += [e, e]
        ys += [ys[-1], la]
    xs.append(x1_ns)
    ys.append(ys[-1])
    return np.array(xs), np.array(ys)


def pick_timebase_ns(edge_ns: float, available, current: float | None = None, margin: float = 1.25,
                     divisions: int = 5) -> float:
    """Auto timebase: smallest ns/div whose half screen (``divisions`` divisions) holds ``margin * |edge|``; going
    back to a finer one needs the offset to shrink well below the current one (no flicker while playing)."""
    need = margin * abs(edge_ns)
    best = next((tb for tb in available if divisions * tb >= need), available[-1])
    if current is not None and best < current and best * 3 > current:
        return current
    return best
