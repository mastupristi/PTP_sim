# SPDX-License-Identifier: Apache-2.0
"""Oscilloscope view of the two clocks: a square wave locked to the Grandmaster and one locked to the slave clock.

Both are ideal outputs at ``freq_hz`` (48 kHz by default, the TDM frame sync): the GM wave has its rising edges at
``k * period`` of the physical (= GM) time (exactly ``k * 1e12 / freq_hz`` ps, rounded to the ps), the slave wave has its rising edges whenever the *slave clock reads* ``k * period``.  The scope
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
from fractions import Fraction

import numpy as np

PS_PER_NS = 1000
DEFAULT_FSYNC_HZ = 48_000             # TDM frame sync
_BIG = 1 << 40                        # |big| below this: int64 arithmetic is safe for any frequency up to 1 MHz


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

    def __init__(self, t0_ps, big_ns, small_ns, slope, freq_hz: int = DEFAULT_FSYNC_HZ):
        self.freq_hz = int(freq_hz)
        self.period_ns = 1e9 / self.freq_hz                       # not an integer for 48 kHz (20 833.33 ns)
        frac = Fraction(10**12, self.freq_hz)                     # the period in ps as an exact fraction pn / pd
        self._pn, self._pd = frac.numerator, frac.denominator
        self._p_ps = self._pn / self._pd
        self.t0 = np.asarray(t0_ps, dtype=np.int64)
        self.big = np.asarray(big_ns, dtype=np.int64)
        self.small = np.asarray(small_ns, dtype=np.float64)
        self.slope = np.asarray(slope, dtype=np.float64)
        # the integer part is reduced modulo the period *as an integer* (big mod 1e9/f = ((big * f) mod 1e9) / f); only
        # then is it added to the float part
        self._big_mod = self._reduce(self.big, self.freq_hz)
        self._t0_list = self.t0.tolist()

    @staticmethod
    def _reduce(big: np.ndarray, f: int) -> np.ndarray:
        out = np.empty(big.shape, dtype=np.float64)
        small = np.abs(big) < _BIG
        out[small] = ((big[small] * f) % 10**9) / f
        for i in np.flatnonzero(~small):                          # a huge offset (PHC at 0 against the epoch): Python ints
            out[i] = ((int(big[i]) * f) % 10**9) / f
        return out

    @classmethod
    def from_clock(cls, clock, freq_hz: int = DEFAULT_FSYNC_HZ) -> "ScopeModel":
        return cls(*clock.exact_breakpoints(), freq_hz=freq_hz)

    @classmethod
    def from_dict(cls, d: dict, freq_hz: int = DEFAULT_FSYNC_HZ) -> "ScopeModel":
        return cls(d["t0_ps"], d["big_ns"], d["small_ns"], d["slope"], freq_hz=freq_hz)

    # ------------------------------------------------------------------ trigger instants
    def trigger_ps(self, k):
        """GM time of the rising edge ``k``: ``round(k * 1e12 / f)`` ps (int, or an int64 array)."""
        n, d = self._pn, self._pd
        if isinstance(k, np.ndarray):
            kmax = int(np.max(np.abs(k))) if k.size else 0
            if 2 * kmax * n + d >= 2**63:
                raise OverflowError("sweep number too large for int64 trigger times")
            return (2 * k.astype(np.int64) * n + d) // (2 * d)
        return (2 * int(k) * n + d) // (2 * d)

    def _eps_ns(self, k, t_ps):
        """Rounding of the trigger instant: ``t_ps - k * P`` in ns (|.| <= 0.5 ps); the slave reads ``t + phi``, which is
        ``phi + eps`` modulo the period."""
        return (self._pd * t_ps - k * self._pn) / self._pd / PS_PER_NS

    # ------------------------------------------------------------------ phase
    def _seg(self, t_ps: int) -> int:
        return max(0, bisect.bisect_right(self._t0_list, t_ps) - 1)

    def _wrapped_a(self, j: int, t_ps: int, eps_ns: float = 0.0) -> float:
        """Phase of segment ``j`` extrapolated to ``t_ps`` (ns), reduced modulo the period to [-P/2, P/2); ``eps_ns``
        is the rounding of ``t_ps`` against the exact trigger instant."""
        a = self._big_mod[j] + self.small[j] + self.slope[j] * ((t_ps - self._t0_list[j]) / PS_PER_NS) + eps_ns
        p = self.period_ns
        return a - p * np.floor((a + p / 2) / p)

    def phi_ns(self, t_ps: int) -> float:
        """True offset slave - GM at ``t_ps`` (ns, float)."""
        j = self._seg(t_ps)
        return float(self.big[j]) + self.small[j] + self.slope[j] * ((t_ps - self._t0_list[j]) / PS_PER_NS)

    # ------------------------------------------------------------------ edges
    def slave_edge_ns(self, t_trig_ps: int, eps_ns: float = 0.0) -> tuple[float, int]:
        """Offset (ns) of the slave rising edge nearest to the trigger ``t_trig_ps`` (a multiple of the period), and
        the index of the segment it falls in.  Solves ``t + phi(t) = k * P`` exactly on the segment that contains the
        edge (``d * (1 + slope) = -A`` with ``A`` the phase of that segment extrapolated to the trigger); when the edge
        falls in another segment than the trigger the solve is repeated there."""
        j = self._seg(t_trig_ps)
        d = 0.0
        for _ in range(4):
            d = -self._wrapped_a(j, t_trig_ps, eps_ns) / (1.0 + self.slope[j])
            j2 = self._seg(t_trig_ps + int(round(d * PS_PER_NS)))
            if j2 == j:
                break
            j = j2
        return float(d) + eps_ns, j                  # relative to the exact trigger instant k * P, not the rounded one

    @property
    def n_sweeps_per_s(self) -> float:
        return 1e9 / self.period_ns

    def sweep(self, k: int) -> Sweep:
        """Sweep triggered on the GM rising edge ``k`` (``t = k * period``)."""
        t_ps = self.trigger_ps(k)
        d, j = self.slave_edge_ns(t_ps, self._eps_ns(int(k), t_ps))
        phi = self.phi_ns(t_ps)
        return Sweep(k=int(k), t_s=t_ps / 1e12, edge_ns=d, half_ns=self.period_ns / 2.0 / (1.0 + float(self.slope[j])),
                     phi_ns=phi, ambiguous=abs(phi) > self.period_ns / 2.0)

    def sweeps(self, ks) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Vectorised :meth:`sweep` for an array of sweep numbers: ``(edge_ns, half_ns, phi_ns)`` arrays, the same
        values as the scalar method (same segment search and iteration)."""
        ks = np.asarray(ks, dtype=np.int64)
        t = self.trigger_ps(ks)
        eps = self._eps_ns(ks, t)
        j = np.maximum(0, np.searchsorted(self.t0, t, side="right") - 1)
        d = np.zeros(t.shape)
        active = np.ones(t.shape, dtype=bool)
        p = self.period_ns
        for _ in range(4):
            ja, ta = j[active], t[active]
            a = self._big_mod[ja] + self.small[ja] + self.slope[ja] * ((ta - self.t0[ja]) / PS_PER_NS) + eps[active]
            a = a - p * np.floor((a + p / 2) / p)
            da = -a / (1.0 + self.slope[ja])
            d[active] = da
            j2 = np.maximum(0, np.searchsorted(self.t0, ta + np.rint(da * PS_PER_NS).astype(np.int64), side="right") - 1)
            changed = j2 != ja
            j[active] = j2
            idx = np.flatnonzero(active)
            active[idx[~changed]] = False
            if not active.any():
                break
        phi = self.big[j].astype(np.float64) + self.small[j] + self.slope[j] * ((t - self.t0[j]) / PS_PER_NS)
        return d + eps, p / 2.0 / (1.0 + self.slope[j]), phi

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


def pick_timebase_ns(edge_ns: float, available, margin: float = 1.25, divisions: int = 5) -> float:
    """Smallest ns/div whose half screen (``divisions`` divisions) holds ``margin * |edge|`` (the largest if none)."""
    need = margin * abs(edge_ns)
    return next((tb for tb in available if divisions * tb >= need), available[-1])


# ---------------------------------------------------------------------- persistence (what a real scope's display does)
def persistence_samples(k: int, span_sweeps: float, max_n: int, k_min: int = 0):
    """Sweeps shown by a variable persistence of ``span_sweeps`` (simulated time) at sweep ``k``, as ``(ks, weights)``.
    The newest trace has weight 1 and the weight falls linearly to 0 at the age ``span_sweeps`` (the trace has faded to
    black and is erased); a longer window is thinned to at most ``max_n`` sweeps on a fixed grid (multiples of the
    step), so the picture is a function of ``k`` only and does not shimmer from frame to frame.  ``k`` itself is the
    first element (weight 1).  Sweeps before ``k_min`` do not exist for the display (it was reset there)."""
    if span_sweeps <= 0 or k <= 0:
        return np.array([k], dtype=np.int64), np.array([1.0])
    step = max(1, int(np.ceil(span_sweeps / max_n)))
    first = int(np.floor((k - span_sweeps) / step)) + 1                 # grid index of the oldest sweep still alive
    ks = np.arange(max(first, 0), k // step + 1, dtype=np.int64) * step
    ks = ks[(ks < k) & (ks > k - span_sweeps) & (ks >= k_min)]
    ks = np.concatenate(([k], ks[::-1])).astype(np.int64)
    return ks, 1.0 - (k - ks) / span_sweeps


def edge_profile(edge_ns, half_ns, weights, x0_ns: float, x1_ns: float, width: int):
    """Weighted per-column statistics of many square waves over the screen ``[x0, x1]`` cut in ``width`` columns.
    Wave ``i`` has rising edges at ``edge_i + 2 m half_i`` and falling ones half a period later.  Returns
    ``(high, edges, total)``: the weight of the waves that are high in each column, the weight of the edges that
    fall into each column, and the total weight (a wave not high is low).  All three are additive, so profiles of
    different batches of sweeps can simply be summed."""
    e = np.asarray(edge_ns, dtype=np.float64)
    h = np.asarray(half_ns, dtype=np.float64)
    w = np.asarray(weights, dtype=np.float64)
    j_lo = np.ceil((x0_ns - e) / h).astype(np.int64)                    # first edge index on screen
    j_hi = np.floor((x1_ns - e) / h).astype(np.int64)
    cnt = np.maximum(0, j_hi - j_lo + 1)
    # an edge j is rising for even j; the level on the left border is the one after edge j_lo - 1
    start_high = ((j_lo - 1) % 2 == 0)
    diff = np.zeros(width + 1)
    edges = np.zeros(width)
    n = int(cnt.sum())
    if n:
        i = np.repeat(np.arange(e.size), cnt)
        jj = j_lo[i] + (np.arange(n) - np.repeat(np.cumsum(cnt) - cnt, cnt))
        col = np.clip(np.floor((e[i] + jj * h[i] - x0_ns) / (x1_ns - x0_ns) * width).astype(np.int64), 0, width - 1)
        sign = np.where(jj % 2 == 0, 1.0, -1.0) * w[i]
        diff += np.bincount(col, weights=sign, minlength=width + 1)[: width + 1]
        edges += np.bincount(col, weights=w[i], minlength=width)[:width]
    high = float(w[start_high].sum()) + np.cumsum(diff[:width])
    return high, edges, float(w.sum())


def phosphor(h, gain: float = 3.0):
    """Brightness (0..1, saturating) of an accumulated weight: one fresh trace gives about 0.3, repeated hits of the
    same pixel brighten it towards 1, like the intensity grading of a digital phosphor display."""
    return 1.0 - np.exp(-np.asarray(h, dtype=np.float64) / gain)
