# SPDX-License-Identifier: Apache-2.0
"""Analytic companions of the simulator (used by tests and the README stability table)."""
from __future__ import annotations

import numpy as np


def baseline_poles(kp: float, ki: float, ts: float) -> np.ndarray:
    """Closed-loop poles of the firmware PI on the ideal integrating clock, noise free.

    Model (exact for the ideal actuator, zero latencies, perfect delay compensation):
        phi[k+1] = phi[k] + Ts * u[k]          (ns, with u in ppb and Ts in s)
        I[k] = I[k-1] - ki * phi[k]
        u[k] = -kp * phi[k] + I[k]
    State (phi, I_prev):  [[1 - Ts(kp + ki), Ts], [-ki, 1]];  trace = 2 - Ts(kp+ki), det = 1 - Ts*kp.
    """
    a = np.array([[1.0 - ts * (kp + ki), ts], [-ki, 1.0]])
    return np.linalg.eigvals(a)


def baseline_recursion(kp: float, ki: float, ts: float, phi0: float, eps_ppb: float, n: int,
                       ratio_of_ppb, floor_estimate: bool = True) -> np.ndarray:
    """Reference recursion for the offset sampled at the Sync arrivals (see ``baseline_poles``).

    ``ratio_of_ppb`` maps the controller output (ppb) to the realised ratio (applies the firmware's
    ppb -> scaled ppm -> ratio quantisation).  The clock rate is ``(1 + eps) * ratio``.
    ``floor_estimate``: the firmware works in integer ns, the offset estimate is
    ``floor(phi)`` when the timestamp quantum is 0 (a truncated ns counter).
    """
    phi = np.empty(n)
    integ = 0.0
    p = phi0
    for k in range(n):
        phi[k] = p
        e = -float(np.floor(p)) if floor_estimate else -p
        integ += ki * e
        u = kp * e + integ
        d = ratio_of_ppb(u) - 1.0
        eps = eps_ppb * 1e-9
        slope = eps + d + eps * d
        p = p + ts * slope * 1e9
    return phi


def max_stable_sync_interval(kp: float, ki: float, ts_grid: np.ndarray | None = None) -> float:
    """Largest Ts on a log grid (0.0625 .. 64 s) for which both poles are inside the unit circle."""
    if ts_grid is None:
        ts_grid = 2.0 ** np.arange(-4, 7)
    best = 0.0
    for ts in ts_grid:
        if np.all(np.abs(baseline_poles(kp, ki, ts)) < 1.0):
            best = float(ts)
    return best
