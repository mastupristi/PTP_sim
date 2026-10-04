# SPDX-License-Identifier: Apache-2.0
"""Synthetic metrics of a trajectory.

All metrics are computed on the full-resolution data (never on down-sampled render data).
The *true* offset is the physical offset slave - GM evaluated on a dense grid plus the exact
piecewise-linear vertices; the *estimated* offset is the firmware's estimate at its samples.

Definitions
-----------
* **Reference** ``x0``: offset at t = 0 (``initial_offset_ns``).
* **Settling time**: first instant after which ``|x| <= band`` for the rest of the run, valid only if
  the remaining time is >= ``dwell_s`` (permanence).  ``None`` = never settled (reason reported).
  If the series never leaves the band the settling time is 0.
* **Overshoot** [%]: ``max(0, max_t(-sign(x0) * x(t))) / |x0| * 100`` - largest excursion to the
  opposite side of the initial offset.  ``None`` when ``x0 == 0`` (undefined).
* **Peak error**: ``max |x|`` over the run (true offset: exact, from the piecewise-linear vertices),
  and ``peak_excess = peak - |x0|`` (growth beyond the initial error, e.g. from a frequency error).
* **RMS / bias** over the final window [T - W, T]: ``sqrt(mean(x^2))`` and ``mean(x)``.
* **Saturation**: servo updates whose command was clamped by the controller, and updates that
  caused a range reset (``ppb`` or actuator limit exceeded -> firmware ``clock_servo_reset``).
* **Diverged**: non-finite, or ``peak_abs > divergence_ns`` after the initial transient, or the final-window RMS
  larger than ``max(|x0|, band)``.
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .clock import PS_PER_NS
from .engine import PS_PER_S, SimResult


@dataclass
class MetricsConfig:
    band_ns: float = 1000.0
    dwell_s: float = 10.0
    final_window_s: float = 60.0
    divergence_ns: float = 1e9
    grid_dt_s: float = 0.01


def true_offset_grid(res: SimResult, dt_s: float = 0.01, max_points: int = 400_000):
    t_end = res.t_end_s
    n = int(min(max_points, max(2, t_end / dt_s + 1)))
    t = np.linspace(0.0, t_end, n)
    return t, res.true_offset_ns(t)


def _series_metrics(t: np.ndarray, x: np.ndarray, x0: float, t_end: float, mc: MetricsConfig,
                    peak_abs: float | None = None) -> dict:
    out: dict = {"n": int(t.size)}
    if t.size == 0 or not np.all(np.isfinite(x)):
        out.update(settling_s=None, settling_reason="no data" if t.size == 0 else "non-finite values",
                   overshoot_pct=None, peak_abs_ns=float("nan"), peak_excess_ns=float("nan"),
                   rms_final_ns=float("nan"), bias_final_ns=float("nan"), diverged=True)
        return out
    ax = np.abs(x)
    peak = float(ax.max()) if peak_abs is None else float(max(peak_abs, ax.max()))
    out["peak_abs_ns"] = peak
    out["peak_excess_ns"] = peak - abs(x0)
    # --- settling
    outside = np.nonzero(ax > mc.band_ns)[0]
    if outside.size == 0:
        t_settle, reason = float(t[0]) if t[0] > 0 else 0.0, "within band from the start"
    elif outside[-1] == t.size - 1:
        t_settle, reason = None, "still outside the band at the end of the run"
    else:
        t_settle, reason = float(t[outside[-1] + 1]), "ok"
    if t_settle is not None and (t_end - t_settle) < mc.dwell_s:
        t_settle, reason = None, f"inside the band for less than the dwell time ({mc.dwell_s:g} s)"
    out["settling_s"], out["settling_reason"] = t_settle, reason
    # --- overshoot
    if x0 == 0:
        out["overshoot_pct"] = None
    else:
        opposite = -np.sign(x0) * x
        out["overshoot_pct"] = float(max(0.0, opposite.max()) / abs(x0) * 100.0)
    # --- final window
    w = min(mc.final_window_s, t_end)
    m = t >= (t_end - w)
    xs = x[m] if m.any() else x[-1:]
    out["window_s"] = w
    out["rms_final_ns"] = float(np.sqrt(np.mean(xs ** 2)))
    out["bias_final_ns"] = float(np.mean(xs))
    out["std_final_ns"] = float(np.std(xs))
    big_final = out["rms_final_ns"] > max(abs(x0), mc.band_ns) and out["rms_final_ns"] > 10 * mc.band_ns
    out["diverged"] = bool(peak > mc.divergence_ns or (t_end > 2 * w and big_final))
    return out


def compute_metrics(res: SimResult, mc: MetricsConfig | None = None) -> dict:
    mc = mc or MetricsConfig()
    t0 = time.perf_counter()
    x0 = res.cfg.oscillator.initial_offset_ns
    tg, xg = true_offset_grid(res, mc.grid_dt_s)
    # exact peak over the piecewise-linear vertices
    t0s, p0, _ends, pend = res.clock.segment_end_values(int(round(res.t_end_s * PS_PER_S)))
    peak_true = float(max(np.abs(p0).max(), np.abs(pend).max()))
    true_m = _series_metrics(tg, xg, x0, res.t_end_s, mc, peak_abs=peak_true)

    est_t = res.servo_t_sample_s
    est_x = res.servo_offset_est_ns
    ok = np.isfinite(est_x)
    est_m = _series_metrics(est_t[ok], est_x[ok], x0, res.t_end_s, mc)

    n_upd = int((res.servo_action == 0).sum())
    cmd = res.servo_cmd_ppb[np.isfinite(res.servo_cmd_ppb)]
    sat = {
        "controller_clamped": res.counters["saturated"],
        "range_resets": res.counters["range_resets"],
        "servo_updates": n_upd,
        "steps": res.counters["steps"],
        "outliers_rejected": res.counters["outliers"],
        "resets": res.counters["resets"],
        "peak_command_ppb": float(np.abs(cmd).max()) if cmd.size else 0.0,
    }
    metrics = {
        "config_band_ns": mc.band_ns, "dwell_s": mc.dwell_s, "final_window_s": mc.final_window_s,
        "true_offset": true_m, "estimated_offset": est_m, "saturation": sat,
        "bias_of_estimate_ns": (est_m["bias_final_ns"] - true_m["bias_final_ns"])
        if est_m.get("bias_final_ns") == est_m.get("bias_final_ns") else None,
        "sim_wall_ms": res.wall_s * 1e3, "n_events": res.n_events,
    }
    metrics["metrics_wall_ms"] = (time.perf_counter() - t0) * 1e3
    return metrics
