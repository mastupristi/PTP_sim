# SPDX-License-Identifier: Apache-2.0
"""Synthetic metrics of a trajectory.

All metrics are computed on the full-resolution data (never on down-sampled render data).
The *true* offset is the physical offset slave - GM evaluated on a dense grid plus the exact
piecewise-linear vertices; the *estimated* offset is the firmware's estimate at its samples.

Definitions
-----------
* **Servo start**: the first PI command.  The servo only starts after the first valid delay sample (the
  first Delay_Req is sent at a random time in (0, 2*2^n] s), so up to a few seconds elapse during which the
  clock runs free (e.g. 100 us + 20 ppm * 3 s = 160 us).  Both controllers share this phase.
* **Reference** ``x_ref``: the true offset at the servo start (the "step" the controller must reject).
  ``x_initial`` (offset at t = 0) is also reported.
* **Settling time**: first instant after which ``|x| <= band`` for the rest of the run, valid only if
  the remaining time is >= ``dwell_s`` (permanence).  ``None`` = never settled (reason reported).
  If the series never leaves the band the settling time is 0.
* **Overshoot** [%]: ``max(0, max_{t >= t_servo}(-sign(x_ref) * x(t))) / |x_ref| * 100`` - largest excursion to
  the opposite side of the reference, also given in ns (``overshoot_ns``).  ``overshoot_vs_initial_pct`` uses
  ``x_initial`` over the whole run instead.  ``None`` when the reference is 0 (undefined).
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
                    peak_abs: float | None = None, x_ref: float | None = None, t_from: float = 0.0) -> dict:
    """``x0``: offset at t = 0; ``x_ref``/``t_from``: reference value and instant (servo start)."""
    if x_ref is None:
        x_ref = x0
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
    # --- overshoot (reference: offset at the servo start)
    after = t >= t_from
    if x_ref == 0 or not after.any():
        out["overshoot_pct"], out["overshoot_ns"] = None, None
    else:
        ov = float(max(0.0, (-np.sign(x_ref) * x[after]).max()))
        out["overshoot_ns"], out["overshoot_pct"] = ov, ov / abs(x_ref) * 100.0
    out["overshoot_ref_ns"], out["overshoot_ref_time_s"] = float(x_ref), float(t_from)
    if x0 == 0:
        out["overshoot_vs_initial_pct"] = None
    else:
        out["overshoot_vs_initial_pct"] = float(max(0.0, (-np.sign(x0) * x).max()) / abs(x0) * 100.0)
    # --- final window
    w = min(mc.final_window_s, t_end)
    m = t >= (t_end - w)
    xs = x[m] if m.any() else x[-1:]
    out["window_s"] = w
    out["rms_final_ns"] = float(np.sqrt(np.mean(xs ** 2)))
    out["bias_final_ns"] = float(np.mean(xs))
    out["std_final_ns"] = float(np.std(xs))
    scale = max(abs(x0), abs(x_ref), mc.band_ns)
    big_final = out["rms_final_ns"] > scale and out["rms_final_ns"] > 10 * mc.band_ns
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
    act = np.nonzero(res.servo_action == 0)[0]
    t_servo = float(res.servo_t_proc_s[act[0]]) if act.size else 0.0
    x_ref = float(res.true_offset_ns(np.array([t_servo]))[0]) if act.size else x0
    true_m = _series_metrics(tg, xg, x0, res.t_end_s, mc, peak_abs=peak_true, x_ref=x_ref, t_from=t_servo)

    est_t = res.servo_t_sample_s
    est_x = res.servo_offset_est_ns
    ok = np.isfinite(est_x)
    est_m = _series_metrics(est_t[ok], est_x[ok], x0, res.t_end_s, mc, x_ref=x_ref, t_from=t_servo)

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
        "servo_start_s": t_servo, "offset_at_servo_start_ns": x_ref, "initial_offset_ns": x0,
        "config_band_ns": mc.band_ns, "dwell_s": mc.dwell_s, "final_window_s": mc.final_window_s,
        "true_offset": true_m, "estimated_offset": est_m, "saturation": sat,
        "bias_of_estimate_ns": (est_m["bias_final_ns"] - true_m["bias_final_ns"])
        if est_m.get("bias_final_ns") == est_m.get("bias_final_ns") else None,
        "sim_wall_ms": res.wall_s * 1e3, "n_events": res.n_events,
    }
    metrics["metrics_wall_ms"] = (time.perf_counter() - t0) * 1e3
    return metrics
