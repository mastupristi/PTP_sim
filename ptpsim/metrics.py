# SPDX-License-Identifier: Apache-2.0
"""Synthetic metrics of a trajectory.

All metrics are computed on the full-resolution data (never on down-sampled render data).
The *true* offset is the physical offset slave - GM evaluated on a dense grid plus the exact
piecewise-linear vertices; the *estimated* offset is the firmware's estimate at its samples.

Definitions
-----------
* **Servo start**: the first PI command (after the last forced alignment, if the offset was beyond 1 s).  The servo only starts after the first valid delay sample (the
  first Delay_Req is sent at a random time in (0, 2*2^n] s), so up to a few seconds elapse during which the
  clock runs free (e.g. 100 us + 20 ppm * 3 s = 160 us).  Both controllers share this phase.
* **Transient end** = settling time of the true offset (band/dwell below); **steady state** = the last
  ``final_window_s`` seconds, or everything after the transient end if ``steady_from_settling``.
  Steady-state statistics (median, min, max, mean, std, RMS, median |x|, max |x|, peak-to-peak) are given for
  the true offset, the estimated offset and the delay estimate.
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
    steady_from_settling: bool = False   # steady state = after the transient end (else: last final_window_s)


def true_offset_grid(res: SimResult, dt_s: float = 0.01, max_points: int = 400_000):
    t_end = res.t_end_s
    n = int(min(max_points, max(2, t_end / dt_s + 1)))
    t = np.linspace(0.0, t_end, n)
    return t, res.true_offset_ns(t)


def window_stats(x: np.ndarray) -> dict:
    """Steady-state statistics of a window of samples."""
    if x.size == 0:
        return {k: float("nan") for k in ("median_ns", "min_ns", "max_ns", "mean_ns", "std_ns", "rms_ns",
                                           "median_abs_ns", "max_abs_ns", "p2p_ns")}
    return {"median_ns": float(np.median(x)), "min_ns": float(x.min()), "max_ns": float(x.max()),
            "mean_ns": float(x.mean()), "std_ns": float(x.std()), "rms_ns": float(np.sqrt(np.mean(x ** 2))),
            "median_abs_ns": float(np.median(np.abs(x))), "max_abs_ns": float(np.abs(x).max()),
            "p2p_ns": float(x.max() - x.min())}


def _series_metrics(t: np.ndarray, x: np.ndarray, x0: float, t_end: float, mc: MetricsConfig,
                    peak_abs: float | None = None, x_ref: float | None = None, t_from: float = 0.0) -> dict:
    """``x0``: offset at t = 0; ``x_ref``/``t_from``: reference value and instant (servo start)."""
    if x_ref is None:
        x_ref = x0
    out: dict = {"n": int(t.size)}
    if t.size == 0 or not np.all(np.isfinite(x)):
        out.update(settling_s=None, settling_code="nodata" if t.size == 0 else "nonfinite",
                   settling_reason="no data" if t.size == 0 else "non-finite values",
                   overshoot_pct=None, overshoot_ns=None, overshoot_vs_initial_pct=None,
                   peak_abs_ns=float("nan"), peak_excess_ns=float("nan"), steady_start_s=None,
                   rms_final_ns=float("nan"), bias_final_ns=float("nan"), diverged=True, **window_stats(x[:0]))
        return out
    ax = np.abs(x)
    after = t >= t_from
    peak = float(ax[after].max()) if after.any() else float(ax.max())
    if peak_abs is not None:
        peak = float(max(peak_abs, peak))
    out["peak_abs_ns"] = peak
    out["peak_overall_ns"] = float(ax.max())
    out["peak_excess_ns"] = peak - abs(x_ref)
    # --- settling (transient end)
    outside = np.nonzero(ax > mc.band_ns)[0]
    if outside.size == 0:
        t_settle, code = float(t[0]) if t[0] > 0 else 0.0, "in_band"
    elif outside[-1] == t.size - 1:
        t_settle, code = None, "outside_end"
    else:
        t_settle, code = float(t[outside[-1] + 1]), "ok"
    if t_settle is not None and (t_end - t_settle) < mc.dwell_s:
        t_settle, code = None, "dwell"
    reasons = {"ok": "ok", "in_band": "within band from the start",
               "outside_end": "still outside the band at the end of the run",
               "dwell": f"inside the band for less than the dwell time ({mc.dwell_s:g} s)"}
    out["settling_s"], out["settling_code"], out["settling_reason"] = t_settle, code, reasons[code]
    # --- overshoot (reference: offset at the servo start)
    if x_ref == 0 or not after.any():
        out["overshoot_pct"], out["overshoot_ns"] = None, None
    else:
        ov = float(max(0.0, (-np.sign(x_ref) * x[after]).max()))
        out["overshoot_ns"], out["overshoot_pct"] = ov, ov / abs(x_ref) * 100.0
    out["overshoot_ref_ns"], out["overshoot_ref_time_s"] = float(x_ref), float(t_from)
    if x0 == 0 or abs(x0) > 1e12:
        out["overshoot_vs_initial_pct"] = None
    else:
        out["overshoot_vs_initial_pct"] = float(max(0.0, (-np.sign(x0) * x).max()) / abs(x0) * 100.0)
    # --- steady state window
    w = min(mc.final_window_s, t_end)
    t_ss = t_settle if (mc.steady_from_settling and t_settle is not None) else t_end - w
    out["steady_start_s"] = float(t_ss)
    m = t >= t_ss
    xs = x[m] if m.any() else x[-1:]
    out["window_s"] = float(t_end - t_ss)
    out.update(window_stats(xs))
    out["rms_final_ns"], out["bias_final_ns"], out["std_final_ns"] = out["rms_ns"], out["mean_ns"], out["std_ns"]
    scale = max(abs(x0) if abs(x0) < 1e12 else 0.0, abs(x_ref), mc.band_ns)
    big_final = out["rms_final_ns"] > scale and out["rms_final_ns"] > 10 * mc.band_ns
    out["diverged"] = bool(peak > mc.divergence_ns or (t_end > 2 * w and big_final))
    return out


def compute_metrics(res: SimResult, mc: MetricsConfig | None = None) -> dict:
    mc = mc or MetricsConfig()
    t0 = time.perf_counter()
    x0 = res.cfg.oscillator.initial_offset_ns
    tg, xg = true_offset_grid(res, mc.grid_dt_s)
    # servo start = first PI command after the last forced alignment (clock step)
    steps = np.nonzero(res.servo_action == 1)[0]
    t_last_step = float(res.servo_t_proc_s[steps[-1]]) if steps.size else -1.0
    act = np.nonzero((res.servo_action == 0) & (res.servo_t_proc_s > t_last_step))[0]
    t_servo = float(res.servo_t_proc_s[act[0]]) if act.size else 0.0
    x_ref = float(res.true_offset_ns(np.array([t_servo]))[0]) if act.size else x0
    # exact peak over the piecewise-linear vertices from the servo start on
    t0s, p0, ends, pend = res.clock.segment_end_values(int(round(res.t_end_s * PS_PER_S)))
    keep = (t0s / PS_PER_S) >= t_servo
    peak_true = float(max(np.abs(p0[keep]).max(), np.abs(pend[keep]).max())) if keep.any() else 0.0
    true_m = _series_metrics(tg, xg, x0, res.t_end_s, mc, peak_abs=peak_true, x_ref=x_ref, t_from=t_servo)

    est_t = res.servo_t_sample_s
    est_x = res.servo_offset_est_ns
    ok = np.isfinite(est_x)
    est_m = _series_metrics(est_t[ok], est_x[ok], x0, res.t_end_s, mc, x_ref=x_ref, t_from=t_servo)

    # delay estimate statistics over the same steady-state window as the true offset
    t_ss = true_m["steady_start_s"] if true_m.get("steady_start_s") is not None else res.t_end_s - mc.final_window_s
    dm = res.delay_t_proc_s >= t_ss
    delay_m = window_stats(res.delay_est_ns[dm])
    delay_m["n"] = int(dm.sum())
    delay_m["nominal_ns"] = res.delay_true_nominal_ns
    delay_m["error_median_ns"] = delay_m["median_ns"] - res.delay_true_nominal_ns if dm.any() else float("nan")

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
        "transient_end_s": true_m["settling_s"], "steady_start_s": true_m["steady_start_s"],
        "config_band_ns": mc.band_ns, "dwell_s": mc.dwell_s, "final_window_s": mc.final_window_s,
        "true_offset": true_m, "estimated_offset": est_m, "delay": delay_m, "saturation": sat,
        "bias_of_estimate_ns": (est_m["bias_final_ns"] - true_m["bias_final_ns"])
        if est_m.get("bias_final_ns") == est_m.get("bias_final_ns") else None,
        "sim_wall_ms": res.wall_s * 1e3, "n_events": res.n_events,
    }
    metrics["metrics_wall_ms"] = (time.perf_counter() - t0) * 1e3
    return metrics
