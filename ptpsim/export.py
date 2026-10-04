# SPDX-License-Identifier: Apache-2.0
"""CSV / JSON export of results and parameters."""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from .engine import SimResult
from .metrics import MetricsConfig, compute_metrics, true_offset_grid

ACTIONS = {0: "pi", 1: "step", 2: "outlier_rejected", 3: "range_reset"}


def export_result(res: SimResult, out_dir: str | Path, dense_dt_s: float = 0.01,
                  mc: MetricsConfig | None = None) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    res.cfg.save(out / "params.json")
    (out / "metrics.json").write_text(json.dumps(compute_metrics(res, mc), indent=2, default=_json) + "\n")

    with open(out / "servo_samples.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_sample_s", "t_processed_s", "offset_est_ns", "offset_true_at_t2_ns",
                    "offset_true_at_processing_ns", "cmd_ppb", "integral", "action"])
        for i in range(res.servo_t_proc_s.size):
            w.writerow([f"{res.servo_t_sample_s[i]:.9f}", f"{res.servo_t_proc_s[i]:.9f}",
                        f"{res.servo_offset_est_ns[i]:.0f}", f"{res.servo_offset_true_ns[i]:.4f}",
                        f"{res.servo_offset_true_proc_ns[i]:.4f}", f"{res.servo_cmd_ppb[i]:.6f}",
                        f"{res.servo_integral[i]:.6f}", ACTIONS[int(res.servo_action[i])]])
    with open(out / "delay_samples.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_processed_s", "delay_est_ns", "delay_true_exchange_ns", "delay_true_nominal_ns"])
        for i in range(res.delay_t_proc_s.size):
            w.writerow([f"{res.delay_t_proc_s[i]:.9f}", f"{res.delay_est_ns[i]:.0f}",
                        f"{res.delay_true_sample_ns[i]:.3f}", f"{res.delay_true_nominal_ns:.3f}"])
    t, x = true_offset_grid(res, dense_dt_s)
    np.savetxt(out / "true_offset.csv", np.column_stack([t, x]), delimiter=",",
               header="t_s,offset_true_ns", comments="", fmt=["%.6f", "%.4f"])
    with open(out / "rate.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "cmd_ppb", "effective_rate_error_ppb"])
        for a, b, c in zip(res.rate_t_s, res.rate_cmd_ppb, res.rate_eff_ppb):
            w.writerow([f"{a:.9f}", f"{b:.6f}", f"{c:.6f}"])
    with open(out / "events.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "event", "detail"])
        for t_, k, d in res.events:
            w.writerow([f"{t_:.9f}", k, d])
        for t_, d in res.changes:
            w.writerow([f"{t_:.9f}", "parameter_change", d])
    return out


def _json(o):
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    raise TypeError(type(o))
