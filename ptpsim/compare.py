# SPDX-License-Identifier: Apache-2.0
"""Reproducible baseline / variant comparison on deterministic and noisy scenarios.

Both controllers see exactly the same exogenous disturbances (same seed, indexed streams).
"""
from __future__ import annotations

import csv
import json
import time
from pathlib import Path

import numpy as np

from .config import SimConfig, noisy_preset
from .engine import simulate
from .metrics import MetricsConfig, compute_metrics

VARIANT_PARAMS = {"wn": 1.0, "zeta": 1.0, "sat_ppb": 400_000.0, "wn_ts_max": 0.35}
CONTROLLERS = {
    "baseline_pi": ("baseline_pi", {"kp": 0.7, "ki": 0.3}),
    "pi_time_aware": ("pi_time_aware", VARIANT_PARAMS),
}
MC = MetricsConfig(band_ns=1000.0, dwell_s=10.0, final_window_s=60.0)
# with path jitter the offset noise (~200 ns RMS) makes a 1 us band meaningless: use 3 sigma-ish
MC_NOISY = MetricsConfig(band_ns=2000.0, dwell_s=10.0, final_window_s=60.0)


def _scenario(kind: str, **kw) -> SimConfig:
    cfg = noisy_preset() if kind == "noisy" else SimConfig()
    cfg.duration_s = kw.pop("duration_s", 300.0)
    cfg.oscillator.initial_offset_ns = 100_000.0
    cfg.oscillator.freq_error_ppb = 20_000.0
    for k, v in kw.items():
        cfg = cfg.with_overrides(**{k: v})
    return cfg


def _with_controller(cfg: SimConfig, ctrl: str) -> SimConfig:
    name, params = CONTROLLERS[ctrl]
    return cfg.with_overrides(**{"controller.name": name, "controller.params": dict(params)})


def _row(cfg: SimConfig, label: dict, mc: MetricsConfig | None = None) -> dict:
    if mc is None:
        noisy = cfg.network.jitter_ms.kind != "none" or cfg.tx_jitter.sync.kind != "none"
        mc = MC_NOISY if noisy else MC
    res = simulate(cfg)
    m = compute_metrics(res, mc)
    t, e, s = m["true_offset"], m["estimated_offset"], m["saturation"]
    return {**label,
            "settle_true_s": t["settling_s"], "overshoot_true_pct": t["overshoot_pct"],
            "peak_true_ns": t["peak_abs_ns"], "rms_true_ns": t["rms_final_ns"],
            "bias_true_ns": t["bias_final_ns"], "settle_est_s": e["settling_s"],
            "rms_est_ns": e["rms_final_ns"], "bias_est_ns": e["bias_final_ns"],
            "diverged": t["diverged"], "resets": s["resets"], "range_resets": s["range_resets"],
            "clamped": s["controller_clamped"], "wall_ms": m["sim_wall_ms"]}


def run_comparison(out_dir: str | Path, seeds: int = 10, quick: bool = False) -> Path:
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    rows: list[dict] = []
    t0 = time.perf_counter()
    dur = 120.0 if quick else 300.0
    seeds = 3 if quick else seeds

    # 1. default scenarios
    for kind in ("deterministic", "noisy"):
        for ctrl in CONTROLLERS:
            for seed in (range(1, seeds + 1) if kind == "noisy" else [1]):
                cfg = _with_controller(_scenario(kind, duration_s=dur, seed=seed), ctrl)
                rows.append(_row(cfg, {"experiment": "default", "scenario": kind, "controller": ctrl,
                                       "sync_log": -2, "delay_log": 1, "actuator": "ideal", "seed": seed}))
    # 2. Sync interval sweep (Delay_Req interval fixed 2 s)
    for sl in (-4, -3, -2, -1, 0, 1, 2):
        for ctrl in CONTROLLERS:
            cfg = _with_controller(_scenario("deterministic", duration_s=max(dur, 600.0) if sl > 0 else dur,
                                             **{"intervals.sync_log": sl}), ctrl)
            rows.append(_row(cfg, {"experiment": "sync_sweep", "scenario": "deterministic",
                                   "controller": ctrl, "sync_log": sl, "delay_log": 1,
                                   "actuator": "ideal", "seed": 1}))
    # 3. Delay_Req interval sweep (Sync fixed 0.25 s)
    for dl in (-2, -1, 0, 1, 2, 3, 4):
        for ctrl in CONTROLLERS:
            cfg = _with_controller(_scenario("deterministic", duration_s=dur, **{"intervals.delay_log": dl}), ctrl)
            rows.append(_row(cfg, {"experiment": "delay_sweep", "scenario": "deterministic",
                                   "controller": ctrl, "sync_log": -2, "delay_log": dl,
                                   "actuator": "ideal", "seed": 1}))
    # 4. actuators
    for act, hz in (("ideal", 100_000_000), ("nxp", 100_000_000), ("nxp", 98_304_000), ("nxp", 24_000_000)):
        for ctrl in CONTROLLERS:
            cfg = _with_controller(_scenario("deterministic", duration_s=dur,
                                             **{"actuator.kind": act, "actuator.clock_hz": hz}), ctrl)
            rows.append(_row(cfg, {"experiment": "actuator", "scenario": "deterministic", "controller": ctrl,
                                   "sync_log": -2, "delay_log": 1, "actuator": f"{act}@{hz / 1e6:g}MHz",
                                   "seed": 1}))
    # 5. message loss (noisy network)
    for p in (0.0, 0.05, 0.2):
        for ctrl in CONTROLLERS:
            cfg = _with_controller(_scenario("noisy", duration_s=dur,
                                             **{"loss.sync": p, "loss.follow_up": p, "loss.delay_req": p,
                                                "loss.delay_resp": p}), ctrl)
            rows.append(_row(cfg, {"experiment": "loss", "scenario": f"noisy+loss{p:g}", "controller": ctrl,
                                   "sync_log": -2, "delay_log": 1, "actuator": "ideal", "seed": 1}))

    cols = list(rows[0].keys())
    with open(out / "comparison.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if v is None else v) for k, v in r.items()})
    _write_markdown(rows, out / "comparison.md", time.perf_counter() - t0, seeds, dur)
    (out / "comparison_setup.json").write_text(json.dumps({
        "controllers": {k: {"name": v[0], "params": v[1]} for k, v in CONTROLLERS.items()},
        "metrics": {"band_ns": MC.band_ns, "band_ns_noisy_scenarios": MC_NOISY.band_ns, "dwell_s": MC.dwell_s,
                    "final_window_s": MC.final_window_s},
        "initial_offset_ns": 100_000.0, "freq_error_ppb": 20_000.0, "duration_s": dur, "noisy_seeds": seeds,
    }, indent=2) + "\n")
    return out


def _fmt(v, nd=1):
    if v is None or (isinstance(v, float) and np.isnan(v)):
        return "–"
    return f"{v:.{nd}f}"


def _write_markdown(rows, path, wall, seeds, dur):
    L = ["# Baseline vs variant: quantitative comparison", "",
         f"Generated by `ptpsim-run compare` (wall {wall:.0f} s). Initial offset 100 µs, oscillator error "
         f"+20 ppm, duration {dur:g} s, settling band ±1 µs (±2 µs in the noisy scenarios) with 10 s dwell, "
         "final window 60 s. "
         "All offsets are the **true** offset slave − GM unless marked *est*. `–` = never settled / undefined.", ""]

    def table(title, exp, keycols, extra=()):
        sel = [r for r in rows if r["experiment"] == exp]
        L.extend([f"## {title}", ""])
        head = list(keycols) + ["controller", "settle [s]", "overshoot [%]", "peak [µs]", "RMS [ns]", "bias [ns]",
                                "resets", "diverged"] + list(extra)
        L.append("| " + " | ".join(head) + " |")
        L.append("|" + "---|" * len(head))
        for r in sel:
            vals = [str(r[k]) for k in keycols] + [r["controller"], _fmt(r["settle_true_s"], 2),
                                                    _fmt(r["overshoot_true_pct"], 1), _fmt(r["peak_true_ns"] / 1e3, 1),
                                                    _fmt(r["rms_true_ns"], 2), _fmt(r["bias_true_ns"], 2),
                                                    str(r["resets"]), "yes" if r["diverged"] else "no"]
            L.append("| " + " | ".join(vals) + " |")
        L.append("")

    # default: noisy aggregated
    L.extend(["## Default scenarios", ""])
    L.append("| scenario | controller | settle [s] | overshoot [%] | peak [µs] | RMS [ns] | bias [ns] | runs |")
    L.append("|---|---|---|---|---|---|---|---|")
    for kind in ("deterministic", "noisy"):
        for ctrl in CONTROLLERS:
            sel = [r for r in rows if r["experiment"] == "default" and r["scenario"] == kind
                   and r["controller"] == ctrl]
            def agg(key, nd):
                v = [r[key] for r in sel if r[key] is not None]
                if not v:
                    return "–"
                if len(v) < len(sel):
                    return f"{np.mean(v):.{nd}f} ({len(v)}/{len(sel)} settled)"
                return f"{np.mean(v):.{nd}f}" + (f" ± {np.std(v):.{nd}f}" if len(v) > 1 else "")
            L.append(f"| {kind} | {ctrl} | {agg('settle_true_s', 2)} | {agg('overshoot_true_pct', 1)} | "
                     f"{_fmt(np.mean([r['peak_true_ns'] for r in sel]) / 1e3, 1)} | {agg('rms_true_ns', 1)} | "
                     f"{agg('bias_true_ns', 1)} | {len(sel)} |")
    L.append("")
    table("Sync interval sweep (Delay_Req 2 s)", "sync_sweep", ["sync_log"])
    table("Delay_Req interval sweep (Sync 0.25 s)", "delay_sweep", ["delay_log"])
    table("Actuators", "actuator", ["actuator"])
    table("Message loss (noisy network)", "loss", ["scenario"])
    Path(path).write_text("\n".join(L) + "\n")
