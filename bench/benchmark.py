# SPDX-License-Identifier: Apache-2.0
"""Engine benchmark (headless).  Writes bench/results_engine.json and prints a table.

Run:  python bench/benchmark.py [--repeat 20]
Every number comes from an actual run on the machine described in the output.
"""
import argparse
import json
import platform
import statistics
import sys
import time
from pathlib import Path

import numpy as np

from ptpsim.config import SimConfig, noisy_preset
from ptpsim.engine import Simulation, simulate
from ptpsim.metrics import compute_metrics


def cases():
    d = SimConfig()
    d.oscillator.initial_offset_ns, d.oscillator.freq_error_ppb = 100_000.0, 20_000.0
    n = noisy_preset()
    nxp = d.with_overrides(**{"actuator.kind": "nxp"})
    nxp24 = d.with_overrides(**{"actuator.kind": "nxp", "actuator.clock_hz": 24_000_000})
    long = d.with_overrides(duration_s=3600.0)
    fast = d.with_overrides(**{"intervals.sync_log": -4})
    return {
        "300 s deterministic, ideal (default)": d,
        "300 s deterministic, NXP 100 MHz": nxp,
        "300 s deterministic, NXP 24 MHz": nxp24,
        "300 s noisy (jitter+net+ts noise), ideal": n,
        "300 s deterministic, Sync 2^-4 (16 Hz)": fast,
        "3600 s deterministic, ideal": long,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--repeat", type=int, default=20)
    a = ap.parse_args()
    out = {"machine": {"python": sys.version.split()[0], "platform": platform.platform(),
                       "processor": platform.processor() or platform.machine(), "numpy": np.__version__},
           "repeat": a.repeat, "cases": {}}
    print(f"python {out['machine']['python']}, numpy {np.__version__}, {out['machine']['platform']}")
    print(f"{'case':<46}{'sim median [ms]':>16}{'min':>8}{'p95':>8}{'+metrics [ms]':>14}{'events':>9}{'ev/s':>10}")
    for name, cfg in cases().items():
        simulate(cfg.with_overrides(duration_s=5.0))          # warm-up (imports, caches)
        t_sim, t_met, nev = [], [], 0
        for _ in range(a.repeat):
            t0 = time.perf_counter()
            res = simulate(cfg)
            t1 = time.perf_counter()
            compute_metrics(res)
            t2 = time.perf_counter()
            t_sim.append((t1 - t0) * 1e3)
            t_met.append((t2 - t1) * 1e3)
            nev = res.n_events
        s = sorted(t_sim)
        row = {"sim_median_ms": statistics.median(t_sim), "sim_min_ms": s[0],
               "sim_p95_ms": s[min(len(s) - 1, int(0.95 * len(s)))], "metrics_median_ms": statistics.median(t_met),
               "events": nev, "events_per_s": nev / (statistics.median(t_sim) / 1e3)}
        out["cases"][name] = row
        print(f"{name:<46}{row['sim_median_ms']:>16.1f}{row['sim_min_ms']:>8.1f}{row['sim_p95_ms']:>8.1f}"
              f"{row['metrics_median_ms']:>14.1f}{nev:>9d}{row['events_per_s']:>10.0f}")
    Path(__file__).with_name("results_engine.json").write_text(json.dumps(out, indent=2) + "\n")


if __name__ == "__main__":
    main()
