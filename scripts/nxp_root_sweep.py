# SPDX-License-Identifier: Apache-2.0
"""Median |offset| vs oscillator error for several NXP clock roots (noisy preset), baseline PI.

Context: the commit message of Zephyr PR #121108 reports a median |offset| of 236 / 181 / 144 / 152 ns on
hardware for 24 / 98.304 / 100 / 196.608 MHz.  The simulator cannot know the real oscillator error between the
boards nor the real network noise; this sweep shows which relative frequency errors would be consistent with
those numbers *in the model*.  Run: python scripts/nxp_root_sweep.py [--seeds 5]
"""
import argparse
from pathlib import Path

import numpy as np

from ptpsim.config import noisy_preset
from ptpsim.engine import simulate

ROOTS = [("24 MHz", 24_000_000), ("98.304 MHz", 98_304_000), ("100 MHz", 100_000_000), ("196.608 MHz", 196_608_000)]
EPS_PPM = [0.0, 0.001, 0.003, 0.01, 0.03, 0.1, 0.5, 2.0, 20.0]
PR_MEDIAN_NS = {"24 MHz": 236, "98.304 MHz": 181, "100 MHz": 144, "196.608 MHz": 152}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "results" / "nxp_root_sweep.md"))
    a = ap.parse_args()
    lines = ["# Median |offset| [ns] vs oscillator error, NXP actuator, noisy preset, baseline PI",
             "", f"Last 200 s of 300 s, median over {a.seeds} seeds of the median |true offset|. "
             "Last row: values reported on hardware in the commit message of PR #121108 (conditions unknown).", "",
             "| clock root | " + " | ".join(f"{e:g} ppm" for e in EPS_PPM) + " |", "|---|" + "---|" * len(EPS_PPM)]
    for name, hz in ROOTS:
        row = []
        for eps in EPS_PPM:
            med = []
            for seed in range(1, a.seeds + 1):
                cfg = noisy_preset()
                cfg.seed = seed
                cfg.duration_s = 300.0
                cfg.oscillator.initial_offset_ns = 0.0
                cfg.oscillator.freq_error_ppb = eps * 1e3
                cfg.actuator.kind, cfg.actuator.clock_hz = "nxp", hz
                r = simulate(cfg)
                t = np.linspace(100.0, 300.0, 20001)
                med.append(np.median(np.abs(r.true_offset_ns(t))))
            row.append(f"{np.median(med):.0f}")
        lines.append(f"| {name} | " + " | ".join(row) + " |")
    lines.append("| PR #121108 (hardware) | " + " | ".join(f"{k}: {v}" for k, v in PR_MEDIAN_NS.items()) + " |")
    Path(a.out).write_text("\n".join(lines) + "\n")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
