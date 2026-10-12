# SPDX-License-Identifier: Apache-2.0
"""baseline_pi / pi_time_aware / pi_gain_scheduled on a few scenarios (same disturbances, same seeds).

    python bench/gain_scheduled.py            # prints a markdown table
Deterministic rows: one run; noisy rows: mean over seeds 1-5 (``noisy_preset``).  Settling: true offset inside the band
(1 us deterministic, 2 us noisy) for 10 s; RMS over the last 60 s of the 300 s run.
"""
import numpy as np

from ptpsim.compare import MC, MC_NOISY, _scenario
from ptpsim.engine import simulate
from ptpsim.metrics import compute_metrics

CONTROLLERS = {"baseline_pi": ("baseline_pi", {"kp": 0.7, "ki": 0.3}),
               "pi_time_aware": ("pi_time_aware", {}),
               "pi_gain_scheduled": ("pi_gain_scheduled", {})}
SCENARIOS = [("100 µs, Sync 250 ms", "det", {}),
             ("20 µs, Sync 250 ms", "det", {"oscillator.initial_offset_ns": 20_000.0}),
             ("100 µs, Sync 1 s", "det", {"intervals.sync_log": 0}),
             ("100 µs, Sync 62.5 ms", "det", {"intervals.sync_log": -4}),
             ("100 µs, noisy, Sync 250 ms", "noisy", {}),
             ("100 µs, noisy, Sync 1 s", "noisy", {"intervals.sync_log": 0}),
             ("100 µs, noisy, Sync 62.5 ms", "noisy", {"intervals.sync_log": -4})]


def run(kind, kw, name, params):
    rows = []
    for seed in ([1] if kind == "det" else range(1, 6)):
        cfg = _scenario(kind, **kw).with_overrides(**{"controller.name": name, "controller.params": dict(params)})
        if kind == "noisy":
            cfg = cfg.with_overrides(seed=seed)
        t = compute_metrics(simulate(cfg), MC if kind == "det" else MC_NOISY)["true_offset"]
        rows.append((np.nan if t["settling_s"] is None else t["settling_s"], t["overshoot_ns"], t["rms_final_ns"]))
    return np.nanmean(np.array(rows, float), axis=0)


def main():
    print("| scenario | controller | settling [s] | overshoot [µs] | RMS final [ns] |\n|---|---|---|---|---|")
    for label, kind, kw in SCENARIOS:
        for cname, (name, params) in CONTROLLERS.items():
            s, o, r = run(kind, kw, name, params)
            print(f"| {label} | `{cname}` | {s:.2f} | {o / 1e3:.1f} | {r:.1f} |")


if __name__ == "__main__":
    main()
