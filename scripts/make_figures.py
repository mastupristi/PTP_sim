# SPDX-License-Identifier: Apache-2.0
"""Figures for the README (needs matplotlib: pip install '.[docs]')."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from ptpsim.compare import CONTROLLERS, _scenario, _with_controller
from ptpsim.config import SimConfig
from ptpsim.engine import simulate

OUT = Path(__file__).resolve().parent.parent / "docs" / "img"
COL = {"baseline_pi": "#CC79A7", "pi_time_aware": "#0072B2"}


def fig_step_response():
    fig, axs = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    for ctrl in CONTROLLERS:
        r = simulate(_with_controller(_scenario("deterministic", duration_s=40.0), ctrl))
        t = np.linspace(0, 40, 8001)
        axs[0].plot(t, r.true_offset_ns(t) / 1e3, color=COL[ctrl], label=f"{ctrl}: true offset")
        axs[1].step(r.delay_t_proc_s, r.delay_est_ns / 1e3, where="post", color=COL[ctrl], label=f"{ctrl}: delay estimate")
    axs[1].axhline(1.0, color="k", ls="--", lw=1, label="physical delay (1 µs each way)")
    axs[0].set_ylabel("slave − GM offset [µs]")
    axs[1].set_ylabel("delay [µs]")
    axs[1].set_xlabel("physical (GM) time [s]")
    axs[0].set_title("Initial offset 100 µs, oscillator +20 ppm, Sync 0.25 s, Delay_Req 2 s, symmetric 1 µs network")
    axs[1].set_title("The delay estimate is biased by the clock dynamics even on a constant network")
    for a in axs:
        a.grid(alpha=.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "step_response.png", dpi=130)


def fig_sync_sweep():
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    sl = [-4, -3, -2, -1, 0, 1, 2]
    for ctrl in CONTROLLERS:
        ov, st = [], []
        for n in sl:
            cfg = _with_controller(_scenario("deterministic", duration_s=600.0, **{"intervals.sync_log": n}), ctrl)
            from ptpsim.metrics import compute_metrics
            m = compute_metrics(simulate(cfg))["true_offset"]
            ov.append(min(m["overshoot_pct"] or 0, 400))
            st.append(m["settling_s"] if m["settling_s"] is not None else np.nan)
        ax[0].plot(sl, ov, "o-", color=COL[ctrl], label=ctrl)
        ax[1].plot(sl, st, "o-", color=COL[ctrl], label=ctrl)
    ax[0].set_ylabel("overshoot [%] (capped at 400)")
    ax[1].set_ylabel("settling time [s]  (gap = never settled / diverged)")
    for a in ax:
        a.set_xlabel("Sync exponent n (interval = 2^n s), Delay_Req fixed at 2 s")
        a.grid(alpha=.3)
        a.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(OUT / "sync_sweep.png", dpi=130)


def fig_nxp24():
    fig, ax = plt.subplots(figsize=(9, 3.6))
    for hz, c in ((100_000_000, "#0072B2"), (98_304_000, "#009E73"), (24_000_000, "#D55E00")):
        cfg = _scenario("deterministic", duration_s=120.0, **{"actuator.kind": "nxp", "actuator.clock_hz": hz})
        r = simulate(cfg)
        t = np.linspace(20, 120, 10001)
        ax.plot(t, r.true_offset_ns(t) / 1e3, color=c, lw=1, label=f"NXP {hz / 1e6:g} MHz")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("true offset [µs]")
    ax.set_title("Baseline PI, +20 ppm oscillator error: rate granularity of the ENET timer")
    ax.grid(alpha=.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT / "nxp_roots.png", dpi=130)


if __name__ == "__main__":
    OUT.mkdir(parents=True, exist_ok=True)
    fig_step_response()
    fig_sync_sweep()
    fig_nxp24()
    print("figures written to", OUT)
