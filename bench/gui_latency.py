# SPDX-License-Identifier: Apache-2.0
"""GUI reactivity benchmark (offscreen Qt, software rendering).

Measures, for a 300 s deterministic scenario with the default configuration, the time from a
parameter change (spin box value change, as a slider drag emits) to the moment the new trajectory is
drawn, split into debounce + worker + IPC/draw.  Also measures how long the GUI thread is blocked
(longest gap between two timer ticks of a 5 ms heartbeat) while the worker computes.

Run:  QT_QPA_PLATFORM=offscreen python bench/gui_latency.py [--trials 40]
"""
import argparse
import json
import os
import platform
import statistics
import sys
import time
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
from PySide6 import QtCore, QtWidgets  # noqa: E402

from ptpsim.config import SimConfig  # noqa: E402
from ptpsim.gui.app import MainWindow  # noqa: E402


def wait(app, cond, tmo=30.0):
    t0 = time.perf_counter()
    while not cond() and time.perf_counter() - t0 < tmo:
        app.processEvents()
        time.sleep(0.001)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=40)
    ap.add_argument("--scenario", choices=["deterministic", "nxp", "overlay"], default="deterministic")
    a = ap.parse_args()
    app = QtWidgets.QApplication([])
    cfg = SimConfig()
    cfg.oscillator.initial_offset_ns, cfg.oscillator.freq_error_ppb = 100_000.0, 20_000.0
    if a.scenario == "nxp":
        cfg.actuator.kind = "nxp"
    w = MainWindow(cfg)
    if a.scenario == "overlay":
        w.ctrl_combo.setCurrentText("pi_time_aware")
        w.chk_overlay.setChecked(True)
    w.show()
    wait(app, lambda: w.n_results >= 1)
    # heartbeat: longest stall of the GUI thread
    gaps, last = [], [time.perf_counter()]

    def beat():
        now = time.perf_counter()
        gaps.append(now - last[0])
        last[0] = now
    hb = QtCore.QTimer()
    hb.setInterval(5)
    hb.timeout.connect(beat)
    hb.start()
    for _ in range(5):                                  # warm-up trials
        n0 = w.n_results
        w.rows["ctrl.kp" if "ctrl.kp" in w.rows else "ctrl.wn"].spin.setValue(0.5)
        wait(app, lambda: w.n_results > n0)
    lat, comp, sim = [], [], []
    row = w.rows["ctrl.kp" if "ctrl.kp" in w.rows else "ctrl.wn"]
    values = [0.4 + 0.01 * (i % 20) for i in range(a.trials)]
    gaps.clear()
    for v in values:
        n0 = w.n_results
        row.spin.setValue(v)
        wait(app, lambda: w.n_results > n0)
        lat.append(w.last_ms["gui"])
        comp.append(w.last_ms["compute"])
        sim.append(w.last_ms["sim"])
        wait(app, lambda: False, 0.02)
    hb.stop()
    s = sorted(lat)
    out = {"scenario": a.scenario, "trials": a.trials,
           "latency_ms": {"median": statistics.median(lat), "p95": s[int(0.95 * (len(s) - 1))], "max": s[-1], "min": s[0]},
           "worker_compute_ms_median": statistics.median(comp), "sim_ms_median": statistics.median(sim),
           "debounce_ms": MainWindow.DEBOUNCE_MS,
           "gui_thread_max_gap_ms": max(gaps) * 1e3, "gui_thread_p99_gap_ms": sorted(gaps)[int(0.99 * (len(gaps) - 1))] * 1e3,
           "machine": {"python": sys.version.split()[0], "platform": platform.platform(), "qt": QtCore.qVersion()}}
    print(json.dumps(out, indent=2))
    Path(__file__).with_name(f"results_gui_{a.scenario}.json").write_text(json.dumps(out, indent=2) + "\n")
    w.close()


if __name__ == "__main__":
    main()
