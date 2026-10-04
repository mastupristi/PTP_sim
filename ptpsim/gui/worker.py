# SPDX-License-Identifier: Apache-2.0
"""Simulation worker process.

A separate *process* (not a thread) is used because the engine is pure Python: a worker thread
would compete with the GUI thread for the GIL while computing.  The GUI sends requests through a
queue and a shared integer holds the newest request generation; the worker checks it between
chunks of simulated time (cooperative cancellation) and the GUI additionally discards any result
whose generation is not the current one.
"""
from __future__ import annotations

import time
import traceback

from ..config import SimConfig
from ..engine import Simulation
from ..export import export_result
from ..live import LiveSession, pack_result
from ..metrics import MetricsConfig, compute_metrics

CHUNK_S = 20.0


def _run_cancellable(sim: Simulation, t_end: float, gen: int, latest) -> bool:
    t = sim.now / 1e12
    while t < t_end:
        if latest.value != gen:
            return False
        t = min(t_end, t + CHUNK_S)
        sim.run_until(t)
    return True


def worker_main(req_q, res_q, latest) -> None:
    live: LiveSession | None = None
    pending = None
    while True:
        msg = pending if pending is not None else req_q.get()
        pending = None
        if msg["cmd"] == "quit":
            return
        # coalesce: only the newest explore request matters
        if msg["cmd"] == "explore":
            while True:
                try:
                    nxt = req_q.get_nowait()
                except Exception:
                    break
                if nxt["cmd"] == "explore" or nxt["cmd"] == "quit":
                    msg = nxt
                    if nxt["cmd"] == "quit":
                        return
                else:
                    pending = nxt
                    break
        gen = msg.get("gen", 0)
        t_start = time.perf_counter()
        try:
            if msg["cmd"] == "explore":
                cfg = SimConfig.from_dict(msg["cfg"])
                mc = MetricsConfig(**msg["mc"])
                out = {"type": "explore", "gen": gen}
                sim = Simulation(cfg)
                if not _run_cancellable(sim, cfg.duration_s, gen, latest):
                    res_q.put({"type": "cancelled", "gen": gen})
                    continue
                res = sim.result()
                out["main"] = pack_result(res)
                out["main"]["act"] = sim.act.describe()
                out["metrics"] = compute_metrics(res, mc)
                if msg.get("overlay") and cfg.controller.name != "baseline_pi":
                    bcfg = cfg.with_overrides(**{"controller.name": "baseline_pi",
                                                 "controller.params": {"kp": 0.7, "ki": 0.3}})
                    bsim = Simulation(bcfg)
                    if not _run_cancellable(bsim, cfg.duration_s, gen, latest):
                        res_q.put({"type": "cancelled", "gen": gen})
                        continue
                    bres = bsim.result()
                    out["base"] = pack_result(bres)
                    out["base_metrics"] = compute_metrics(bres, mc)
                out["compute_ms"] = (time.perf_counter() - t_start) * 1e3
                res_q.put(out)
            elif msg["cmd"] == "live_reset":
                cfg = SimConfig.from_dict(msg["cfg"])
                live = LiveSession(cfg, overlay=msg.get("overlay", False))
                res_q.put({"type": "live_reset", "gen": gen, "delta": live.delta(),
                           "overlay": live.overlay})
            elif msg["cmd"] == "live_advance" and live is not None:
                live.advance(msg["t_target"])
                res_q.put({"type": "live", "gen": gen, "delta": live.delta(),
                           "compute_ms": (time.perf_counter() - t_start) * 1e3})
            elif msg["cmd"] == "live_update" and live is not None:
                live.update(msg["overrides"], msg.get("policy", "keep"))
                res_q.put({"type": "live_updated", "gen": gen})
            elif msg["cmd"] == "export":
                cfg = SimConfig.from_dict(msg["cfg"])
                sim = Simulation(cfg)
                sim.run_until(cfg.duration_s)
                p = export_result(sim.result(), msg["dir"], mc=MetricsConfig(**msg["mc"]))
                res_q.put({"type": "exported", "path": str(p)})
        except Exception:  # noqa: BLE001 - report to the GUI instead of dying silently
            res_q.put({"type": "error", "gen": gen, "msg": traceback.format_exc()})
