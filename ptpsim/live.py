# SPDX-License-Identifier: Apache-2.0
"""Incremental (live) view of a running simulation, plus packing helpers shared with the GUI.

The simulation maths is independent of how the run is chunked: ``Simulation.run_until`` only
processes events up to the requested time, so playback speed and chunk size never change results.
"""
from __future__ import annotations

import numpy as np

from .config import FirmwareConfig, SimConfig
from .controllers import REGISTRY
from .engine import PS_PER_S, SimResult, Simulation
from .metrics import MetricsConfig, compute_metrics, true_offset_grid


def overlay_config(cfg: SimConfig) -> SimConfig | None:
    """Reference run drawn by the baseline overlay: the unmodified firmware (baseline PI 0.7/0.3,
    ``clock.c`` constants, no command clamp) on the same scenario and seed.

    ``None`` when the main run already uses the baseline PI with unmodified firmware settings (the
    overlay would add nothing, as before the firmware options existed)."""
    if cfg.controller.name == "baseline_pi" and cfg.firmware == FirmwareConfig():
        return None
    b = cfg.with_overrides(**{"controller.name": "baseline_pi", "controller.params": {"kp": 0.7, "ki": 0.3}})
    b.firmware = FirmwareConfig()
    return b


def command_limits(cfg: SimConfig) -> dict[str, float]:
    """Limits drawn on the PI-terms plot, in ppm (0 = not active)."""
    cls = REGISTRY[cfg.controller.name]
    params = {k: v[0] for k, v in cls.PARAMS.items()}
    params.update(cfg.controller.params)
    return {"actuator": float(cfg.actuator.max_ratio_ppm),
            "clamp": float(cfg.firmware.cmd_clamp_ppm),
            "i_max": float(params.get("i_max_ppm", 0.0)),
            "sat": float(params.get("sat_ppb", 0.0)) / 1000.0}


def _pi_terms(t_proc, p_ppb, i_ppb, out_ppb) -> dict:
    """P, I and controller output at the updates where the controller ran (not steps/outliers)."""
    ran = np.isfinite(p_ppb)
    return {"pi_t": t_proc[ran], "pi_p": p_ppb[ran], "pi_i": i_ppb[ran], "pi_out": out_ppb[ran]}


def pack_result(res: SimResult, dense_dt_s: float | None = None) -> dict:
    """Arrays (and a few scalars) needed to draw and describe a full trajectory."""
    if dense_dt_s is None:
        dense_dt_s = max(0.01, res.t_end_s / 60_000.0)
    t, x = true_offset_grid(res, dense_dt_s)
    ok = np.isfinite(res.servo_offset_est_ns)
    return {
        "t_end": res.t_end_s,
        "true_t": t, "true_x": x,
        "est_t": res.servo_t_sample_s[ok], "est_x": res.servo_offset_est_ns[ok],
        "est_action": res.servo_action[ok],
        "delay_t": res.delay_t_proc_s, "delay_x": res.delay_est_ns,
        "delay_true_sample": res.delay_true_sample_ns, "delay_nominal": res.delay_true_nominal_ns,
        "rate_t": res.rate_t_s, "rate_cmd": res.rate_cmd_ppb, "rate_eff": res.rate_eff_ppb,
        **_pi_terms(res.servo_t_proc_s, res.servo_p_ppb, res.servo_integral, res.servo_cmd_ppb),
        "limits": command_limits(res.cfg),
        "events": [(t_, k) for (t_, k, _d) in res.events if k in ("step", "servo_reset", "locked")],
        "changes": list(res.changes), "counters": dict(res.counters),
        "wall_ms": res.wall_s * 1e3,
    }


class LiveSession:
    """Main simulation (+ optional baseline overlay) advanced on demand, with delta extraction."""

    def __init__(self, cfg: SimConfig, overlay: bool = False, dense_dt_s: float = 0.01):
        self.dense_dt = dense_dt_s
        self.cfg = cfg.copy()
        self.sims: dict[str, Simulation] = {"main": Simulation(cfg)}
        base_cfg = overlay_config(cfg) if overlay else None
        self.overlay = base_cfg is not None
        if self.overlay:
            self.sims["base"] = Simulation(base_cfg)
        self._cursor: dict[str, dict] = {k: dict(servo=0, delay=0, rate=0, ev=0, ch=0, t_dense=0.0) for k in self.sims}

    @property
    def t(self) -> float:
        return self.sims["main"].now / PS_PER_S

    def advance(self, t_target: float) -> None:
        for s in self.sims.values():
            s.run_until(t_target)

    def update(self, overrides: dict, policy: str) -> None:
        self.sims["main"].update_config(overrides, policy)
        if "base" in self.sims:
            # the overlay stays the unmodified firmware: controller and firmware options are not forwarded
            ov = {k: v for k, v in overrides.items() if not k.startswith(("controller.", "firmware."))}
            if ov:
                self.sims["base"].update_config(ov, "keep")

    def delta(self) -> dict:
        """New data since the previous call (O(new samples), the engine lists are sliced)."""
        out = {}
        arr = np.array
        for key, sim in self.sims.items():
            cur = self._cursor[key]
            n = len(sim._s_t_proc)
            t_samp = arr(sim._s_t_samp[cur["servo"]:n])
            pi = _pi_terms(arr(sim._s_t_proc[cur["servo"]:n]), arr(sim._s_p[cur["servo"]:n]),
                           arr(sim._s_int[cur["servo"]:n]), arr(sim._s_cmd[cur["servo"]:n]))
            est = arr(sim._s_off[cur["servo"]:n], dtype=np.float64)
            t_now = sim.now / PS_PER_S
            t_new = np.arange(cur["t_dense"], t_now, self.dense_dt)
            if t_new.size:
                x_new = sim.clock.sample_ns(np.round(t_new * PS_PER_S).astype(np.int64))
                cur["t_dense"] = float(t_new[-1] + self.dense_dt)
            else:
                x_new = np.empty(0)
            nd, nr = len(sim._d_t), len(sim._r_t)
            out[key] = {
                "t_now": t_now,
                "true_t": t_new, "true_x": x_new,
                "est_t": t_samp, "est_x": est,
                "delay_t": arr(sim._d_t[cur["delay"]:nd]), "delay_x": arr(sim._d_est[cur["delay"]:nd], dtype=np.float64),
                "delay_nominal": (sim.cfg.network.delay_ms_ns + sim.cfg.network.delay_sm_ns) / 2.0,
                "rate_t": arr(sim._r_t[cur["rate"]:nr]), "rate_cmd": arr(sim._r_cmd[cur["rate"]:nr]),
                "rate_eff": arr(sim._r_eff[cur["rate"]:nr]),
                **pi, "limits": command_limits(sim.cfg),
                "events": [(t_, k) for (t_, k, _d) in sim.events[cur["ev"]:] if k in ("step", "servo_reset", "locked")],
                "changes": sim.changes[cur["ch"]:],
                "counters": dict(sim.counters),
            }
            cur.update(servo=n, delay=nd, rate=nr, ev=len(sim.events), ch=len(sim.changes))
        return out
