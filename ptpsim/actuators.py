# SPDX-License-Identifier: Apache-2.0
"""Rate actuators: translate the servo's rate-ratio request into the slave clock's rate.

Both actuators expose the same interface:

* ``adjust(ratio) -> int``   0 on success, negative errno on rejection (as the C driver);
  on success the new hardware state is *programmed* (``effective_ratio`` changes);
* ``effective_ratio``        time advanced by the clock per unit of physical time at a nominal
  oscillator (>1 = clock runs faster); ``effective_delta`` = ratio - 1 computed without the
  cancellation of ``1 + x`` (the simulator accumulates it over up to 1e6 s);
* ``timestamp_quantum_ns``   resolution of the timestamps (timer tick) or 0.

The ideal actuator realises any ratio exactly inside the same +-MAX_RATIO_PPM window as the
NXP driver (so a baseline comparison is fair: an out-of-window request is rejected and the
firmware resets the servo, exactly as with the NXP driver).
"""
from __future__ import annotations

from . import fwport
from .config import ActuatorConfig


class IdealActuator:
    kind = "ideal"

    def __init__(self, cfg: ActuatorConfig):
        self.max_ratio_ppm = cfg.max_ratio_ppm
        self.effective_ratio = 1.0
        self.effective_delta = 0.0
        self.timestamp_quantum_ns = 0.0

    def adjust(self, ratio: float) -> int:
        lim = self.max_ratio_ppm * 1.0e-6
        if ratio > 1.0 + lim or ratio < 1.0 - lim:
            return -fwport.EINVAL
        self.effective_ratio = ratio
        self.effective_delta = ratio - 1.0      # exact (Sterbenz) for ratio close to 1
        return 0

    def describe(self) -> dict:
        return {"kind": "ideal", "ratio": self.effective_ratio}


class NxpActuator:
    kind = "nxp"

    def __init__(self, cfg: ActuatorConfig):
        self.timer = fwport.NxpTimer(cfg.clock_hz, cfg.max_ratio_ppm)
        self._refresh()
        self.timestamp_quantum_ns = self.timer.nominal_tick_ns   # timestamps resolve to one timer tick

    def _refresh(self) -> None:
        t = self.timer
        self.effective_ratio = t.effective_ratio
        self.effective_delta = t.effective_delta

    def adjust(self, ratio: float) -> int:
        rc = self.timer.rate_adjust(ratio)
        if rc == 0:
            self._refresh()
        return rc

    def describe(self) -> dict:
        t = self.timer
        return {"kind": "nxp", "inc": t.inc, "inc_corr": t.inc_corr, "cor": t.cor,
                "ratio": self.effective_ratio}


def make_actuator(cfg: ActuatorConfig):
    if cfg.kind == "ideal":
        return IdealActuator(cfg)
    if cfg.kind == "nxp":
        return NxpActuator(cfg)
    raise ValueError(f"unknown actuator {cfg.kind!r}")
