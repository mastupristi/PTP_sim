# SPDX-License-Identifier: Apache-2.0
from ptpsim.config import SimConfig


def quiet_cfg(**overrides) -> SimConfig:
    """Deterministic, noise-free scenario with 1 us symmetric delay (nonzero: mean_delay == 0
    means 'no delay yet' in the firmware and would keep the servo off)."""
    cfg = SimConfig()
    cfg.duration_s = 60.0
    return cfg.with_overrides(**overrides) if overrides else cfg


def no_control(cfg: SimConfig) -> SimConfig:
    """Baseline PI with kp = ki = 0: ppb = 0 -> ratio 1.0, i.e. an uncontrolled clock."""
    cfg.controller.name = "baseline_pi"
    cfg.controller.params = {"kp": 0.0, "ki": 0.0}
    return cfg
