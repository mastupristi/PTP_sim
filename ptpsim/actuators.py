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


class PllActuator:
    """Architecture B (``fsync.mode = shared``): the ENET 1588 timer and FSYNC are both clocked by the Audio PLL, and the
    servo command is realised by its ``NUM`` (``ptpsim/pll.py``).

    The timer runs at ``VCO / timer_div`` (196.608 MHz).  Its period is not a whole number of ns, so the driver programs
    ``ATINC``/``ATCOR`` once for ratio 1.0 at start (as ``NxpTimer`` does) and the correction is then left alone: the
    rate commands go to the PLL.  The average tick of that frozen correction is not exactly the nominal one, so the PHC
    keeps a constant residual rate ``timer_delta`` (-204.5 ppb at 196.608 MHz) that FSYNC does not have.

    ``adjust`` writes ``NUM`` as one or several writes (``step_policy``): ``steps`` holds, per write, the PHC deviation
    and the FSYNC deviation (both vs a nominal crystal); the engine applies them one lock time apart.  A ratio outside
    the 30-bit ``NUM`` range is rejected (-EINVAL), and the firmware resets the servo as with the NXP driver.
    """
    kind = "audio_pll"

    def __init__(self, cfg: ActuatorConfig, pll_cfg):
        from .pll import AudioPll
        self.pll = AudioPll(pll_cfg.fref_hz, pll_cfg.vco_hz, pll_cfg.den, pll_cfg.step_limit_hz)
        if pll_cfg.vco_hz % pll_cfg.timer_div:
            raise ValueError("VCO / timer_div is not a whole number of Hz")
        self.clock_hz = pll_cfg.vco_hz // pll_cfg.timer_div
        self.timer = fwport.NxpTimer(self.clock_hz, cfg.max_ratio_ppm)     # correction frozen at ratio 1.0
        self.timer_delta = self.timer.effective_delta
        self.policy = pll_cfg.step_policy
        self.lock_time_s = self.pll.lock_time_s
        self.timestamp_quantum_ns = self.timer.nominal_tick_ns
        self.num = self.pll.num0                     # last NUM written
        self.n_writes = 0
        self.n_capped = 0
        self.steps: list[tuple[float, float]] = [self._deltas(self.num)]
        self.effective_delta, self.fsync_delta = self.steps[-1]
        self.effective_ratio = 1.0 + self.effective_delta

    def _deltas(self, num: int) -> tuple[float, float]:
        dp = self.pll.delta_of(num)
        dt = self.timer_delta
        return dt + dp + dt * dp, dp

    def adjust(self, ratio: float) -> int:
        target = self.pll.num_for_ratio(ratio)
        if target is None:
            return -fwport.EINVAL
        writes = self.pll.ramp(self.num, target, self.policy)
        if writes[-1] != target:
            self.n_capped += 1
        self.n_writes += len(writes)
        self.num = writes[-1]
        self.steps = [self._deltas(n) for n in writes]
        self.effective_delta, self.fsync_delta = self.steps[-1]
        self.effective_ratio = 1.0 + self.effective_delta
        return 0

    def describe(self) -> dict:
        return {"kind": "audio_pll", "num": self.num, "clock_hz": self.clock_hz, "timer_delta_ppb": self.timer_delta * 1e9,
                "ratio": self.effective_ratio, **self.pll.describe()}


def make_actuator(cfg: ActuatorConfig, fsync_cfg=None):
    if fsync_cfg is not None and fsync_cfg.mode == "shared":
        return PllActuator(cfg, fsync_cfg.pll)
    if cfg.kind == "ideal":
        return IdealActuator(cfg)
    if cfg.kind == "nxp":
        return NxpActuator(cfg)
    raise ValueError(f"unknown actuator {cfg.kind!r}")
