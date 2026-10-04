# SPDX-License-Identifier: Apache-2.0
"""Python ports of the firmware arithmetic that must stay bit-faithful.

Each function names the C original (file, function, commit) in its docstring and is
compared with the real C code, compiled from the sources vendored in ``tests/c_ref``,
by ``tests/test_c_reference.py``.

Reference: Zephyr branch ``zmagnifico-integration-2`` @ 553d973f1b78f64a9f7e49bbb2c8b7acee8701db.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

NSEC_PER_SEC = 1_000_000_000
SCALED_PPM_SHIFT = 16
SCALED_PPM_ONE = 1 << SCALED_PPM_SHIFT
EINVAL = 22
ENODEV = 19
ERANGE = 34


# --------------------------------------------------------------------------- PI

class PrecisionPI:
    """``subsys/precision_timing/precision_pi.c``.

    ``integral += ki * error; return kp * error + integral``: absolute output (ppb for an
    error in ns), no dt, integrator updated before the output is computed.
    """

    def __init__(self, kp: float, ki: float):
        self.kp = float(kp)
        self.ki = float(ki)
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0

    def update(self, error: float) -> float:
        self.integral += self.ki * error
        return self.kp * error + self.integral


# ---------------------------------------------------------------- ppb -> ratio

def ppb_to_scaled_ppm(ppb: float) -> int | None:
    """``precision_clock_ppb_to_scaled_ppm`` (precision_clock.c:30).

    Returns ``None`` where C returns ``-ERANGE``.  The C cast truncates toward zero.
    """
    if not math.isfinite(ppb):
        return None
    value = ppb * SCALED_PPM_ONE / 1000.0
    if not math.isfinite(value) or value < -9.223372036854775808e18 or value >= 9.223372036854775808e18:
        return None
    return int(value)  # Python int() truncates toward zero like the C cast


def scaled_ppm_to_ratio(scaled_ppm: int) -> float:
    """``precision_clock_ptp_adjust_rate`` (precision_clock_ptp.c:77)."""
    return 1.0 + float(scaled_ppm) / (1000000.0 * float(SCALED_PPM_ONE))


# --------------------------------------------------------- NXP correction math

def _try_period(delta, period, frac, cor_max, state):
    """``try_period`` of ptp_clock_nxp_enet_rate_math.c (state = [best_err, delta, cor])."""
    if period < 2 or period - 1 > cor_max:
        return False
    err = abs(delta / period - frac)
    if err >= state[0]:
        return False
    state[0] = err
    state[1] = delta
    state[2] = period - 1
    return True


def _find_best_delta_cor_scalar(frac, delta_max, cor_max):
    state = [math.inf, 0, 0]
    found = False
    for delta in range(1, delta_max + 1):
        period_exact = delta / frac
        if period_exact > cor_max:
            continue
        period_floor = int(math.floor(period_exact))
        found |= _try_period(delta, period_floor, frac, cor_max, state)
        found |= _try_period(delta, period_floor + 1, frac, cor_max, state)
    return found, state[1], state[2]


def _find_best_delta_cor(frac, delta_max, cor_max):
    """Vectorised equivalent of ``find_best_delta_cor``.

    The C loop keeps the first candidate with the strictly smallest error, scanning
    ``delta`` ascending and, for each ``delta``, the floor period before the ceil period.
    ``numpy.argmin`` returns the first minimum of the same interleaved order, so the
    result is identical (verified bit-exactly against the C code in the test-suite).
    """
    delta = np.arange(1, delta_max + 1, dtype=np.float64)
    period_exact = delta / frac
    ok = period_exact <= cor_max
    if not ok.any():
        return False, 0, 0
    delta = delta[ok]
    period_floor = np.floor(period_exact[ok])
    periods = np.empty(2 * delta.size)
    periods[0::2] = period_floor
    periods[1::2] = period_floor + 1.0
    deltas = np.repeat(delta, 2)
    valid = (periods >= 2.0) & (periods - 1.0 <= cor_max)
    if not valid.any():
        return False, 0, 0
    err = np.abs(deltas / np.where(valid, periods, 1.0) - frac)
    err[~valid] = np.inf
    i = int(np.argmin(err))
    if not math.isfinite(err[i]):
        return False, 0, 0
    return True, int(deltas[i]), int(periods[i]) - 1


def find_correction(inc: int, target_ns: float, inc_corr_max: int, cor_max: int,
                    vectorised: bool = True):
    """``ptp_clock_nxp_enet_find_correction``.

    Returns ``(rc, inc_corr, cor)``; on ``-EINVAL`` the outputs are ``None`` ("not modified").
    """
    frac = target_ns - float(inc)
    if frac == 0.0:
        return 0, inc, 0
    delta_max = (inc_corr_max - inc) if frac > 0.0 else inc
    if delta_max < 1:
        return -EINVAL, None, None
    fn = _find_best_delta_cor if vectorised else _find_best_delta_cor_scalar
    found, delta, cor = fn(abs(frac), delta_max, cor_max)
    if not found:
        return 0, inc, 0
    inc_corr = inc + delta if frac > 0.0 else inc - delta
    return 0, inc_corr, cor


# ------------------------------------------------------------- NXP timer model

ENET_ATINC_INC_MAX = 0x7F        # ENET_ATINC_INC_MASK >> SHIFT (7 bit)
ENET_ATINC_INC_CORR_MAX = 0x7F   # 7 bit field
ENET_ATCOR_COR_MAX = 0x7FFFFFFF  # ENET_ATCOR_COR_MASK


def rate_usable(rate_hz: int) -> bool:
    """``ptp_clock_nxp_enet_rate_usable`` (driver, PR #121108)."""
    if rate_hz == 0:
        return False
    inc = NSEC_PER_SEC // rate_hz
    return 1 <= inc <= ENET_ATINC_INC_MAX


@dataclass
class NxpTimer:
    """Register-level model of the ENET 1588 timer driven by ``ptp_clock_nxp_enet.c``.

    State is ``ATINC.INC`` (whole ns, truncated), ``ATINC.INC_CORR`` and ``ATCOR``.
    The *average* tick is ``inc + (inc_corr - inc) / (cor + 1)`` (``cor != 0``), see
    docs/firmware_reconstruction.md section 8.
    """

    clock_hz: int
    max_ratio_ppm: int = 50000
    inc: int = 0
    inc_corr: int = 0
    cor: int = 0

    def __post_init__(self):
        if not rate_usable(self.clock_hz):
            raise ValueError(f"1588 timer clock of {self.clock_hz} Hz is not usable")
        self.inc = NSEC_PER_SEC // self.clock_hz
        # Programmed once at timer start for ratio 1.0 (makes up the cut fractional tick)
        rc, ic, c = self._correction(1.0)
        if rc == 0:
            self.inc_corr, self.cor = ic, c
        else:
            self.inc_corr, self.cor = self.inc, 0

    def _correction(self, ratio: float):
        target_ns = float(NSEC_PER_SEC) / float(self.clock_hz) * ratio
        return find_correction(self.inc, target_ns, ENET_ATINC_INC_CORR_MAX, ENET_ATCOR_COR_MAX)

    def rate_adjust(self, ratio: float) -> int:
        """``ptp_clock_nxp_enet_rate_adjust``. Returns 0 or ``-EINVAL``."""
        if (ratio > 1.0 + self.max_ratio_ppm * 1.0e-6) or (ratio < 1.0 - self.max_ratio_ppm * 1.0e-6):
            return -EINVAL
        rc, ic, c = self._correction(ratio)
        if rc != 0:
            return rc
        self.inc_corr, self.cor = ic, c
        return 0

    @property
    def avg_tick_ns(self) -> float:
        if self.cor == 0:
            return float(self.inc)
        return float(self.inc) + float(self.inc_corr - self.inc) / float(self.cor + 1)

    @property
    def nominal_tick_ns(self) -> float:
        return float(NSEC_PER_SEC) / float(self.clock_hz)

    @property
    def effective_delta(self) -> float:
        """``effective_ratio - 1`` from exact integer terms (no 1 + x cancellation)."""
        hz = self.clock_hz
        n = float(self.inc * hz - NSEC_PER_SEC)
        if self.cor != 0:
            n += float(self.inc_corr - self.inc) * hz / float(self.cor + 1)
        return n / float(NSEC_PER_SEC)

    @property
    def effective_ratio(self) -> float:
        """Average tick over the exact nominal tick: time advanced per physical tick."""
        return self.avg_tick_ns / self.nominal_tick_ns
