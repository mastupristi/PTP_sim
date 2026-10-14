# SPDX-License-Identifier: Apache-2.0
"""Fractional-N Audio PLL of the i.MX RT1170 (IMXRT1170RM rev. 5, sections 14.4.3, 15.8, 15.9.5; AN14420).

    f_VCO = Fref * (DIV_SELECT + NUM / DENOM)

* ``Fref`` = 24 MHz (the crystal oscillator that feeds *every* PLL of the chip, so its error is common to the PTP timer
  clock and to the audio clocks);
* ``DIV_SELECT`` 27..54, VCO 650..1300 MHz;
* ``NUM`` signed 30 bits, ``DENOM`` unsigned 30 bits; RM: ``NUM/DENOM`` limited to -1..+1 (AN14420 says -2..+2: the
  stricter RM range is used);
* ``NUM`` is the only field that may change on the fly (RM 15.8: "PLL's input cannot be changed on-the-fly (except for
  MFN)"); AN14420: at most ``Fref/200`` writes per second (lock time <= 200 / Fref = 8.33 us) and no more than 200 kHz of
  VCO change per write without a significant glitch.

Everything here is exact integer arithmetic: the configuration is the integer ``n_total = DIV_SELECT * DENOM + NUM`` and
the VCO is ``Fref * n_total / DENOM``.  The rate ratio of a setting against the nominal one is ``n_total / n0``, its
deviation ``(NUM - NUM0) / n0`` -- exact, no ``1 + x`` cancellation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from fractions import Fraction

NUM_MIN, NUM_MAX = -(1 << 29), (1 << 29) - 1           # signed 30 bits
DEN_MAX = (1 << 30) - 1                                # unsigned 30 bits
DIV_MIN, DIV_MAX = 27, 54
VCO_MIN_HZ, VCO_MAX_HZ = 650_000_000, 1_300_000_000
FRAC_LIMIT = 1                                         # |NUM / DENOM| <= 1 (RM 15.8)


@dataclass(frozen=True)
class PllSetting:
    div_select: int
    num: int
    den: int

    @property
    def n_total(self) -> int:
        return self.div_select * self.den + self.num


class AudioPll:
    """Nominal setting for a target VCO frequency, and the ``NUM`` that realises a rate ratio.

    The nominal ``NUM/DENOM`` must represent ``vco_hz / fref_hz`` exactly with the requested ``DENOM``; ``DIV_SELECT`` is
    chosen so that ``NUM`` fits its signed 30 bits (for 786.432 MHz with ``DENOM`` = 1e9: ``DIV_SELECT`` = 33,
    ``NUM`` = -232 000 000, because +768 000 000 with ``DIV_SELECT`` = 32 does not fit).
    """

    def __init__(self, fref_hz: int = 24_000_000, vco_hz: int = 786_432_000, den: int = 1_000_000_000,
                 step_limit_hz: float = 200_000.0):
        if not 1 <= den <= DEN_MAX:
            raise ValueError(f"DENOM {den} does not fit 30 unsigned bits")
        if not VCO_MIN_HZ <= vco_hz <= VCO_MAX_HZ:
            raise ValueError(f"VCO {vco_hz} Hz outside {VCO_MIN_HZ}..{VCO_MAX_HZ}")
        self.fref_hz = int(fref_hz)
        self.vco_hz = int(vco_hz)
        self.den = int(den)
        n0 = Fraction(self.vco_hz * self.den, self.fref_hz)
        if n0.denominator != 1:
            raise ValueError(f"{vco_hz} Hz is not representable with DENOM {den} (Fref {fref_hz} Hz)")
        self.n0 = int(n0)
        self.nominal = self._split(self.n0)
        if self.nominal is None:
            raise ValueError(f"{vco_hz} Hz: no DIV_SELECT in {DIV_MIN}..{DIV_MAX} with a 30-bit NUM")
        self.div_select = self.nominal.div_select
        self.num0 = self.nominal.num
        # NUM range: 30 signed bits and |NUM/DENOM| <= 1
        self.num_min = max(NUM_MIN, -FRAC_LIMIT * self.den)
        self.num_max = min(NUM_MAX, FRAC_LIMIT * self.den)
        # 200 kHz of VCO per write -> in NUM LSB (1 LSB = Fref / DENOM Hz of VCO)
        self.step_limit_hz = float(step_limit_hz)
        self.max_step_num = int(math.floor(self.step_limit_hz * self.den / self.fref_hz)) if step_limit_hz > 0 else 0
        self.lock_time_s = 200.0 / self.fref_hz

    def _split(self, n_total: int) -> PllSetting | None:
        """``DIV_SELECT`` nearest to ``n_total / DENOM`` whose ``NUM`` fits 30 signed bits and |NUM/DENOM| <= 1."""
        q = n_total / self.den
        for div in sorted(range(DIV_MIN, DIV_MAX + 1), key=lambda d: abs(d - q)):
            num = n_total - div * self.den
            if NUM_MIN <= num <= NUM_MAX and abs(num) <= FRAC_LIMIT * self.den:
                return PllSetting(div, num, self.den)
        return None

    # ------------------------------------------------------------------ rates
    @property
    def lsb_hz(self) -> float:
        """VCO change of one ``NUM`` LSB."""
        return self.fref_hz / self.den

    @property
    def lsb_ppb(self) -> float:
        return 1e9 / self.n0

    @property
    def num_per_ppb(self) -> float:
        """``NUM`` LSB per ppb of rate (``n0 / 1e9``; 32.768 for 786.432 MHz with DENOM 1e9)."""
        return self.n0 / 1e9

    def vco_of(self, num: int) -> Fraction:
        return Fraction(self.fref_hz * (self.div_select * self.den + num), self.den)

    def delta_of(self, num: int) -> float:
        """Rate deviation of ``NUM`` against the nominal setting: ``(NUM - NUM0) / n0`` (exact)."""
        return (num - self.num0) / self.n0

    def ratio_of(self, num: int) -> float:
        return (self.div_select * self.den + num) / self.n0

    def num_for_ratio(self, ratio: float) -> int | None:
        """Nearest ``NUM`` for a rate ratio (``None`` outside the representable range).  ``ratio - 1`` is taken as is
        (the firmware hands over a ratio close to 1, see ``fwport.scaled_ppm_to_ratio``)."""
        if not math.isfinite(ratio):
            return None
        num = self.num0 + int(round((ratio - 1.0) * self.n0))
        if num < self.num_min or num > self.num_max:
            return None
        return num

    def num_for_ppb(self, ppb: float) -> int | None:
        if not math.isfinite(ppb):
            return None
        num = self.num0 + int(round(ppb * self.num_per_ppb))
        if num < self.num_min or num > self.num_max:
            return None
        return num

    @property
    def range_ppm(self) -> tuple[float, float]:
        """Reachable rate deviation (min, max) in ppm."""
        return self.delta_of(self.num_min) * 1e6, self.delta_of(self.num_max) * 1e6

    def ramp(self, num_from: int, num_to: int, policy: str) -> list[int]:
        """``NUM`` values written to reach ``num_to`` from ``num_from``.

        ``off``: one write, whatever the size; ``cap``: one write limited to ``max_step_num`` (the rest is *not*
        applied); ``substep``: as many writes of at most ``max_step_num`` as needed, one per lock time.
        """
        if policy == "off" or self.max_step_num <= 0 or abs(num_to - num_from) <= self.max_step_num:
            return [num_to]
        if policy == "cap":
            return [num_from + int(math.copysign(self.max_step_num, num_to - num_from))]
        if policy != "substep":
            raise ValueError(f"unknown step policy {policy!r}")
        n = -(-abs(num_to - num_from) // self.max_step_num)
        return [num_from + (num_to - num_from) * i // n for i in range(1, n + 1)]

    def describe(self) -> dict:
        return {"fref_hz": self.fref_hz, "vco_hz": self.vco_hz, "div_select": self.div_select, "num0": self.num0,
                "den": self.den, "lsb_hz": self.lsb_hz, "lsb_ppb": self.lsb_ppb, "num_per_ppb": self.num_per_ppb,
                "range_ppm": self.range_ppm, "max_step_num": self.max_step_num, "lock_time_s": self.lock_time_s}
