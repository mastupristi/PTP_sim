# SPDX-License-Identifier: Apache-2.0
"""Audio PLL arithmetic (IMXRT1170RM 15.8: f_VCO = Fref * (DIV_SELECT + NUM / DENOM))."""
from fractions import Fraction

import pytest

from ptpsim.pll import NUM_MAX, NUM_MIN, AudioPll


def test_default_setting_is_representable_and_exact():
    p = AudioPll()                                         # 24 MHz, 786.432 MHz, DENOM 1e9
    # +768 000 000 does not fit 30 signed bits: DIV_SELECT 33 with a negative NUM
    assert (p.div_select, p.num0, p.den) == (33, -232_000_000, 1_000_000_000)
    assert NUM_MIN <= p.num0 <= NUM_MAX and 768_000_000 > NUM_MAX
    assert p.vco_of(p.num0) == 786_432_000
    assert p.vco_of(p.num0) / 4 == 196_608_000             # PTP timer root (<= 200 MHz)
    assert p.vco_of(p.num0) / 16_384 == 48_000             # FSYNC: 4096 timer ticks per frame
    assert p.lsb_hz == pytest.approx(0.024) and p.lsb_ppb == pytest.approx(1e9 / 32_768_000_000)
    assert p.num_per_ppb == pytest.approx(32.768)


def test_ppb_and_ratio_to_num_round_to_the_nearest_lsb():
    p = AudioPll()
    assert p.num_for_ppb(0.0) == p.num0
    assert p.num_for_ppb(1000.0) - p.num0 == 32_768        # 1 ppm = 32 768 LSB exactly
    assert p.num_for_ppb(0.01) == p.num0                   # 0.33 LSB -> 0
    assert p.num_for_ppb(0.02) == p.num0 + 1               # 0.66 LSB -> 1
    assert p.num_for_ratio(1.0 + 20e-6) == p.num0 + 655_360
    for ppb in (-3e6, -1234.5, 0.5, 777.0, 2e7):
        num = p.num_for_ppb(ppb)
        assert abs(p.delta_of(num) * 1e9 - ppb) <= p.lsb_ppb / 2 + 1e-12
        assert p.delta_of(num) == float(Fraction(num - p.num0, p.n0))   # correctly rounded, no 1 + x cancellation
        assert p.ratio_of(num) == pytest.approx(1.0 + ppb * 1e-9, abs=p.lsb_ppb * 1e-9)   # within one LSB


def test_range_is_the_30_bit_numerator_and_is_asymmetric():
    p = AudioPll()
    lo, hi = p.range_ppm
    assert lo == pytest.approx((NUM_MIN - p.num0) / p.n0 * 1e6)
    assert hi == pytest.approx((NUM_MAX - p.num0) / p.n0 * 1e6)
    assert -9_400 < lo < -9_200 and 23_400 < hi < 23_500
    assert p.num_for_ppb(-9_400_000.0) is None and p.num_for_ppb(23_500_000.0) is None
    assert p.num_for_ppb(float("nan")) is None and p.num_for_ratio(float("inf")) is None


def test_step_limit_policies():
    p = AudioPll()                                         # 200 kHz of VCO = 8 333 333 LSB (254 ppm at 786 MHz)
    assert p.max_step_num == 8_333_333 and p.lock_time_s == pytest.approx(200 / 24e6)
    small = p.num0 + 1_000_000
    for pol in ("off", "cap", "substep"):
        assert p.ramp(p.num0, small, pol) == [small]
    big = p.num0 + 20_000_000
    assert p.ramp(p.num0, big, "off") == [big]
    assert p.ramp(p.num0, big, "cap") == [p.num0 + 8_333_333]
    steps = p.ramp(p.num0, big, "substep")
    assert steps[-1] == big and len(steps) == 3
    prev = p.num0
    for s in steps:
        assert 0 < s - prev <= p.max_step_num
        prev = s
    down = p.ramp(big, p.num0, "substep")
    assert down[-1] == p.num0 and all(abs(b - a) <= p.max_step_num for a, b in zip([big] + down, down))


def test_settings_that_cannot_be_represented_are_rejected():
    with pytest.raises(ValueError):
        AudioPll(vco_hz=786_432_001, den=1000)              # not a multiple of 24 MHz / 1000
    with pytest.raises(ValueError):
        AudioPll(vco_hz=600_000_000)                       # VCO below 650 MHz
    with pytest.raises(ValueError):
        AudioPll(den=1 << 30)                              # DENOM does not fit 30 bits
    p = AudioPll(den=100_000_000)                          # the coarser 76.8e6 / 1e8 alternative
    assert (p.div_select, p.num0) == (33, -23_200_000) and p.num_per_ppb == pytest.approx(3.2768)   # nearest DIV_SELECT
