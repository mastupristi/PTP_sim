# SPDX-License-Identifier: Apache-2.0
"""Bit-exact comparison of the Python ports with the real Zephyr C code (vendored in c_ref)."""
import ctypes
import math
import random
import shutil
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import pytest

from ptpsim import fwport

C_REF = Path(__file__).parent / "c_ref"

pytestmark = pytest.mark.skipif(shutil.which("gcc") is None, reason="gcc not available")


@pytest.fixture(scope="module")
def lib():
    tmp = Path(tempfile.mkdtemp(prefix="ptpsim_cref_"))
    inc = tmp / "inc" / "zephyr" / "precision_timing"
    inc.mkdir(parents=True)
    (inc / "precision_time.h").write_text(
        "#include <stdint.h>\ntypedef int64_t precision_time_t;\n#define PRECISION_TIME_MAX INT64_MAX\n")
    for h in ("precision_pi.h", "precision_clock.h"):
        shutil.copy(C_REF / h, inc / h)
    so = tmp / "libref.so"
    cmd = ["gcc", "-O2", "-shared", "-fPIC", "-std=gnu11", "-ffp-contract=off",
           "-I", str(tmp / "inc"), "-I", str(C_REF),
           str(C_REF / "ref_wrapper.c"), str(C_REF / "precision_pi.c"),
           str(C_REF / "precision_clock.c"), str(C_REF / "ptp_clock_nxp_enet_rate_math.c"),
           "-lm", "-o", str(so)]
    subprocess.run(cmd, check=True)
    dll = ctypes.CDLL(str(so))
    dll.ref_pi_run.restype = ctypes.c_double
    dll.ref_pi_run.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.POINTER(ctypes.c_double),
                               ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                               ctypes.POINTER(ctypes.c_double)]
    dll.ref_ppb_to_scaled_ppm.restype = ctypes.c_int
    dll.ref_ppb_to_scaled_ppm.argtypes = [ctypes.c_double, ctypes.POINTER(ctypes.c_int64)]
    dll.ref_find_correction.restype = ctypes.c_int
    dll.ref_find_correction.argtypes = [ctypes.c_int, ctypes.c_double, ctypes.c_int, ctypes.c_uint32,
                                        ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_uint32)]
    yield dll
    shutil.rmtree(tmp, ignore_errors=True)


def test_pi_matches_c(lib):
    rng = np.random.default_rng(1)
    err = rng.normal(0, 5e4, 500)
    err[::50] *= 1e3
    out = (ctypes.c_double * err.size)()
    integ = ctypes.c_double()
    e_arr = (ctypes.c_double * err.size)(*err)
    for kp, ki in [(0.7, 0.3), (0.0, 0.5), (3.3, 0.01), (10.0, 10.0)]:
        lib.ref_pi_run(kp, ki, e_arr, out, err.size, ctypes.byref(integ))
        pi = fwport.PrecisionPI(kp, ki)
        for i, e in enumerate(err):
            assert pi.update(float(e)) == out[i]  # exact equality
        assert pi.integral == integ.value


def test_ppb_to_scaled_ppm_matches_c(lib):
    cases = [0.0, 1.0, -1.0, 0.999, -0.999, 1e3, -1e3, 12345.678, -98765.4321, 5e4, 1e12, -1e12,
             1e15, 1e18, 1e19, -1e19, 1e30, float("inf"), float("-inf"), float("nan"), 0.01526,
             -0.01526, 1.5258789e-2]
    rnd = random.Random(2)
    cases += [rnd.uniform(-1e8, 1e8) for _ in range(2000)]
    for ppb in cases:
        out = ctypes.c_int64()
        rc = lib.ref_ppb_to_scaled_ppm(ppb, ctypes.byref(out))
        got = fwport.ppb_to_scaled_ppm(ppb)
        if rc < 0:
            assert got is None, ppb
        else:
            assert got == out.value, ppb


def _c_find(lib, inc, target, imax, cmax):
    ic = ctypes.c_int(-12345)
    c = ctypes.c_uint32(0xDEAD)
    rc = lib.ref_find_correction(inc, target, imax, cmax, ctypes.byref(ic), ctypes.byref(c))
    return rc, ic.value, c.value


@pytest.mark.parametrize("clock_hz", [196608000, 98304000, 24000000, 100000000, 25000000, 61003000,
                                      50000000, 125000000, 16000000])
def test_find_correction_matches_c(lib, clock_hz):
    inc = fwport.NSEC_PER_SEC // clock_hz
    nominal = 1e9 / clock_hz
    rnd = random.Random(clock_hz)
    ppms = [0.0, 1e-3, -1e-3, 0.5, -0.5, 200.0, -200.0, 5000.0, -50000.0, 50000.0]
    ppms += [rnd.uniform(-300, 300) for _ in range(150)]
    for ppm in ppms:
        target = nominal * (1.0 + ppm * 1e-6)
        rc_c, ic_c, c_c = _c_find(lib, inc, target, 127, 0x7FFFFFFF)
        rc_p, ic_p, c_p = fwport.find_correction(inc, target, 127, 0x7FFFFFFF)
        assert rc_p == rc_c
        if rc_c == 0:
            assert (ic_p, c_p) == (ic_c, c_c), (clock_hz, ppm)
        # scalar port agrees too
        rc_s, ic_s, c_s = fwport.find_correction(inc, target, 127, 0x7FFFFFFF, vectorised=False)
        assert (rc_s, ic_s, c_s) == (rc_p, ic_p, c_p)


def test_find_correction_edge_cases_match_c(lib):
    for inc, target in [(127, 128.0), (127, 126.0), (126, 126.4082), (126, 126.9), (10, 10 + 1e-12),
                        (10, 10.0), (1, 0.5), (1, 0.999999)]:
        rc_c, ic_c, c_c = _c_find(lib, inc, target, 127, 0x7FFFFFFF)
        rc_p, ic_p, c_p = fwport.find_correction(inc, target, 127, 0x7FFFFFFF)
        assert rc_p == rc_c
        if rc_c == 0:
            assert (ic_p, c_p) == (ic_c, c_c)


# ---- ports of the PR #121108 ztest cases (tests/drivers/ptp_clock/nxp_enet_rate_math) ----

COR_MAX = 0x7FFFFFFF


def test_pr_closer_neighbour_period():
    rc, ic, cor = fwport.find_correction(126, 126.0 + 1.0 / 2.45, 127, COR_MAX)
    assert rc == 0 and cor == 2


def test_pr_register_is_period_minus_one():
    for hz, expect in [(98304000, 64), (196608000, 32)]:
        inc = 1_000_000_000 // hz
        rc, ic, cor = fwport.find_correction(inc, 1e9 / hz, 127, COR_MAX)
        assert rc == 0 and ic == expect and cor == 312


def test_pr_shortest_period_is_two():
    rc, ic, cor = fwport.find_correction(126, 126.9, 127, COR_MAX)
    assert (rc, ic, cor) == (0, 127, 1)


def test_pr_whole_tick_not_corrected():
    for hz in (100000000, 25000000):
        inc = 1_000_000_000 // hz
        rc, ic, cor = fwport.find_correction(inc, 1e9 / hz, 127, COR_MAX)
        assert (rc, ic, cor) == (0, inc, 0)


def test_pr_tiny_fraction_not_corrected():
    rc, ic, cor = fwport.find_correction(10, 10 + 1e-12, 127, COR_MAX)
    assert (rc, ic, cor) == (0, 10, 0)


def test_pr_no_room_einval():
    assert fwport.find_correction(127, 128.0, 127, COR_MAX)[0] == -fwport.EINVAL
    assert fwport.find_correction(127, 126.0, 127, COR_MAX)[0] == 0


CLOCK_CASES = [("audio_pll_div2", 196608000, 5.0), ("audio_pll_div4", 98304000, 5.0),
               ("osc_24m", 24000000, 40.0), ("sys_pll1_div2_div5", 100000000, 0.1),
               ("whole_tick_25m", 25000000, 0.1), ("arbitrary_61m", 61003000, 6.0)]


def _avg(inc, ic, cor):
    return float(inc) if cor == 0 else inc + (ic - inc) / (cor + 1)


@pytest.mark.parametrize("name,hz,limit", CLOCK_CASES)
def test_pr_sweep_within_limit(name, hz, limit):
    inc = 1_000_000_000 // hz
    nominal = 1e9 / hz
    ppm = -200.0
    prev = -math.inf
    while ppm <= 200.0:
        target = nominal * (1.0 + ppm * 1e-6)
        rc, ic, cor = fwport.find_correction(inc, target, 127, COR_MAX)
        assert rc == 0
        resid = abs((_avg(inc, ic, cor) - target) / target * 1e6)
        assert resid <= limit, (name, ppm, resid)
        assert 0 <= ic <= 127 and 0 <= cor <= COR_MAX
        assert (cor == 0) == (ic == inc)
        ppm += 0.5


@pytest.mark.parametrize("name,hz,limit", CLOCK_CASES)
def test_pr_average_tick_monotonic(name, hz, limit):
    inc = 1_000_000_000 // hz
    nominal = 1e9 / hz
    prev = -math.inf
    for ppm in (-200, -100, 0, 100, 200):
        rc, ic, cor = fwport.find_correction(inc, nominal * (1 + ppm * 1e-6), 127, COR_MAX)
        avg = _avg(inc, ic, cor)
        assert avg >= prev
        prev = avg


def test_nxp_timer_model_nominal_ratio():
    t = fwport.NxpTimer(100_000_000)
    assert (t.inc, t.inc_corr, t.cor) == (10, 10, 0)
    assert t.effective_ratio == 1.0
    t24 = fwport.NxpTimer(24_000_000)
    # nominal tick 41.666.. ns, INC = 41: the fractional part must be made up at start
    assert abs(t24.effective_ratio - 1.0) < 40e-6
    assert t24.cor != 0


def test_nxp_timer_rejects_out_of_range_ratio():
    t = fwport.NxpTimer(100_000_000)
    assert t.rate_adjust(1.0 + 60000e-6) == -fwport.EINVAL
    assert t.rate_adjust(1.0 - 60000e-6) == -fwport.EINVAL
    assert t.rate_adjust(1.0 + 49999e-6) == 0


def test_nxp_timer_unusable_clock():
    with pytest.raises(ValueError):
        fwport.NxpTimer(0)
    with pytest.raises(ValueError):
        fwport.NxpTimer(5_000_000)  # inc = 200 > 127
