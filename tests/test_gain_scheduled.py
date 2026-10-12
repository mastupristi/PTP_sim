# SPDX-License-Identifier: Apache-2.0
"""``pi_gain_scheduled``: the bandwidth follows the (filtered) size of the offset."""
import math
from pathlib import Path

import numpy as np
import pytest

from ptpsim.config import SimConfig
from ptpsim.controllers import PIGainScheduled, PITimeAware, ServoSample
from ptpsim.engine import simulate
from ptpsim.metrics import MetricsConfig, compute_metrics

CONFIGS = Path(__file__).parent.parent / "configs"


def sample(offset_ns, dt=0.25, index=0):
    return ServoSample(offset_ns=offset_ns, sync_interval_s=dt, nominal_interval_s=dt, index=index)


def test_schedule_is_geometric_between_the_two_corners():
    c = PIGainScheduled(wn_min=0.2, wn_max=1.2, e_lo_ns=1000.0, e_hi_ns=100_000.0)
    assert c.wn_for(0.0) == 0.2 and c.wn_for(1000.0) == 0.2
    assert c.wn_for(100_000.0) == pytest.approx(1.2) and c.wn_for(1e9) == pytest.approx(1.2)
    assert c.wn_for(10_000.0) == pytest.approx(math.sqrt(0.2 * 1.2))              # log-midpoint -> geometric mean
    s = np.logspace(2, 6, 50)
    assert (np.diff([c.wn_for(x) for x in s]) >= 0).all()
    # kp = 2 zeta wn, ki = wn^2 follow the applied wn
    c.update(sample(-100_000))
    assert c.kp == pytest.approx(2 * 1.2) and c.ki == pytest.approx(1.2 ** 2)


def test_constant_bandwidth_is_pi_time_aware():
    a = PIGainScheduled(wn_min=1.0, wn_max=1.0)
    b = PITimeAware(wn=1.0)
    rng = np.random.default_rng(3)
    for i in range(200):
        s = sample(int(rng.normal(0, 50_000)), index=i)
        assert a.update(s) == b.update(s)


def test_one_noisy_sample_does_not_speed_the_loop_up():
    c = PIGainScheduled(wn_min=0.2, wn_max=1.2)
    for off in (100, -100, 100):
        c.update(sample(off))
    assert c.scheduling_variable == 100.0
    c.update(sample(2_000_000))                                    # a single outlier: median of (100, 100, 2e6) = 100
    assert c.scheduling_variable == 100.0 and c.kp == pytest.approx(2 * 0.2)
    c.update(sample(2_000_000))                                    # two in a row: the median follows at once
    assert c.scheduling_variable == 2_000_000.0 and c.kp == pytest.approx(2 * 1.2)


def test_release_is_slow_attack_is_immediate():
    tau, dt = 4.0, 0.25
    c = PIGainScheduled(wn_min=0.2, wn_max=1.2, tau_release_s=tau)
    for i in range(3):
        c.update(sample(200_000, index=i))
    assert c.scheduling_variable == 200_000.0                      # attack: at once
    c.update(sample(500, index=3))                                 # one small sample: the median is still 200 000
    assert c.scheduling_variable == 200_000.0
    c.update(sample(500, index=4))                                 # two: the median is 500, s decays with tau, no jump
    decay = 1.0 - math.exp(-dt / tau)
    assert c.scheduling_variable == pytest.approx(200_000.0 + (500.0 - 200_000.0) * decay)
    prev = c.scheduling_variable
    for i in range(40):
        c.update(sample(500, index=6 + i))
        assert c.scheduling_variable < prev
        prev = c.scheduling_variable
    assert prev == pytest.approx(500.0 + 199_500.0 * math.exp(-41 * dt / tau), rel=1e-6)      # exponential release
    no_filter = PIGainScheduled(tau_release_s=0.0)
    for off in (200_000, 200_000, 200_000, 500, 500):
        no_filter.update(sample(off))
    assert no_filter.scheduling_variable == 500.0


def test_stability_guard_still_caps_the_bandwidth():
    c = PIGainScheduled(wn_min=0.2, wn_max=5.0, wn_ts_max=0.35)
    c.update(sample(1_000_000, dt=1.0))
    assert c.ki == pytest.approx(0.35 ** 2) and c.kp == pytest.approx(2 * 0.35)


def test_reset_forgets_the_history():
    c = PIGainScheduled()
    for i in range(3):
        c.update(sample(500_000, index=i))
    c.reset()
    assert c.scheduling_variable == 0.0 and c.integral == 0.0
    c.update(sample(100))
    assert c.scheduling_variable == 100.0


def test_closed_loop_quieter_than_pi_time_aware_on_the_noisy_scenario():
    base = SimConfig.load(CONFIGS / "noisy_seed1.json")
    mc = MetricsConfig(band_ns=2000.0, dwell_s=10.0, final_window_s=60.0)

    def run(name, params):
        r = compute_metrics(simulate(base.with_overrides(**{"controller.name": name, "controller.params": params})), mc)
        return r["true_offset"]

    ta, gs = run("pi_time_aware", {}), run("pi_gain_scheduled", {})
    assert gs["settling_s"] is not None and ta["settling_s"] is not None
    assert gs["rms_final_ns"] < 0.8 * ta["rms_final_ns"]           # slow bandwidth near zero filters the jitter
    assert gs["settling_s"] <= 1.3 * ta["settling_s"]              # without a (much) slower transient


@pytest.mark.parametrize("cls,kw", [(PITimeAware, {"wn": 1.0}), (PIGainScheduled, {})])
def test_integrator_limit_bounds_the_integral_and_is_off_by_default(cls, kw):
    free, limited = cls(**kw), cls(i_max_ppm=5.0, **kw)
    peak_free = 0.0
    for i in range(400):
        s = sample(-50_000 if i < 200 else 50_000, index=i)                       # long error of one sign, then the other
        free.update(s)
        limited.update(s)
        peak_free = max(peak_free, abs(free.integral))
        assert abs(limited.integral) <= 5_000.0 + 1e-9
    assert peak_free > 5_000.0                                                    # ... which the free integrator exceeds
    assert cls(i_max_ppm=0.0, **kw).update(sample(-1000)) == cls(**kw).update(sample(-1000))
