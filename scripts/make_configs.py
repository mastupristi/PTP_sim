# SPDX-License-Identifier: Apache-2.0
"""Regenerate the example scenario files in configs/ (python scripts/make_configs.py)."""
from pathlib import Path

from ptpsim.config import SimConfig, noisy_preset

OUT = Path(__file__).resolve().parent.parent / "configs"


def base() -> SimConfig:
    c = SimConfig()
    c.oscillator.initial_offset_ns = 100_000.0
    c.oscillator.freq_error_ppb = 20_000.0
    return c


def main():
    OUT.mkdir(exist_ok=True)
    scen = {
        "default_deterministic": base(),
        "noisy_seed1": noisy_preset(),
        "variant_pi_time_aware": base().with_overrides(**{
            "controller.name": "pi_time_aware",
            "controller.params": {"wn": 1.0, "zeta": 1.0, "sat_ppb": 400000.0, "wn_ts_max": 0.35}}),
        "sync_1s_delay_8s": base().with_overrides(**{"intervals.sync_log": 0, "intervals.delay_log": 3,
                                                      "duration_s": 600.0}),
        "every_8_sync_mode": base().with_overrides(**{"intervals.delay_mode": "every_n_sync",
                                                       "intervals.delay_every_n": 8}),
        "nxp_100mhz": base().with_overrides(**{"actuator.kind": "nxp", "actuator.clock_hz": 100_000_000}),
        "nxp_98p304mhz": base().with_overrides(**{"actuator.kind": "nxp", "actuator.clock_hz": 98_304_000}),
        "nxp_24mhz": base().with_overrides(**{"actuator.kind": "nxp", "actuator.clock_hz": 24_000_000}),
        "asymmetric_network": base().with_overrides(**{"network.delay_ms_ns": 1500.0, "network.delay_sm_ns": 500.0}),
        "loss_5pct_noisy": noisy_preset().with_overrides(**{"loss.sync": 0.05, "loss.follow_up": 0.05,
                                                             "loss.delay_req": 0.05, "loss.delay_resp": 0.05}),
        "large_offset_50ms": base().with_overrides(**{"oscillator.initial_offset_ns": 50e6, "duration_s": 300.0}),
    }
    for name, cfg in scen.items():
        cfg.save(OUT / f"{name}.json")
        print("wrote", OUT / f"{name}.json")


if __name__ == "__main__":
    main()
