# SPDX-License-Identifier: Apache-2.0
"""Simulation configuration (JSON-serialisable dataclasses).

Units: times are in the unit named by the field suffix (``_ns``, ``_s``); frequency errors in
ppb (1 ppb = 1 ns/s); the physical time of the simulator is the GM time.
"""
from __future__ import annotations

import copy
import dataclasses
import json
from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any


@dataclass
class JitterSpec:
    """Zero-mean (except ``exponential``) random time offset, drawn once per message.

    kind: ``none`` | ``uniform`` (half-width ``scale_ns``) | ``normal`` (sigma ``scale_ns``,
    truncated at ``clip_sigma`` sigmas) | ``exponential`` (mean ``scale_ns``, one-sided).
    """
    kind: str = "none"
    scale_ns: float = 0.0
    clip_sigma: float = 4.0


@dataclass
class TxJitterConfig:
    """Send-instant jitter around the nominal instant (non-cumulative), per message type.

    With hardware timestamps this does NOT corrupt t1..t4: it only makes the sampling
    irregular (and delays/anticipates Follow_Up / Delay_Resp / Delay_Req). See README.
    """
    sync: JitterSpec = field(default_factory=JitterSpec)
    follow_up: JitterSpec = field(default_factory=JitterSpec)
    delay_req: JitterSpec = field(default_factory=JitterSpec)
    delay_resp: JitterSpec = field(default_factory=JitterSpec)


@dataclass
class NetworkConfig:
    delay_ms_ns: float = 1000.0      # GM -> slave one-way delay (Sync, Follow_Up, Delay_Resp)
    delay_sm_ns: float = 1000.0      # slave -> GM one-way delay (Delay_Req)
    jitter_ms: JitterSpec = field(default_factory=JitterSpec)   # path delay variation, GM->slave
    jitter_sm: JitterSpec = field(default_factory=JitterSpec)   # path delay variation, slave->GM
    correction_sync_ns: float = 0.0       # correctionField of Sync (ns, 2^-16 fraction kept)
    correction_follow_up_ns: float = 0.0
    correction_delay_resp_ns: float = 0.0
    delay_asymmetry_ns: float = 0.0       # port_ds.delay_asymmetry configured in the slave


@dataclass
class LatencyConfig:
    """Software latencies (ns). Constant part; random part is in TxJitterConfig."""
    follow_up_ns: float = 100_000.0      # GM: Sync TX timestamp -> Follow_Up emission
    delay_resp_ns: float = 100_000.0     # GM: Delay_Req RX -> Delay_Resp emission
    rx_processing_ns: float = 50_000.0   # slave: frame arrival -> message processed by the PTP thread
    command_ns: float = 0.0              # slave: servo processing -> new rate effective in hardware
    tx_timestamp_cb_ns: float = 0.0      # slave: Delay_Req TX -> TX timestamp callback


@dataclass
class TimestampConfig:
    gm_quantum_ns: float = 0.0           # 0 = none
    gm_noise_sigma_ns: float = 0.0
    slave_quantum_ns: float | None = None   # None = auto (0 for ideal actuator, tick for NXP)
    slave_noise_sigma_ns: float = 0.0


@dataclass
class OscillatorConfig:
    initial_offset_ns: float = 0.0       # slave - GM at t = 0
    freq_error_ppb: float = 0.0          # slave oscillator error at t = 0
    drift_ppb_per_s: float = 0.0         # linear frequency drift (ppb/s), applied in steps
    walk_ppb_per_sqrt_s: float = 0.0     # random-walk of the frequency (ppb / sqrt(s))
    update_period_s: float = 1.0         # step used to update drift / walk (events, not a grid)
    timer_error_ppb: float = 0.0         # k_timer rate error vs physical time (independent of the PHC)


@dataclass
class LossConfig:
    sync: float = 0.0
    follow_up: float = 0.0
    delay_req: float = 0.0
    delay_resp: float = 0.0


@dataclass
class IntervalConfig:
    sync_log: int = -2                   # Sync interval = 2^n s
    delay_mode: str = "interval"         # "interval" (firmware) | "every_n_sync" (NOT in firmware)
    delay_log: int = 1                   # Delay_Req interval = 2^n s (advertised in Delay_Resp)
    delay_every_n: int = 8
    delay_rearm_from_handling: bool = False   # firmware re-arms its k_timer at handling time (drift)
    # (False = jitter around a fixed nominal grid, as observed in the traces, see README)


@dataclass
class ActuatorConfig:
    kind: str = "ideal"                  # "ideal" | "nxp"
    clock_hz: int = 100_000_000          # ENET 1588 timer clock root (NXP)
    max_ratio_ppm: int = 50000           # CONFIG_PTP_CLOCK_NXP_ENET_MAX_RATIO_PPM (also for ideal)


@dataclass
class ControllerConfig:
    name: str = "baseline_pi"
    params: dict[str, float] = field(default_factory=lambda: {"kp": 0.7, "ki": 0.3})


@dataclass
class FirmwareConfig:
    """Constants of clock.c (SYNC_SERVO_*). Not changed unless explicitly requested."""
    step_threshold_ns: int = 1_000_000_000
    lock_offset_ns: int = 10_000_000
    outlier_ns: int = 100_000_000
    lock_samples: int = 3
    outlier_samples: int = 2
    delay_req_clear_ns: int = 3_000_000_000   # PORT_DELAY_REQ_CLEAR_TO


@dataclass
class SimConfig:
    duration_s: float = 300.0
    seed: int = 1
    epoch_ns: int = 1_700_000_000_000_000_000   # absolute time base of all timestamps
    intervals: IntervalConfig = field(default_factory=IntervalConfig)
    oscillator: OscillatorConfig = field(default_factory=OscillatorConfig)
    network: NetworkConfig = field(default_factory=NetworkConfig)
    latency: LatencyConfig = field(default_factory=LatencyConfig)
    tx_jitter: TxJitterConfig = field(default_factory=TxJitterConfig)
    timestamps: TimestampConfig = field(default_factory=TimestampConfig)
    loss: LossConfig = field(default_factory=LossConfig)
    actuator: ActuatorConfig = field(default_factory=ActuatorConfig)
    controller: ControllerConfig = field(default_factory=ControllerConfig)
    firmware: FirmwareConfig = field(default_factory=FirmwareConfig)

    # ---- (de)serialisation -------------------------------------------------
    def to_dict(self) -> dict[str, Any]:
        return dataclasses.asdict(self)

    @classmethod
    def from_dict(cls, d: dict[str, Any]) -> "SimConfig":
        return _build(cls, d)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, sort_keys=True)

    def save(self, path: str | Path) -> None:
        Path(path).write_text(self.to_json() + "\n")

    @classmethod
    def load(cls, path: str | Path) -> "SimConfig":
        return cls.from_dict(json.loads(Path(path).read_text()))

    def copy(self) -> "SimConfig":
        return copy.deepcopy(self)

    def with_overrides(self, **dotted: Any) -> "SimConfig":
        """``cfg.with_overrides(**{"intervals.sync_log": -3, "duration_s": 60})``."""
        c = self.copy()
        for key, value in dotted.items():
            set_path(c, key, value)
        return c


def _build(cls, d):
    kwargs = {}
    names = {f.name: f for f in fields(cls)}
    for k, v in d.items():
        if k not in names:
            raise KeyError(f"unknown config key {k!r} for {cls.__name__}")
        default = getattr(cls(), k)
        if is_dataclass(default) and isinstance(v, dict):
            kwargs[k] = _build(type(default), v)
        else:
            kwargs[k] = v
    return cls(**kwargs)


def set_path(cfg: Any, dotted: str, value: Any) -> None:
    obj = cfg
    parts = dotted.split(".")
    for p in parts[:-1]:
        obj = getattr(obj, p)
    if isinstance(obj, dict):
        obj[parts[-1]] = value
        return
    if not hasattr(obj, parts[-1]):
        raise KeyError(dotted)
    setattr(obj, parts[-1], value)


def get_path(cfg: Any, dotted: str) -> Any:
    obj = cfg
    for p in dotted.split("."):
        obj = obj[p] if isinstance(obj, dict) else getattr(obj, p)
    return obj


def default_scenario() -> SimConfig:
    """Deterministic scenario used by the GUI/CLI/benchmarks: 100 us initial offset, +20 ppm oscillator,
    Sync 0.25 s, Delay_Req 2 s, symmetric 1 us network, ideal actuator, baseline PI (0.7, 0.3)."""
    c = SimConfig()
    c.oscillator.initial_offset_ns = 100_000.0
    c.oscillator.freq_error_ppb = 20_000.0
    return c


def noisy_preset() -> SimConfig:
    """Scenario with the disturbances described in CLAUDE.md: tens-of-us send jitter on all
    messages, small path jitter and timestamp noise.  Deterministic for a given seed."""
    c = SimConfig()
    j = JitterSpec("normal", 20_000.0)
    c.tx_jitter = TxJitterConfig(sync=copy.deepcopy(j), follow_up=copy.deepcopy(j),
                                 delay_req=copy.deepcopy(j), delay_resp=copy.deepcopy(j))
    c.network.jitter_ms = JitterSpec("exponential", 200.0)
    c.network.jitter_sm = JitterSpec("exponential", 200.0)
    c.timestamps.gm_noise_sigma_ns = 3.0
    c.timestamps.slave_noise_sigma_ns = 3.0
    c.oscillator.freq_error_ppb = 20_000.0
    c.oscillator.initial_offset_ns = 100_000.0
    return c
