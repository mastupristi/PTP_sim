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
    step_ns: float = 0.0                 # slave: PHC read -> PHC set inside clock_step (residual error after a step)


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
class PllConfig:
    """Fractional-N Audio PLL of the i.MX RT1170 (``ptpsim/pll.py``): f_VCO = Fref * (DIV_SELECT + NUM / DENOM).
    DIV_SELECT and the nominal NUM are derived from ``vco_hz`` and ``den``."""
    fref_hz: int = 24_000_000            # crystal (common to every PLL of the chip)
    vco_hz: int = 786_432_000
    den: int = 1_000_000_000             # DENOM (30 bits); a power of ten keeps NUM per ppb a decimal constant
    timer_div: int = 4                   # VCO / timer_div = PTP timer clock in the shared mode (196.608 MHz)
    fsync_div: int = 16_384              # VCO / fsync_div = FSYNC (48 kHz)
    step_limit_hz: float = 200_000.0     # AN14420: max VCO change per NUM write without glitch (0 = no limit)
    step_policy: str = "substep"         # substep (several writes, one per lock time) | cap (one capped write) | off


@dataclass
class FsyncConfig:
    """TDM frame sync of the follower.

    mode ``off``: no FSYNC model (the simulator as before).  ``shared`` (architecture B): the PTP timer and FSYNC both
    come from the Audio PLL, and the PTP servo trims its NUM (the ENET timer correction stays at its nominal setting).
    ``cascade`` (architecture A): the PTP timer is disciplined as configured in ``actuator``; FSYNC comes from the
    Audio PLL and a second servo, fed by the PTP-timer timestamps of the FSYNC rising edges, trims its NUM.
    """
    mode: str = "off"
    initial_offset_ns: float | None = None   # FSYNC phase vs the GM grid at t = 0 (None = the PHC initial offset)
    decimation: int = 48                 # cascade: one servo-2 update every this many FSYNC edges
    phase_detector: str = "unwrapped"    # cascade: unwrapped (grid anchored on the first edge: FSYNC follows PTP steps
                                         # too) | wrapped (edge vs the PHC grid modulo the period: re-aligns within P/2)
    cmd_latency_ns: float = 0.0          # cascade: capture -> NUM write
    capture_noise_ns: float = 0.0        # cascade: noise of the edge timestamps (on top of the timer tick)
    # second servo: its command limit sits inside the NUM range (-9 304 / +23 464 ppm at 786.432 MHz), otherwise the
    # controller would wind up while NUM is clamped
    controller: ControllerConfig = field(default_factory=lambda: ControllerConfig(
        "pi_time_aware", {"wn": 20.0, "zeta": 1.0, "sat_ppb": 9_000_000.0}))
    pll: PllConfig = field(default_factory=PllConfig)


@dataclass
class FirmwareConfig:
    """Constants of clock.c (SYNC_SERVO_*). Not changed unless explicitly requested."""
    step_threshold_ns: int = 1_000_000_000
    lock_offset_ns: int = 10_000_000
    outlier_ns: int = 100_000_000
    lock_samples: int = 3
    outlier_samples: int = 2
    delay_req_clear_ns: int = 3_000_000_000   # PORT_DELAY_REQ_CLEAR_TO
    # Experimental, NOT in the firmware: saturate the servo command to +-cmd_clamp_ppm before it is
    # handed to the driver.  0 = off = firmware behaviour (an out-of-range command is rejected by
    # the driver and the servo is reset).  Keep it <= actuator.max_ratio_ppm or the driver still rejects.
    cmd_clamp_ppm: float = 0.0
    # Experimental, NOT in the firmware: correct the delay sample for the clock moving between t2 and t3
    # (see ``Simulation._ptp_clock_delay``).  False = firmware behaviour.
    delay_rate_comp: bool = False


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
    fsync: FsyncConfig = field(default_factory=FsyncConfig)

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
