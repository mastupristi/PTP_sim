# SPDX-License-Identifier: Apache-2.0
"""Interchangeable servo controllers.

Interface (``Controller``): the firmware model hands over a :class:`ServoSample` (only data the
firmware really has: estimated offset, the interval between the last two Sync pairs measured
with GM timestamps, ...) and receives the **absolute** frequency correction in ppb that must
be applied to the clock (positive = faster).  The controller never sees simulator ground truth.

``reset()`` is invoked by the firmware servo state machine (``clock_servo_reset``) - it is the
firmware, not the controller, that decides when a reset happens.

Gain-change policy (live mode): ``set_params(params, policy)`` with policy
``keep`` (integrator state untouched - the default), ``reset`` (integrator zeroed) or
``bumpless`` (integrator recomputed so that the *last output* is unchanged).  Nothing is reset
implicitly.  (For ``pi_per_second`` a carried-over integral above ``i_max_ppm`` is clamped at the
next update, not at the change.)
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from . import fwport

POLICIES = ("keep", "reset", "bumpless")


@dataclass
class ServoSample:
    offset_ns: int             # estimated offset (slave - GM), firmware convention
    sync_interval_s: float     # interval between the two last completed pairs (from t1 differences)
    nominal_interval_s: float  # logMessageInterval-based nominal Sync interval
    index: int                 # number of samples handed to the controller since the last reset


class Controller:
    name = "base"
    #: parameter name -> (default, minimum, maximum, description)
    PARAMS: dict[str, tuple[float, float, float, str]] = {}
    #: saturation of the command in ppb handled by the controller itself (None = none)
    saturation_ppb: float | None = None

    def __init__(self, **params: float):
        self.params = {k: v[0] for k, v in self.PARAMS.items()}
        for k, v in params.items():
            if k not in self.PARAMS:
                raise KeyError(f"{self.name}: unknown parameter {k!r}")
            self.params[k] = float(v)
        self.last_error = 0.0
        self.last_output = 0.0
        self.saturated_count = 0

    # -- to be provided by subclasses
    def update(self, s: ServoSample) -> float:  # pragma: no cover
        raise NotImplementedError

    def reset(self) -> None:  # pragma: no cover
        raise NotImplementedError

    @property
    def integral(self) -> float:  # pragma: no cover
        raise NotImplementedError

    def _set_integral(self, value: float) -> None:  # pragma: no cover
        raise NotImplementedError

    # -- common
    def set_params(self, params: dict[str, float], policy: str = "keep") -> None:
        if policy not in POLICIES:
            raise ValueError(f"policy must be one of {POLICIES}")
        for k, v in params.items():
            if k not in self.PARAMS:
                raise KeyError(f"{self.name}: unknown parameter {k!r}")
        changed = any(self.params.get(k) != float(v) for k, v in params.items())
        self.params.update({k: float(v) for k, v in params.items()})
        self._apply_params()
        if not changed:
            return
        if policy == "reset":
            self.reset()
        elif policy == "bumpless":
            self._set_integral(self.last_output - self._proportional(self.last_error))

    def _apply_params(self) -> None:
        pass

    def _proportional(self, error: float) -> float:  # pragma: no cover
        raise NotImplementedError


class BaselinePI(Controller):
    """Faithful port of the firmware PI (``precision_pi_update``).

    ``integral += ki * e;  ppb = kp * e + integral``  with ``e = -offset_ns``.
    kp, ki are dimensionless-per-sample gains (ppb per ns of offset, per update): there is
    no dt, so the continuous-time integral gain is ki / T_sync and the damping depends on the
    Sync interval (Kconfig: tuned for ~1 s).  No saturation, no anti-windup: out-of-range
    commands are handled by the firmware servo (reject -> reset).
    """
    name = "baseline_pi"
    PARAMS = {"kp": (0.7, 0.0, 10.0, "proportional gain [ppb/ns]"),
              "ki": (0.3, 0.0, 10.0, "integral gain [ppb/ns per update]")}

    def __init__(self, **params):
        super().__init__(**params)
        self._pi = fwport.PrecisionPI(self.params["kp"], self.params["ki"])

    def _apply_params(self):
        self._pi.kp = self.params["kp"]
        self._pi.ki = self.params["ki"]

    def update(self, s: ServoSample) -> float:
        e = -float(s.offset_ns)
        out = self._pi.update(e)
        self.last_error, self.last_output = e, out
        return out

    def reset(self) -> None:
        self._pi.reset()

    @property
    def integral(self) -> float:
        return self._pi.integral

    def _set_integral(self, value: float) -> None:
        self._pi.integral = value

    def _proportional(self, error: float) -> float:
        return self._pi.kp * error


class PITimeAware(Controller):
    """Experimental PI with explicit sampling time, stability-protected bandwidth and anti-windup.

    Parameterised by the closed-loop natural frequency ``wn`` [rad/s] and damping ``zeta`` of the
    continuous loop  phi' = u  (the offset is the integral of the frequency command u):

        kp = 2 * zeta * wn            [1/s = ppb/ns]
        ki = wn^2                     [1/s^2]
        u  = kp * e + I               [ppb],     I += ki * dt * e      (dt = measured interval)

    Differences from the baseline (``precision_pi``):

    * gains are per second, so the loop shape does not change with the Sync interval
      (the baseline's ki acts *per sample*: damping 0.64 at 1 s but 0.32 at 0.25 s);
    * the integral uses the interval measured from the GM timestamps (t1 differences);
    * the bandwidth is limited to ``wn * dt <= wn_ts_max``: the discretised loop is stable only for
      ``kp * dt < 2``, so a fixed wn would go unstable at long Sync intervals;
    * the command is clamped to +-``sat_ppb`` and the integrator is frozen when the output is
      saturated and the error would push it further (conditional integration): no windup,
      and the actuator limit (50000 ppm) is never reached, so there is no servo reset;
    * optionally (``i_max_ppm`` > 0) the integrator itself is limited to +-``i_max_ppm``: it must stay above the
      steady frequency correction (oscillator error + drift), otherwise the offset cannot reach zero.

    Same inputs (offset estimate only), same absolute-ppb output, same firmware servo state machine.
    """
    name = "pi_time_aware"
    PARAMS = {"wn": (1.0, 0.01, 20.0, "natural frequency [rad/s]"),
              "zeta": (1.0, 0.1, 5.0, "damping ratio []"),
              "sat_ppb": (400_000.0, 1.0, 5e7, "command limit [ppb] (must stay below the actuator limit)"),
              "i_max_ppm": (0.0, 0.0, 50_000.0, "integrator limit [ppm] (0 = off)"),
              "wn_ts_max": (0.35, 0.01, 1.0, "max wn * dt [rad] (stability guard, kp*dt < 2)"),
              "dt_max_s": (10.0, 0.01, 100.0, "measured dt is clamped to this value [s] (>= nominal Sync interval)")}

    def __init__(self, **params):
        super().__init__(**params)
        self._i = 0.0
        self._wn = self._wn_idle()
        self.saturation_ppb = self.params["sat_ppb"]

    def _apply_params(self):
        self.saturation_ppb = self.params["sat_ppb"]
        self._wn = self._wn_idle()

    def _wn_idle(self) -> float:
        """Bandwidth before the first update (and after a parameter change)."""
        return self.params["wn"]

    def _wn_request(self, error: float, dt: float) -> float:
        """Bandwidth asked for by this update, before the stability guard (a constant for ``pi_time_aware``)."""
        return self.params["wn"]

    @property
    def kp(self) -> float:
        return 2.0 * self.params["zeta"] * self._wn

    @property
    def ki(self) -> float:
        return self._wn ** 2

    def update(self, s: ServoSample) -> float:
        e = -float(s.offset_ns)
        dt = s.sync_interval_s if s.sync_interval_s > 0 else s.nominal_interval_s
        dt = min(dt, self.params["dt_max_s"])
        self._wn = min(self._wn_request(e, dt), self.params["wn_ts_max"] / dt)
        sat = self.params["sat_ppb"]
        i_new = self._i + self.ki * dt * e
        limit_ppb = self.params["i_max_ppm"] * 1000.0
        if limit_ppb > 0.0 and abs(i_new) > limit_ppb:
            i_new = math.copysign(limit_ppb, i_new)
        out = self.kp * e + i_new
        if out > sat or out < -sat:
            self.saturated_count += 1
            clamped = max(-sat, min(sat, out))
            # conditional integration: keep the new integrator value only if it moves the output inward
            if (out > sat and e > 0) or (out < -sat and e < 0):
                i_new = self._i
            out = clamped
        self._i = i_new
        self.last_error, self.last_output = e, out
        return out

    def reset(self) -> None:
        self._i = 0.0

    @property
    def integral(self) -> float:
        return self._i

    def _set_integral(self, value: float) -> None:
        self._i = value

    def _proportional(self, error: float) -> float:
        return self.kp * error


class PIGainScheduled(PITimeAware):
    """Experimental PI whose bandwidth grows with the size of the offset: fast while the error is large, slow (and so
    quiet) once it is small.

    Same law as ``pi_time_aware`` (``kp = 2 zeta wn``, ``ki = wn^2``, ``I += ki dt e``, clamp and conditional
    integration, ``wn dt <= wn_ts_max``) but ``wn`` is not a constant: it follows the scheduling variable ``s`` [ns],

        x = clip(ln(s / e_lo) / ln(e_hi / e_lo), 0, 1);     wn = wn_min * (wn_max / wn_min) ** x

    i.e. ``wn_min`` for ``s <= e_lo``, ``wn_max`` for ``s >= e_hi`` and a geometric interpolation between.

    ``s`` is **not** the raw ``|e|``: it is the median of the last three ``|e|`` (one noisy sample cannot speed the
    loop up) with a slow release -- it follows the median upwards at once and downwards with the time constant
    ``tau_release_s`` -- so the gain does not chatter around a steady state set by the jitter.

    ``I`` is kept in ppb (the output of the integrator), so changing ``ki`` does not move the command: the schedule
    is bumpless by construction.  Stability is not guaranteed for any schedule (gain scheduling is sound when the
    scheduling variable varies slowly against the loop, which is what the filter is for); ``wn_max`` is further capped by
    ``wn_ts_max / dt`` as in ``pi_time_aware``.  The slow bandwidth at small error also means a slower tracking of an
    oscillator drift: for a ramp in the frequency the offset settles at (ramp rate) / wn_min^2.
    """
    name = "pi_gain_scheduled"
    PARAMS = {"wn_min": (0.2, 0.01, 20.0, "natural frequency for a small error [rad/s]"),
              "wn_max": (1.2, 0.01, 20.0, "natural frequency for a large error [rad/s]"),
              "e_lo_ns": (1_000.0, 1.0, 1e9, "error at/below which wn = wn_min [ns]"),
              "e_hi_ns": (100_000.0, 1.0, 1e12, "error at/above which wn = wn_max [ns]"),
              "tau_release_s": (4.0, 0.0, 600.0, "time constant of the slow release of the scheduling variable [s]"),
              "zeta": (1.0, 0.1, 5.0, "damping ratio []"),
              "sat_ppb": (400_000.0, 1.0, 5e7, "command limit [ppb] (must stay below the actuator limit)"),
              "i_max_ppm": (0.0, 0.0, 50_000.0, "integrator limit [ppm] (0 = off)"),
              "wn_ts_max": (0.35, 0.01, 1.0, "max wn * dt [rad] (stability guard, kp*dt < 2)"),
              "dt_max_s": (10.0, 0.01, 100.0, "measured dt is clamped to this value [s] (>= nominal Sync interval)")}

    def __init__(self, **params):
        self._hist: list[float] = []           # last |e| (ns), newest last
        self._sched = 0.0                      # scheduling variable s [ns]
        super().__init__(**params)

    def _wn_idle(self) -> float:
        return self.params["wn_min"]

    def wn_for(self, s_ns: float) -> float:
        """Bandwidth the schedule gives for a scheduling variable ``s_ns`` (before the ``wn_ts_max`` guard)."""
        lo, hi = self.params["e_lo_ns"], self.params["e_hi_ns"]
        wmin, wmax = self.params["wn_min"], self.params["wn_max"]
        if s_ns <= lo:
            x = 0.0
        elif hi <= lo or s_ns >= hi:
            x = 1.0
        else:
            x = math.log(s_ns / lo) / math.log(hi / lo)
        return wmin * (wmax / wmin) ** x

    @property
    def scheduling_variable(self) -> float:
        """``s`` [ns] used by the last update."""
        return self._sched

    def _wn_request(self, error: float, dt: float) -> float:
        self._hist = (self._hist + [abs(error)])[-3:]
        med = sorted(self._hist)[len(self._hist) // 2]
        if med >= self._sched or self.params["tau_release_s"] <= 0.0 or len(self._hist) == 1:
            self._sched = med                                        # up (or the first sample): at once
        else:
            self._sched += (med - self._sched) * (1.0 - math.exp(-dt / self.params["tau_release_s"]))
        return self.wn_for(self._sched)

    def reset(self) -> None:
        super().reset()
        self._hist = []
        self._sched = 0.0


class PIPerSecond(BaselinePI):
    """Firmware PI law with the integral gain scaled by the measured Sync interval and an integrator limit.

    ``ki_eff = ki * dt / t_ref_s``;  ``integral += ki_eff * e;  integral = clamp(integral, +-i_max);
    ppb = kp * e + integral``  (``e = -offset_ns``, absolute output in ppb, same sign and clamp as the firmware).
    The integrator limit is the anti-windup: with the firmware command clamp (``firmware.cmd_clamp_ppm``) the output
    saturates while the baseline integrator keeps accumulating the error (windup), so the release overshoots.  The
    limit must stay above the steady frequency correction the loop needs (oscillator error + drift): below it the
    offset cannot reach zero.  ``i_max_ppm = 0`` disables the limit.

    Why: the firmware ``ki`` acts per update, so the continuous-time integral gain is ``ki / T_sync`` and the
    damping changes with the Sync interval (0.64 at 1 s, 0.32 at 0.25 s for 0.7 / 0.3).  Here ``kp`` and
    ``ki`` are the gains *tuned at* ``t_ref_s``: ``ki / t_ref_s`` [s^-2] is held constant, so the integrator
    adds ``ki * dt / t_ref_s`` per update and the loop shape no longer depends on the interval.
    ``kp`` is **not** scaled: ppb/ns is already 1/s, a continuous gain.  Default ``t_ref_s = 1`` (the interval the
    firmware gains are tuned for, Kconfig help); at ``dt == t_ref_s`` (and ``kp <= kp_dt_max``, ``i_max_ppm = 0``) the law is the firmware one (``baseline_pi``).
    Not bit-identical in a run: the measured ``dt`` carries the Sync jitter, and a lost Sync doubles it.

    ``dt`` is the interval measured from the GM timestamps (t1 differences), so a lost Sync gives a longer
    step; it is clamped to the absolute ``dt_max_s`` (the first sample after a long gap can be arbitrarily old).
    ``dt_max_s`` must be >= the nominal Sync interval, otherwise regular steps are clamped too.

    Guard on ``kp * dt``: the proportional step per update is ``kp * dt``, and the discrete loop cannot be
    stable beyond ``kp * dt = 2`` (no-lag model).  The gain actually applied is ``kp_eff = min(kp, kp_dt_max / dt)``
    (``kp_dt_max = 0``: guard off).  The default 1.0 leaves ``kp * dt <= 1`` untouched (so at ``dt == t_ref_s`` it
    is the firmware law for ``kp <= 1``) and caps a larger ``kp`` at long intervals.  It is a
    necessary condition only: the integral gain and the delay-estimate feedback also limit stability (see
    docs/model.md), so it does not make every ``kp, ki, dt`` combination stable.
    """
    name = "pi_per_second"
    PARAMS = {**BaselinePI.PARAMS,
              "i_max_ppm": (0.0, 0.0, 50_000.0, "integrator limit [ppm] (0 = off)"),
              "t_ref_s": (1.0, 0.01, 10.0, "interval at which kp, ki are tuned [s]; ki acts as ki*dt/t_ref_s"),
              "dt_max_s": (10.0, 0.01, 100.0, "measured dt is clamped to this value [s] (>= nominal Sync interval)"),
              "kp_dt_max": (1.0, 0.0, 10.0, "guard: kp_eff = min(kp, kp_dt_max / dt); 0 = off")}

    @property
    def ki_eff(self) -> float:
        """Per-update integral gain applied at the last update."""
        return self._ki_eff

    @property
    def kp_eff(self) -> float:
        """Proportional gain applied at the last update (``kp`` unless the ``kp * dt`` guard cut it)."""
        return self._kp_eff

    def __init__(self, **params):
        super().__init__(**params)
        self._ki_eff = self.params["ki"]
        self._kp_eff = self.params["kp"]
        self._dt = self.params["t_ref_s"]            # interval of the last update

    def _kp_for(self, dt: float) -> float:
        kp_dt_max = self.params["kp_dt_max"]
        return min(self.params["kp"], kp_dt_max / dt) if kp_dt_max > 0.0 else self.params["kp"]

    def _proportional(self, error: float) -> float:
        # bumpless transfer after a gain change: P with the *new* gains at the last interval
        return self._kp_for(self._dt) * error

    def update(self, s: ServoSample) -> float:
        e = -float(s.offset_ns)
        dt = s.sync_interval_s if s.sync_interval_s > 0 else s.nominal_interval_s
        dt = min(dt, self.params["dt_max_s"])
        pi = self._pi
        self._ki_eff = pi.ki * dt / self.params["t_ref_s"]
        self._dt, self._kp_eff = dt, self._kp_for(dt)
        pi.integral += self._ki_eff * e
        limit_ppb = self.params["i_max_ppm"] * 1000.0
        if limit_ppb > 0.0 and abs(pi.integral) > limit_ppb:
            pi.integral = math.copysign(limit_ppb, pi.integral)
        out = self._kp_eff * e + pi.integral
        self.last_error, self.last_output = e, out
        return out


REGISTRY: dict[str, type[Controller]] = {c.name: c for c in (BaselinePI, PITimeAware, PIGainScheduled, PIPerSecond)}


def make_controller(name: str, params: dict[str, float] | None = None) -> Controller:
    try:
        cls = REGISTRY[name]
    except KeyError:
        raise KeyError(f"unknown controller {name!r}; available: {sorted(REGISTRY)}") from None
    return cls(**(params or {}))
