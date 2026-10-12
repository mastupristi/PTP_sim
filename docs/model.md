# Model, conventions and approximations

Companion of [firmware_reconstruction.md](firmware_reconstruction.md) (what the firmware does) — this file is
what the simulator does, why, and where it is approximate.

## 1. Time and numbers

* Physical simulation time = GM time, **integer picoseconds** (Python `int`, exact). No fixed-step loop, no sleeps.
* Timestamps (`t1..t4`) are integer nanoseconds relative to a configurable epoch (default 1.7e18 ns, like a TAI/PTP
  time). They never go through float64 (at that magnitude a double has 256 ns resolution). The only floats are the
  small, bounded true offset `phi` (slave − GM, ns) and the rate error; the slave clock keeps exact integer
  segment anchors. `tests/test_engine.py::test_time_precision_on_a_million_seconds` runs 1e6 s and checks that
  offsets/delays stay exact and that a 1 ppb error integrates to 1e6 ns within 1e-6 ns.
* Rate: `slope = (1+ε)(1+d) − 1` evaluated as `ε + d + εd` (no `1 + x` cancellation), `ε` oscillator error, `d` realised
  rate ratio minus one (from the actuator, exactly).
* Conventions: offset = slave − GM (firmware convention); positive = slave ahead; `t1,t4` GM domain, `t2,t3` slave domain.
  Rates in ppb (1 ppb = 1 ns/s). The actual rate of the slave clock = oscillator error × applied ratio.

## 2. Event model

| Event | What happens |
|---|---|
| `GM_SYNC_TX(k)` | nominal grid `n·2^sync_log` s + send jitter; `t1` = GM hardware timestamp; next Sync scheduled with the *current* interval; loss; Follow_Up scheduled |
| `GM_FUP_TX` | at `TX + follow_up latency + jitter`; carries `t1` |
| `SLAVE_SYNC_RX` | **`t2` read from the slave clock at the arrival instant** (noise, tick quantisation); processing scheduled after the RX-processing latency |
| `*_PROC` | firmware software: pairing slot, `port_synchronize`, servo, Delay_Resp handling |
| `DREQ_TIMER` / trigger | independent timer (monotonic, own rate error) or non-firmware "every N Sync"; `t3` read from the slave clock at the TX instant |
| `GM_DREQ_RX` | `t4` = GM timestamp at arrival; Delay_Resp scheduled |
| `RATE_EFFECT` | the new rate becomes effective `command latency` after the servo processing (the driver's accept/reject decision is synchronous, as in C) |
| `OSC_UPDATE` | oscillator drift / random walk step |

Equal times are ordered by priority (rate change, TX, RX, processing) then FIFO creation order: deterministic.
`Simulation.run_until(t)` can be called in chunks with identical results (used by the live mode: playback speed
does not change the maths).

## 3. Disturbance sources (jitter analysis)

| Source | Where | Corrupts `t1..t4`? | Effect |
|---|---|---|---|
| send-instant jitter (all 4 message types) | emission time vs nominal grid | **no** (HW timestamps) | irregular sampling period seen by the PI (which has no dt), processing time of Follow_Up/Delay_Resp, spacing of `t2`/`t3` |
| path-delay variation, per direction | network | **yes**, directly | noise on the offset estimate; asymmetric realisations bias the delay |
| fixed asymmetry | network / `correctionField` | yes | true offset converges to −asym/2 (tested) |
| software latencies | Follow_Up, Delay_Resp, RX processing, command, TX-timestamp callback | no | delays when the servo/delay update happens and when the command takes effect; TX-callback latency can make the firmware use its pre-filled software `t3` |
| timestamp noise / quantisation | GM and slave HW | yes | noise; slave quantum = timer tick for the NXP actuator |
| clock motion between `t2` and `t3` | the servo itself | indirectly | delay bias `(o(t2) − o(t3))/2` |

Send jitter is drawn once per message from its own stream (`normal` truncated at 4σ, `uniform`, `exponential`,
or `none`) around the nominal instant, **non-cumulative** (as observed: ± tens of µs on both sides). The firmware
itself re-arms its `k_timer` at handling time (cumulative drift); that variant exists as
`intervals.delay_rearm_from_handling`.

Reproducibility: stream = `SeedSequence(seed, crc32(name))`, value = element `i` of the stream where `i` is the
message index; the order of draws is irrelevant.

## 4. Delay_Req: three distinct things

1. **Independent interval** (firmware): first request uniform in (0, 2·2ⁿ], then every 2ⁿ s of the undisciplined timer.
2. **Every N Sync** (not in the firmware): triggered after the N-th processed Sync/Follow_Up pair. Changing the Sync
   interval keeps `N`, and in mode 1 changing the Sync interval does *not* change the Delay_Req period (no hidden 8:1 ratio).
3. The interval the slave uses is overwritten by the `logMessageInterval` of each Delay_Resp (`fw_delay_log`),
   the GM-side value is `intervals.delay_log`.

## 5. Actuators

* **Ideal:** realises exactly the requested ratio inside ±`max_ratio_ppm` (default 50 000 ppm, the NXP driver limit);
  outside, the request is rejected and the firmware resets the servo — so baseline comparisons are fair.
* **NXP:** register-level model of `ptp_clock_nxp_enet.c`: `INC = floor(1e9/f)`, the pair for ratio 1.0 programmed
  once, `ptp_clock_nxp_enet_find_correction` per request, average tick `INC + (INC_CORR−INC)/(ATCOR+1)`. The realised
  rate is the *average* tick over the exact nominal tick.

### What the average-rate model leaves out

The hardware adds `INC` every tick and `INC_CORR` once every `ATCOR+1` ticks, so the timer value is a staircase
that deviates from the straight line by up to `INC_CORR−INC` ns (pk-pk) with period `(ATCOR+1)` ticks. This
deterministic pattern is timestamp noise (it also shows in the PHC reading). It matters when `INC_CORR−INC` is
large (98.304/196.608 MHz: 54/27 ns) and is negligible at 100/25 MHz. It is not reproduced; the correction counter
phase at `ATCOR` writes is undocumented (the PR measured only the period). Emulate with
`timestamps.slave_noise_sigma_ns ≈ pk-pk/√12`.

Generated with `python scripts/nxp_table.py` (same search as PR #121108, ±200 ppm sweep in 0.25 ppm steps):

| clock root | INC | INC_CORR (ratio 1.0) | ATCOR | nominal-pair error [ppm] | counter step pattern pk-pk [ns] | largest gap between reachable rates in ±200 ppm [ppm] | worst request error in ±200 ppm [ppm] |
|---|---|---|---|---|---|---|---|
| 100 MHz (SYS_PLL1_DIV2/5, SoC default) | 10 | 10 | 0 | +0.000 | 0 | 0.27 | 0.02 |
| 98.304 MHz (AUDIO_PLL/4) | 10 | 64 | 312 | -0.204 | 54 | 6.47 | 3.15 |
| 196.608 MHz (AUDIO_PLL/2) | 5 | 32 | 312 | -0.204 | 27 | 6.15 | 3.04 |
| 24 MHz (OSC_24M) | 41 | 43 | 2 | +0.000 | 2 | 62.99 | 31.49 |
| 25 MHz | 40 | 40 | 0 | +0.000 | 0 | 0.26 | 0.02 |

At 24 MHz no average rate exists between 0 and ±63 ppm, so any requested correction of a few ppm is realised by
dithering between neighbours (visible in `docs/img/nxp_roots.png`). The PR's own test accepts up to 40 ppm of
residual at 24 MHz.

## 6. Controllers (`ptpsim/controllers.py`)

| | `baseline_pi` (firmware) | `pi_time_aware` (experimental) | `pi_per_second` (experimental) |
|---|---|---|---|
| law | `I += ki·e; u = kp·e + I` | `u = kp·e + I; I += ki·dt·e` (conditional integration) | `I += ki·dt/t_ref·e; I = clamp(I, ±i_max); u = kp·e + I` |
| `e` | −offset [ns] | −offset [ns] | −offset [ns] |
| parameters | `kp` [ppb/ns], `ki` [ppb/ns per update] | `wn` [rad/s], `zeta`; `kp = 2ζ·wn` [s⁻¹], `ki = wn²` [s⁻²] | as baseline (at `t_ref_s`) + `i_max_ppm` [ppm] (0 = off), `t_ref_s`, `dt_max_s`, `kp_dt_max` |
| dt | none | measured from consecutive `t1` (GM timestamps), clamped to `dt_max_s` | measured from consecutive `t1`, clamped to `dt_max_s`; scales `ki` only |
| stability | none (Kconfig: tuned for ~1 s) | `wn·dt ≤ wn_ts_max` (0.35 rad): `kp·dt < 2` | `kp_eff = min(kp, kp_dt_max/dt)` (`kp·dt ≤ 1` by default) |
| saturation | none: driver rejects → `clock_servo_reset` | clamp ±`sat_ppb` (default 45 000 000 ppb = 45 000 ppm, 90 % of the actuator limit; the comparison below uses 400 000 ppb explicitly) + anti-windup | integrator only (output limited by the servo's command clamp, if on) |
| output | absolute ppb | absolute ppb | absolute ppb |

`pi_per_second` has `ki_eff = ki·dt/t_ref_s` (`dt` measured from consecutive `t1`,
clamped to the absolute `dt_max_s` (10 s); `t_ref_s = 1 s` by default, the interval the firmware gains are tuned for). `kp` is
not scaled: ppb/ns is already s⁻¹. `ki/t_ref_s` [s⁻²] is thus constant and the continuous damping
`ζ = kp / (2·sqrt(ki/t_ref_s))` = 0.64 for any Sync interval, whereas in the baseline it is `0.7 / (2·sqrt(0.3/T))`:
0.64 at 1 s, 0.32 at 250 ms, 0.16 at 62.5 ms. At `dt = t_ref_s` (and `kp ≤ kp_dt_max`, `i_max_ppm = 0`) it is the firmware law, bit-identical to `baseline_pi` when the measured `dt` equals `t_ref_s` exactly (no Sync jitter, no lost Sync; with jitter `dt` varies and so does `ki_eff`). Measured
(quiet scenario, 100 µs step, 0 ppm, 0.7 / 0.3, minimum of the true offset after the step; `tests/test_servo_options.py`
repeats the check): baseline −64.6 µs (62.5 ms), −55.6 (125 ms), −45.9 (250 ms), −35.0 (500 ms), −30.0 (1 s);
`pi_per_second` −22.9, −22.3, −22.9, −21.1, −30.0 µs. 

Guard on `kp·dt`: the applied gain is `kp_eff = min(kp, kp_dt_max/dt)` (`kp_dt_max` = 1 by default, 0 = off), because
the discrete loop cannot be stable for `kp·dt ≥ 2` and a fixed `kp` meets that at long Sync intervals. For `kp·dt ≤ 1`
it changes nothing. Measured on `configs/noisy_seed1.json` (5 seeds, 100 µs step, +20 ppm, `pi_per_second`, Sync 1 s,
band ±2 µs; seeds settled / median settling / median final RMS):

| kp / ki | guard off | guard on (`kp_dt_max` = 1) |
|---|---|---|
| 0.7 / 0.3 | 5/5, 13.1 s, 221 ns | same (kp·dt = 0.7: untouched) |
| 1.6 / 0.3 | 3/5, 273 s, 647 ns | 5/5, 12.1 s, 290 ns |
| 1.6 / 0.6 | 1/5, 283 s, 1604 ns | 5/5, 11.3 s, 395 ns |
| 2.5 / 0.3 | 0/5 (diverges) | 5/5, 12.1 s, 290 ns |
| 1.6 / 1.0 | 0/5 (diverges) | 3/5, 197 s, 595 ns |

So the guard removes the divergence caused by a large `kp`, but it is only necessary: with a large `ki` the loop still
rings at 1 s (last row). Sync = 2 s is a different problem and the guard does not cure it: a sweep of `kp·dt` = 0.5-1.6
at `ki` = 0.3 (run **before** the guard existed, i.e. effectively with it off, `i_max_ppm` = 0) on a quiet 49 ms / +20 ppm
scenario (`pi_per_second`, command clamp 50 000 ppm, step threshold 50 ms) settles only for `kp·dt` ≤ 0.7 (overshoot
70-90 %, 37-114 s at ±1 µs); on the noisy seeds 4/5 settle at `kp·dt` = 0.5 and none from 0.9. The cause is the
delay-estimate coupling described in the README (known limitation 3; the baseline test
`test_baseline_instability_at_long_sync_comes_from_delay_estimate_coupling`): in a 2 s noisy run the offset estimate
alternates in sign at every update and the delay estimate swings by ±70 µs (true delay: 1 µs). Reaching it takes a different
change, not a limit on `kp·dt`: the experimental `firmware.delay_rate_comp` below.

**Delay compensated for the clock rate** (`firmware.delay_rate_comp`, **not in the firmware**, off by default). The
firmware computes `delay = ((t2−t3)+(t4−t1))/2` with the latest `t1, t2` and a later `t3`. With offset `x(t)` (slave − GM)
this is `d + (x(t2) − x(t3))/2 = d − rate·(t3 − t2)/2` on a constant network: at +20 ppm and `t3 − t2` up to 2 s the
sample is low by up to 20 µs (tested open loop). Since the offset is `t2 − t1 − delay`, that error feeds the servo and,
at Sync 2 s, the loop diverges. The option adds `rate·(t3 − t2)/2` back, using only quantities the firmware has: `rate`
is the slope of `t2 − t1` between the last two Syncs (the delay cancels) plus the change of the command the servo has just
issued (`_last_cmd_ppb`, which is 0 after a reset). The history is cleared on a clock step and while the option is off.
The baseline overlay never uses it. Measured (`default_deterministic`, 100 µs step, +20 ppm, 600 s, 0.7 / 0.3, band
±1 µs; settling / overshoot / range resets; `pi_per_second` has `kp_dt_max` = 1, so kp_eff = 0.5 at 2 s):

| controller | Sync | option off | option on |
|---|---|---|---|
| baseline_pi | 2 s | diverges, 140 resets | 26.5 s, 87 %, 0 |
| baseline_pi | 1 s | 13.7 s, 24 % | 10.9 s, 21 % |
| baseline_pi | 250 ms | 14.6 s, 43 % | 15.1 s, 43 % |
| pi_per_second | 2 s | diverges, 276 resets | 12.7 s, 108 %, 0 |
| pi_per_second | 1 s | 13.7 s, 24 % | 10.9 s, 21 % |

and on `noisy_seed1` (5 seeds, 600 s, band ±2 µs; settled seeds / median settling / median final RMS):

| controller | Sync | option off | option on |
|---|---|---|---|
| baseline_pi | 2 s | 0/5, 1129 resets | 3/5, 534 s, 671 ns, 0 resets |
| baseline_pi | 1 s | 5/5, 13.1 s, 288 ns | 5/5, 10.5 s, 295 ns |
| baseline_pi | 62.5 ms | 5/5, 14.4 s, 202 ns | 5/5, 14.4 s, 285 ns |
| pi_per_second | 2 s | 0/5, 1398 resets | 3/5, 510 s, 605 ns, 0 resets |
| pi_per_second | 62.5 ms | 5/5, 12.0 s, 125 ns | 5/5, 12.0 s, 182 ns |

It removes the divergence, but it is not a free improvement: the rate comes from a single pair of Syncs, so it carries
their timestamp noise (a short estimate puts the correction noise at the order of the timestamp noise, whatever the Sync interval), and the
final RMS grows by 15-45 % at short Sync intervals; this is why it is off by default and meant for Sync ≥ 1 s. At 2 s the
loop is stable but poorly damped (87-108 % overshoot quiet), and in the noisy runs the 2 seeds that do not settle in
±2 µs are noise-limited, not diverging: over 1200 s the RMS stays at 450-960 ns in every 300 s window (peaks 2-6 µs)
with no trend and no resets.

`pi_gain_scheduled` (experimental) is `pi_time_aware` with a bandwidth that depends on the size of the offset, `wn(s) = wn_min·(wn_max/wn_min)^x`
with `x = clip(ln(s/e_lo)/ln(e_hi/e_lo), 0, 1)` (defaults 0.2 → 1.2 rad/s between 1 µs and 100 µs), `kp = 2ζ·wn`, `ki = wn²`, `I += ki·dt·e`, clamp and
conditional integration as in `pi_time_aware` (plus its optional integrator limit `i_max_ppm`, off by default), and `wn ≤ wn_ts_max/dt`. The scheduling variable `s` is the median of the last three `|e|` (one noisy sample
cannot speed the loop up) with a slow release (`tau_release_s`, 4 s: it rises at once and falls exponentially), so the gain does not chatter around a steady
state set by the jitter. `I` is stored in ppb, so a change of `ki` does not move the command (bumpless by construction). Gain scheduling has no general
stability guarantee: it relies on `s` varying slowly against the loop. The slow bandwidth at small error tracks a drift of the oscillator frequency
more slowly (for a frequency ramp the offset settles at ramp rate / `wn_min²`); and at Sync 1 s the guard `wn_ts_max/dt = 0.35` dominates, so it behaves like
`pi_time_aware` in the transient. Measured with `bench/gain_scheduled.py` (300 s, 100 µs / 20 ppm unless stated, band 1 µs deterministic and 2 µs noisy, noisy rows = mean of
seeds 1-5, RMS over the last 60 s; one run per row, no confidence intervals):

| scenario | controller | settling [s] | overshoot [µs] | RMS final [ns] |
|---|---|---|---|---|
| 100 µs, Sync 250 ms | `baseline_pi` | 14.58 | 69.1 | 0.7 |
| 100 µs, Sync 250 ms | `pi_time_aware` | 9.60 | 19.2 | 0.6 |
| 100 µs, Sync 250 ms | `pi_gain_scheduled` | 7.52 | 6.1 | 0.2 |
| 20 µs, Sync 250 ms | `baseline_pi` | 12.16 | 32.3 | 0.7 |
| 20 µs, Sync 250 ms | `pi_time_aware` | 8.07 | 7.3 | 0.6 |
| 20 µs, Sync 250 ms | `pi_gain_scheduled` | 10.30 | 8.4 | 0.2 |
| 100 µs, Sync 1 s | `baseline_pi` | 13.72 | 39.1 | 0.6 |
| 100 µs, Sync 1 s | `pi_time_aware` | 19.50 | 6.7 | 0.5 |
| 100 µs, Sync 1 s | `pi_gain_scheduled` | 19.50 | 6.7 | 0.4 |
| 100 µs, Sync 62.5 ms | `baseline_pi` | 17.25 | 100.3 | 0.6 |
| 100 µs, Sync 62.5 ms | `pi_time_aware` | 9.52 | 16.9 | 0.5 |
| 100 µs, Sync 62.5 ms | `pi_gain_scheduled` | 9.84 | 17.8 | 0.5 |
| 100 µs, noisy, Sync 250 ms | `baseline_pi` | 11.64 | 64.8 | 223.9 |
| 100 µs, noisy, Sync 250 ms | `pi_time_aware` | 7.78 | 14.9 | 212.2 |
| 100 µs, noisy, Sync 250 ms | `pi_gain_scheduled` | 6.78 | 10.5 | 100.0 |
| 100 µs, noisy, Sync 1 s | `baseline_pi` | 12.20 | 34.4 | 227.9 |
| 100 µs, noisy, Sync 1 s | `pi_time_aware` | 15.30 | 5.9 | 185.9 |
| 100 µs, noisy, Sync 1 s | `pi_gain_scheduled` | 15.30 | 5.9 | 119.5 |
| 100 µs, noisy, Sync 62.5 ms | `baseline_pi` | 14.07 | 94.2 | 224.2 |
| 100 µs, noisy, Sync 62.5 ms | `pi_time_aware` | 8.03 | 15.4 | 148.8 |
| 100 µs, noisy, Sync 62.5 ms | `pi_gain_scheduled` | 8.03 | 15.7 | 81.0 |

Reading: with noise the RMS at steady state is 40-50 % lower than `pi_time_aware` (100 vs 212 ns at 250 ms) with a faster or equal transient; with a small initial offset (20 µs) the transient is *slower*
(10.3 s vs 8.1 s) because the bandwidth is already low when the error is small. At Sync 1 s there is no difference in the transient. The defaults are not tuned for a drifting
oscillator; the numbers do not cover the NXP actuator or packet loss.

All feed the same firmware servo state machine (lock, outlier rejection, step, reset). The integrator limit
`i_max_ppm` of `pi_per_second` (0 = off) must exceed the steady frequency correction, otherwise
the offset settles at `(correction − i_max)/kp`.

**Experimental servo options** (`FirmwareConfig`, applied around any controller):
`cmd_clamp_ppm` (not in the firmware, 0 = off) saturates the command to ±limit between the controller and
`ppb_to_scaled_ppm`, so a request beyond the actuator window is applied at the limit instead of being rejected
(→ servo reset); non-finite requests are not clamped and still reset the servo. `step_threshold_ns` is
`SYNC_SERVO_STEP_THRESHOLD_NS` (1 s) made configurable. The baseline overlay of the GUI always runs the unmodified
firmware (baseline PI 0.7/0.3, `FirmwareConfig()` defaults). Gain changes during a live run
use an explicit integrator policy (`keep`, `reset`, `bumpless`).
Loop analysis: `ptpsim.analysis.baseline_poles(kp, ki, Ts)` gives the poles of the ideal loop
(trace = 2 − Ts(kp+ki), det = 1 − Ts·kp); the simulated loop with the firmware's delay estimator is *less*
stable than that (README finding 3).

## 7. Metrics

See the README and the docstring of `ptpsim/metrics.py` for the exact definitions.

## 7b. Oscilloscope view (`ptpsim/scope.py`)

Two ideal square waves at 1 kHz (50 %): the GM one has its rising edges at `k·P` of the physical time; the slave one
has them whenever the *slave clock reads* `k·P`. The scope triggers on the GM edge, so a sweep shows the slave edge at
`d` with `d·(1 + slope) = −A`, where `A` is the phase of the slave-clock segment containing the edge, extrapolated to the
trigger and reduced to `[−P/2, P/2)`. The solve is exact (a bisect on the segment breakpoints, repeated if the edge falls in
the next segment); the first-order `d = −φ` is wrong by `slope·φ` (2.4 ns at 20 ppm and 120 µs). The integer part of the
phase (a PHC at 0 against a 1.7e18 ns epoch) is reduced modulo `P` *as an integer* before the float part is added, so
the picture keeps ns resolution. When `|φ| > P/2` the wave only shows `φ mod P` (the sweep is flagged `ambiguous`).
Nothing is stored per edge: any sweep of the run is computed on demand, so the playback length is the simulated duration. The result carries the slave-clock segments (int64 anchors, integer and float phase, slope): 1,189 segments for the default 300 s run (38 kB of 610 kB), 14,389 for 3,600 s (0.46 MB of 2.5 MB); building the model takes 0.06–0.3 ms; a frame with persistence (up to 2,000 sweeps, vectorised) takes ≈ 11 ms including the offscreen repaint, 4.5 ms without persistence (1400×800 window, 300 s run).

**Persistence** follows what the display of a digital scope does (variable persistence, as documented e.g. by Keysight for the Infiniium/FlexDCA: new points at full intensity, older ones fade to black over the persistence time; infinite persistence: no fading, until a setting changes). Each trigger draws one sweep; a sweep stays on screen for the persistence time and its weight falls linearly from 1 (newest) to 0 at that age, where it is erased. Persistence is in *simulated* time, so the picture depends on the sweep number only (scrubbing, rewinding and the playback speed give the same image). The sweeps of the window are accumulated per screen column as weighted statistics of the square waves (`edge_profile`: weight of the waves high in the column, weight of the edges in the column, total weight; additive), and the brightness is a saturating function of the accumulated weight (`phosphor`: one fresh trace ≈ 0.3, overlapping traces brighten towards 1; the LUT goes black → channel colour → pale), which gives the intensity grading: rare positions dim, frequent ones bright. A window longer than 2,000 sweeps is thinned on a fixed grid. The display has a memory origin: changing the persistence, pressing the reset button or loading a new result makes it start from the current sweep, as if nothing had been drawn before (sweeps before the origin do not exist for the window), and moving back before the origin restarts it there. Infinite persistence cannot be recomputed from scratch (3.6 M sweeps in an hour): it accumulates the sweeps the playback passes over since the origin; a new timebase rebuilds it from the origin with at most 2,000 sweeps. The auto timebase looks at the sweeps of the last 60 s of the window.

Approximations: the slave wave comes from the average-rate clock model (it ignores the NXP timer tick, see §5) and has
no output jitter. The oscilloscope shows Exploration results only (live mode would need the breakpoint stream
in `LiveSession.delta()`; not done). Tests: `tests/test_scope.py` (identical clocks, sign, linear drift, exact solve
within 1 ps against the clock, phase step, huge offset, ambiguity), `tests/test_gui.py` (transport, timebase,
persistence, main-window wiring); the vectorised sweeps equal the scalar ones bit for bit, `edge_profile` matches the square wave column by column and is additive.

## 8. Test matrix (CLAUDE.md §10 → tests)

| Requirement | Test |
|---|---|
| identical clocks, symmetric net, no noise | `test_identical_clocks_symmetric_network_no_noise` (+ zero-delay quirk) |
| known offset, same rate | `test_known_initial_offset_same_rate_no_control` |
| known frequency error, no control | `test_known_frequency_error_no_control` |
| phase continuity on rate change | `test_phase_is_continuous_when_rate_changes`, `test_clock_phase_continuity_...` |
| timestamps before Follow_Up / Delay_Resp | `test_t2_is_taken_at_sync_arrival...`, `test_t3_is_taken_at_delay_req_transmission...` |
| association, missing/stale data | `test_pairing_both_orders_and_stale_replacement`, `test_follow_up_loss...`, `test_delay_resp_for_unknown_request...`, `test_step_clears_...` |
| rate varying between t2 and t3 | `test_rate_variable_between_t2_and_t3_biases_delay_estimate...` |
| simultaneous events | `test_event_order_is_deterministic_at_equal_times`, `test_simultaneous_events_end_to_end_reproducible` |
| asymmetry bias | `test_asymmetry_biases_true_offset_by_half_the_difference` |
| saturation / anti-windup | `test_variant_saturates_with_anti_windup...`, `test_anti_windup_conditional_integration`, `test_out_of_window_command_resets...`, `tests/test_servo_options.py` (command clamp, integrator limit) |
| interval changes | `test_live_interval_changes`, `test_delay_req_first_random...`, `test_every_n_sync_mode...` |
| long-run precision | `test_time_precision_on_a_million_seconds`, `test_precision_of_integer_timestamps_at_large_epoch` |
| NXP arithmetic vs C | `tests/test_c_reference.py` |
| jitter distribution, seeds | `test_jitter_distributions`, `test_stream_value_depends_only_on_index...`, `test_same_exogenous_disturbances_for_different_controllers` |
| firmware baseline vs firmware code | `test_c_reference.py` (PI, conversions), `test_matches_discrete_closed_loop_recursion_exactly`, lock/outlier tests |

## 9. 24 MHz versus the hardware figure of PR #121108

`python scripts/nxp_root_sweep.py` (noisy preset, baseline PI, 5 seeds): median |true offset| [ns], last 200 s.

| clock root | 0 ppm | 0.001 ppm | 0.003 ppm | 0.01 ppm | 0.03 ppm | 0.1 ppm | 0.5 ppm | 2 ppm | 20 ppm |
|---|---|---|---|---|---|---|---|---|---|
| 24 MHz | 0 | 200 | 733 | 2741 | 6616 | 3496 | 4017 | 3739 | 2953 |
| 98.304 MHz | 158 | 153 | 150 | 157 | 154 | 157 | 159 | 161 | 162 |
| 100 MHz | 159 | 159 | 159 | 159 | 159 | 159 | 159 | 159 | 158 |
| 196.608 MHz | 159 | 156 | 157 | 155 | 154 | 150 | 156 | 162 | 154 |
| PR #121108 (hardware) | 24 MHz: 236 | 98.304 MHz: 181 | 100 MHz: 144 | 196.608 MHz: 152 |

At 24 MHz the nominal pair (INC 41 / INC_CORR 43 / ATCOR 2) realises the nominal rate exactly, and the nearest other
reachable rates are ±63 ppm away, so with any non-zero oscillator error the servo alternates between them. In the
model the median offset exceeds the 236 ns of the PR text above ≈ 0.002 ppm of relative frequency error. The other
three roots give the same ≈ 155 ns whatever the error (their rate resolution is fine; the level is set by the
noise of the preset, not by the hardware), i.e. close to the 144–181 ns reported on hardware — a coincidence of the
preset, not a validation. Open question for the author of the measurement: relative frequency error, shared reference?
