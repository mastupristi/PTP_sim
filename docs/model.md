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

| | `baseline_pi` (firmware) | `pi_time_aware` (experimental) | `pi_anti_windup` (experimental) |
|---|---|---|---|
| law | `I += ki·e; u = kp·e + I` | `u = kp·e + I; I += ki·dt·e` (conditional integration) | `I += ki·e; I = clamp(I, ±i_max); u = kp·e + I` |
| `e` | −offset [ns] | −offset [ns] | −offset [ns] |
| parameters | `kp` [ppb/ns], `ki` [ppb/ns per update] | `wn` [rad/s], `zeta`; `kp = 2ζ·wn` [s⁻¹], `ki = wn²` [s⁻²] | as baseline + `i_max_ppm` [ppm] (0 = off) |
| dt | none | measured from consecutive `t1` (GM timestamps), clamped to `dt_max_s` | none |
| stability | none (Kconfig: tuned for ~1 s) | `wn·dt ≤ wn_ts_max` (0.35 rad): `kp·dt < 2` | `kp_eff = min(kp, kp_dt_max/dt)` (`kp·dt ≤ 1` by default) |
| saturation | none: driver rejects → `clock_servo_reset` | clamp ±`sat_ppb` (400 000 ppb) + anti-windup | integrator only (output limited by the servo's command clamp, if on) |
| output | absolute ppb | absolute ppb | absolute ppb |

`pi_per_second` (experimental) is `pi_anti_windup` with `ki_eff = ki·dt/t_ref_s` (`dt` measured from consecutive `t1`,
clamped to the absolute `dt_max_s` (10 s); `t_ref_s = 1 s` by default, the interval the firmware gains are tuned for). `kp` is
not scaled: ppb/ns is already s⁻¹. `ki/t_ref_s` [s⁻²] is thus constant and the continuous damping
`ζ = kp / (2·sqrt(ki/t_ref_s))` = 0.64 for any Sync interval, whereas in the baseline it is `0.7 / (2·sqrt(0.3/T))`:
0.64 at 1 s, 0.32 at 250 ms, 0.16 at 62.5 ms. At `dt = t_ref_s` (and `kp ≤ kp_dt_max`) it is bit-identical to `pi_anti_windup`. Measured
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
alternates in sign at every update and the delay estimate swings by ±70 µs (true delay: 1 µs). Reaching it would need a
different change (e.g. rejecting or filtering delay samples taken across a large rate change), not a limit on `kp·dt`.

All feed the same firmware servo state machine (lock, outlier rejection, step, reset). With `i_max_ppm = 0`
`pi_anti_windup` is bit-identical to `baseline_pi`; `i_max` must exceed the steady frequency correction, otherwise
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
