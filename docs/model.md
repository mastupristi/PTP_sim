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

| | `baseline_pi` (firmware) | `pi_time_aware` (experimental) |
|---|---|---|
| law | `I += ki·e; u = kp·e + I` | `u = kp·e + I; I += ki·dt·e` (conditional integration) |
| `e` | −offset [ns] | −offset [ns] |
| parameters | `kp` [ppb/ns], `ki` [ppb/ns per update] | `wn` [rad/s], `zeta`; `kp = 2ζ·wn` [s⁻¹], `ki = wn²` [s⁻²] |
| dt | none | measured from consecutive `t1` (GM timestamps), clamped |
| stability | none (Kconfig: tuned for ~1 s) | `wn·dt ≤ wn_ts_max` (0.35 rad): `kp·dt < 2` |
| saturation | none: driver rejects → `clock_servo_reset` | clamp ±`sat_ppb` (400 000 ppb) + anti-windup |
| output | absolute ppb | absolute ppb |

Both feed the same firmware servo state machine (lock, outlier rejection, step, reset). Gain changes during a live run
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
| saturation / anti-windup | `test_variant_saturates_with_anti_windup...`, `test_anti_windup_conditional_integration`, `test_out_of_window_command_resets...` |
| interval changes | `test_live_interval_changes`, `test_delay_req_first_random...`, `test_every_n_sync_mode...` |
| long-run precision | `test_time_precision_on_a_million_seconds`, `test_precision_of_integer_timestamps_at_large_epoch` |
| NXP arithmetic vs C | `tests/test_c_reference.py` |
| jitter distribution, seeds | `test_jitter_distributions`, `test_stream_value_depends_only_on_index...`, `test_same_exogenous_disturbances_for_different_controllers` |
| firmware baseline vs firmware code | `test_c_reference.py` (PI, conversions), `test_matches_discrete_closed_loop_recursion_exactly`, lock/outlier tests |
