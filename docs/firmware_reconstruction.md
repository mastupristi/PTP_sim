# Firmware reconstruction (phase 1)

This document reconstructs the closed control loop of the Zephyr PTP time receiver
that PTP_sim reproduces. Every statement is tied to a file, a function and a commit.
Nothing here is inferred from generic PTP formulas: where the firmware deviates from
IEEE 1588 the deviation is stated.

## Reference commits

| Ref | SHA | Content |
|---|---|---|
| `master` | `8f62a4ab82b5c2121c088842d3233f75b603ce35` | upstream base used for the PTP library |
| `zmagnifico-integration-2` | `553d973f1b78f64a9f7e49bbb2c8b7acee8701db` | master + the four PRs (**baseline used by the simulator**) |
| `ptp-timer-fractional-tick` | `c273dca92e632d58ba17b92db03f1a0dc2787dd2` | PR #121108 (NXP rate/fractional tick), identical driver files to the integration branch |
| `ptp-timer-root-dt` | `3768c664918bed1e23ca6485eff0b4e982afc476` | PR #121109 (clock root from devicetree) |
| `nxp-enet-rx-drain` | `cdea9cdd3f1994c64b58e573c900861ebb801629` | PR #121089 (RX ring drain) |
| `ptp-unsupported-version` | `caebea4b3aec3abc019c8eef4894d63fd5b0834f` | PR #121088 (invalid messages) |
| NXP HAL (`modules/hal/nxp`) | `608b9ff3c26fa8a8292dbccdb20eca190ba4b4e9` | `fsl_enet.c` timer functions |

`git diff master zmagnifico-integration-2` over `subsys/net/lib/ptp`, `subsys/precision_timing`
and `include/zephyr/precision_timing` only touches `msg.c`, `msg.h` and `port.c`
(`ptp_port_event_gen`, PR #121088: malformed messages are dropped instead of faulting the port).
`clock.c`, the PI and the precision clock layer are identical on both. Line numbers below refer
to the integration branch (add 9 to `port.c` lines above 1860 for `master`).
The Zephyr working tree was never modified: all reads used `git show`/`git grep` on refs.

## What was **not** found (declared explicitly)

* The debug instrumentation that printed `t1/t2` (Follow_Up), `t3/t4` (Delay_Resp), delay and
  offset is **not present** in any local branch, in the working tree or in the reflog commits
  (`git log --all -i --grep=debug|trace|instrument` on `subsys/net/lib/ptp` and `drivers/ptp_clock`
  only finds PR #121088; the reflog commits are the PR commits before rebasing). It lives outside
  the repository.
* Consequence: **no real log format is available**, so no log importer was written and no
  hardware validation is claimed. The simulator's timestamp CSV export uses its own format.
  Real logs (raw files) are needed to add an importer and to validate the baseline against hardware.
* The application-level Kconfig/overlay (clock-root frequency of the board, `PTP_SYNC_LOG_INTERVAL`,
  `PRECISION_TIMING_PI_*` overrides) is outside the repository. Defaults from Kconfig are used and
  all are configurable in the simulator.

## 1. Role and message flow

The PTP library implements both ends (`subsys/net/lib/ptp/port.c`). The simulated firmware is
the **time receiver** (slave) in state `PTP_PS_TIME_RECEIVER`/`PTP_PS_UNCALIBRATED`, E2E delay
mechanism (`PTP_DM_E2E`), two-step, with the hardware PHC (`ptp_clock_nxp_enet`) as the only clock
that is disciplined. BMCA/announce/state-machine transitions are not simulated (see limitations).

## 2. Timestamps and clock domains

| Quantity | Origin | Domain | Where |
|---|---|---|---|
| `t1` | `preciseOriginTimestamp` of the Follow_Up = HW TX timestamp of the Sync taken by the GM | GM | `port_sync_timestamp_cb` (port.c:512), copied at 583-585 |
| `t2` | HW RX timestamp of the **Sync** (`msg->timestamp.host`) | slave PHC | `port_synchronize` (port.c:235) arguments `ingress_ts` |
| `t3` | HW TX timestamp of the Delay_Req, set by `port_delay_req_timestamp_cb` (port.c:419) into `req->timestamp.host` | slave PHC | |
| `t4` | `receiveTimestamp` of the Delay_Resp = GM HW RX timestamp of the Delay_Req | GM | `port_delay_resp_msg_process` (port.c:1216) |

* `t2` and `t3` come from the **same disciplined PHC** whose rate the servo changes.
* A Follow_Up never creates `t2`; a Delay_Resp never creates `t3`.
* Fallback for `t3`: `port_delay_req_msg_transmit` (port.c:723) pre-fills `timestamp.host` with a
  software `ptp_clock_get()` taken at message creation (only with `CONFIG_PTP_IEEE_802_3_PROTOCOL`);
  the TX timestamp callback overwrites it. If the callback has not run when the Delay_Resp is
  processed, the pre-filled value is used.
* A Sync without a valid RX timestamp is dropped (`port_sync_rx_timestamp_valid`, port.c:1021).
* `clock_synchronize_with_delay` (clock.c:765) replaces `t2` by the current PHC time if the ingress
  timestamp is missing or differs more than 5 s from it (`INGRESS_TS_PHC_DELTA_GUARD_NS`).
  Not reachable in normal operation; not simulated.

## 3. Pairing, sequence ids, previous samples

* **Sync/Follow_Up**: `port_sync_fup_ooo_handle` (port.c:973) keeps a single slot
  `port->last_sync_fup`. A Sync followed by a Follow_Up with the same `sequence_id` (or the reverse
  order) completes the pair and calls `port_synchronize`. Any non-matching message **replaces** the
  slot (the older one is dropped). Out-of-order arrival is supported.
  One-step Sync (`TWO_STEP` flag clear) is handled separately and not simulated.
* **Delay_Req/Delay_Resp**: `port->delay_req_list` holds sent requests; `port_delay_resp_msg_process`
  matches by `sequence_id` and by `req_port_id`; no match → ignored. `port_delay_req_cleanup`
  (port.c:941) removes requests older than `PORT_DELAY_REQ_CLEAR_TO` = 3 s, but only when the Delay_Req
  timer fires (port.c:2067-2075).
* **Not associated with the Sync that preceded the Delay_Req**: `ptp_clock_delay` (clock.c:826) uses
  `ptp_clk.timestamp.t1/t2`, i.e. the *latest* completed Sync pair at the moment the Delay_Resp is
  **processed**. It returns without effect while `t1 == 0 || t2 == 0`.
* `clock_step` (clock.c:673) clears `timestamp` and `mean_delay` but does **not** clear
  `delay_req_list`: a Delay_Resp for a request sent before a step pairs a pre-step `t3` with a
  post-step `t2`. Reproduced, not fixed.

## 4. Equations and sign conventions

All arithmetic is `int64` nanoseconds.

* `t1c = t1 + (corr_sync >> 16) + (corr_fup >> 16)`; `correctionField` is ns·2^16 and `>> 16` is an
  arithmetic shift (floor). `corr_sync` also gets `port_ds.delay_asymmetry` added
  (port.c:1090 `port_sync_msg_process`). `t4c = t4 - (corr_delay_resp >> 16)` (port.c:1216).
* Offset (`clock_synchronize_with_delay`, clock.c:765): `offset = (t2 - t1c) - delay` with
  `delay = mean_delay >> 16`. **Positive offset = slave ahead of the GM.**
* Delay (`ptp_clock_delay`, clock.c:826):
  `delay = ((t2 - t3) + (t4c - t1)) / 2` with C `int64` division (truncation toward zero).
  Only rejection: `|delay| > 1 s` (the request is still consumed). Negative delays are accepted.
  **No filter, no averaging**: each sample replaces `mean_delay` (stored as `ns << 16`, clamped to
  the 48-bit range).
* Consequence: `t2` and `t3` are separated by up to one Sync interval. The estimate equals
  `d + (o(t2) - o(t3))/2` (plus asymmetry/2), where `o` is the true offset. It is therefore corrupted
  by the slave clock's phase motion between `t2` and `t3`, even with a perfectly constant network.

## 5. Servo events and update period

* The servo runs **once per completed Sync/Follow_Up pair**, at the processing of the *second* message
  of the pair (`port_synchronize` → `ptp_clock_synchronize`, clock.c:813).
  It does **not** run while `mean_delay == 0` (no valid delay yet, or just after a step).
  The first Delay_Req is sent at a random time, so the servo starts late (see §7).
* Effective period = Sync interval as seen by the receiver (jitter and losses included). No dt is
  passed to the PI.
* Offset steps: `|offset| > 1 s` → `clock_step` (clock.c:673): `target = phc_now - offset`,
  `precision_clock_set`, then `timestamp = 0`, `mean_delay = 0`, `clock_servo_reset()`.
* Acquisition/validity (`clock_servo_update_lock`, clock.c:610; `clock_adjust_rate`, clock.c:721):
  * lock after 3 consecutive servo samples with `|offset| <= 10 ms`;
  * while locked, a sample with `|offset| > 100 ms` is **rejected** (PI untouched, no command);
    2 consecutive rejections → `clock_servo_reset()`;
  * before lock there is no rejection.

## 6. PI controller and rate command

`precision_pi_update` (`subsys/precision_timing/precision_pi.c:22`):

```c
pi->integral += pi->ki * error;
return pi->kp * error + pi->integral;
```

* `kp = CONFIG_PRECISION_TIMING_PI_KP / 1000 = 0.7`, `ki = CONFIG_PRECISION_TIMING_PI_KI / 1000 = 0.3`
  (`subsys/precision_timing/Kconfig`; `clock_init`, clock.c:354). Kconfig help: tuned for a Sync
  interval around **1 s**; shorter intervals "may require a smaller gain".
* `error = -offset` (ns). The output is `ppb` and is an **absolute** frequency correction
  (integrator updated first, then output; integrator initialised to 0 in `precision_pi_init` and on
  `precision_pi_reset`). It is not an increment.
* Units: kp is ppb/ns (= s⁻¹ in continuous terms, since 1 ns/s = 1 ppb); ki is ppb/ns **per update**
  — there is no `dt`. The continuous-time integral gain is `ki / T_sync` (s⁻²). Hence the loop
  has damping that depends on T_sync: continuous approximation `ζ = kp / (2 sqrt(ki/T))`
  = 0.64 for T = 1 s and 0.32 for T = 0.25 s.
* Conversion: `precision_clock_ppb_to_scaled_ppm` (`precision_clock.c:30`):
  `scaled_ppm = (int64)(ppb * 65536 / 1000)` (truncation toward zero; fail if not finite);
  `precision_clock_ptp_adjust_rate` (`precision_clock_ptp.c:77`) → `ratio = 1 + scaled_ppm / (1e6 * 65536)`;
  `ptp_clock_rate_adjust(dev, ratio)`.
* Failure of the conversion or of `adjust_rate` (e.g. the NXP driver rejects `|ratio-1| > 50000 ppm`,
  `CONFIG_PTP_CLOCK_NXP_ENET_MAX_RATIO_PPM`) → `clock_servo_reset()`: integrator = 0, rate back to
  `ratio = 1.0` (`adjust_rate(0)`), lock cleared. There is no clamp/saturation and no anti-windup.
* Sign: `offset > 0` (slave ahead) → `error < 0` → `ratio < 1` → fewer ns per tick → slave slows.
  Negative feedback, verified against the NXP driver (average tick = nominal·ratio).

## 7. Timing of Sync and Delay_Req

* Sync interval (`log_sync_interval`): taken from the Sync header `logMessageInterval` for multicast
  Sync (port.c:1090-1114, accepted range -10..22); the Kconfig range of the default is -1..1 but
  the header value wins.
* Delay_Req interval: `log_min_delay_req_interval`, overwritten by the `logMessageInterval` of
  each received Delay_Resp (port.c:1258-1273).
  * First Delay_Req timer after entering TIME_RECEIVER: `port_timer_set_timeout_random(delay, 0, 2, log)`
    (port.c:216, 1993): uniform in (0, 2·2^n] s.
  * Afterwards, at each expiry: `port_timer_set_timeout(delay, 1, log)` (port.c:2072) → period 2^n s
    **restarted at handling time** (no catch-up of handler latency).
  * The timers are Zephyr `k_timer`: the **monotonic kernel clock, not the disciplined PHC**. Their
    rate does not depend on the servo.
  * The firmware has **no "every N Sync" mode**. The simulator offers it as an explicitly labelled
    non-firmware option. Nominal 2 s = 8 Syncs only when Sync = 2^-2 and Delay_Req = 2^1.
* Sync reception also re-arms the Sync receipt timeout (`port_synchronize`); a timeout leads to
  `PTP_EVT_ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES` and a port-state decision. Not simulated.

## 8. NXP actuator (PR #121108, `drivers/ptp_clock/ptp_clock_nxp_enet.c`)

* Timer start (`ENET_Ptp1588StartTimer`, HAL `fsl_enet.c:2982`): `ATINC.INC = 1e9 / clock_rate` (integer
  division, **truncated**), `ATPER = 1e9`. Seconds are counted by software.
* `ptp_clock_nxp_enet_rate_adjust` (driver:143): rejects `ratio` outside `1 ± MAX_RATIO_PPM·1e-6`
  with `-EINVAL`; otherwise `target_ns = (1e9 / clock_rate) * ratio` and
  `ptp_clock_nxp_enet_find_correction(inc, target_ns, 127, 0x7FFFFFFF, &inc_corr, &cor)`
  (`ptp_clock_nxp_enet_rate_math.c`) then `ENET_Ptp1588AdjustTimer(base, inc_corr, cor)`
  (HAL: writes `ATINC.INC_CORR` and `ATCOR`).
* Semantics: every tick adds `INC`, except one tick out of every `ATCOR + 1`, which adds `INC_CORR`.
  Average tick `avg = inc + (inc_corr - inc) / (cor + 1)` (`cor != 0`), else `inc`. The `+1` was measured
  on an i.MX RT1170 (commit message of `c273dca92e6`), not stated by the reference manual.
* The pair for `ratio = 1.0` is programmed once at timer start (makes up the cut fractional part).
  `ratio = 1.0` still goes through the search (no early return).
* Search: for each `delta = |INC_CORR - INC|` in `1..delta_max` it tries the two integer periods around
  `delta/frac`; `ATCOR = period - 1`, `period >= 2`; keeps the smallest `|delta/period - frac|`.
  A fraction too small for any period → correction off (`ATCOR = 0`).
* Rate resolution therefore depends on the clock root: 100 MHz (whole tick) is fine-grained; 24 MHz
  can leave tens of ppm of residual (PR test limits: 24 MHz ≤ 40 ppm, 98.304/196.608 MHz ≤ 5 ppm).
* Timestamps are values of the free-running timer in ns: their resolution is the tick (INC ns).

## 9. Uncertainties and approximations carried by the simulator

1. GM implementation, link layer (transparent clocks, `correctionField`) and Sync TX jitter origin are
   not in the repository; they are modelled generically and are configurable.
2. Clock root frequency of the real board is unknown (the default 100 MHz comes from the SoC code,
   `soc/nxp/imxrt/imxrt11xx/soc.c`); configurable.
3. The ENET correction counter phase after `ATCOR` changes is not documented; the average-rate model
   ignores it (see README, "approximations").
4. Real-log validation: not possible without the logs (see above).
