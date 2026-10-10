# PTP_sim user manual (English)

Italian version: [manual.it.md](manual.it.md). Model and firmware background: [model.md](model.md),
[firmware_reconstruction.md](firmware_reconstruction.md).

## 1. What you are looking at

PTP_sim simulates a PTPv2 **time receiver (slave)** locked to a **Grandmaster (GM)**. The GM clock is the
reference: its rate is constant and *physical simulation time = GM time*. The slave has its own clock (the PHC)
whose **rate** is changed by a servo (a PI controller) — the phase is never jumped, except by the firmware's own
forced alignment (see *Initial offset*).

Four messages are exchanged: **Sync** and **Follow_Up** (GM → slave, every Sync interval), **Delay_Req**
(slave → GM) and **Delay_Resp** (GM → slave, every Delay_Req interval). Four timestamps result:

| | meaning | clock |
|---|---|---|
| `t1` | GM transmit time of the Sync (carried by the Follow_Up) | GM |
| `t2` | slave receive time of the Sync | **slave (disciplined)** |
| `t3` | slave transmit time of the Delay_Req | **slave (disciplined)** |
| `t4` | GM receive time of the Delay_Req (carried by the Delay_Resp) | GM |

The firmware estimates `delay = ((t2−t3)+(t4−t1))/2` and `offset = (t2−t1) − delay` (positive = slave ahead).
Because `t2` and `t3` come from a clock whose rate the servo is changing, the estimates are affected by the
control itself — this is the point of simulating the closed loop.

**Always keep two quantities apart:**

* **True offset** (blue): slave − GM at the same *physical* instant. Known only to the simulator.
* **Estimated offset** (orange dots): what the firmware computes from the timestamps and feeds to the PI. It is
  available only at the Sync samples, and it can differ from the true offset (stale delay, asymmetry, noise,
  clock motion between `t2` and `t3`).

The controller never sees the true values.

## 2. The window

```
┌ tabs (parameters) ┐ ┌ toolbar: units, view, fit, status, language ─────────────────────────────────────┐
│ Run               │ │ show: overlay, estimated, diagnostics, PI terms, transient line                  │
│ Controller        │ │ plot 1: Delay                                                                    │
│ PTP intervals     │ │ plot 2: Offset   (all plots share the time axis, zoom/pan with mouse wheel/drag) │
│ Scenario          │ │ plot 3 (optional): Rate diagnostics                                              │
│ Network and noise │ │ plot 4 (optional): PI terms (P, I, controller output, applied command)           │
│ Actuator          │ │ metrics table (always fully visible)                                             │
└───────────────────┘ └──────────────────────────────────────────────────────────────────────────────────┘
```

**Language** (top right): English (default) or Italian. Switching rebuilds the window in the new language and keeps
the current configuration (a running live session is restarted). The choice is remembered.

**Plots.** Mouse wheel = zoom, drag = pan, right-click = pyqtgraph menu, double-click on the corner "A" = auto-range.

* *Delay plot*: the firmware's delay estimate (green) is **held** until the next Delay_Resp is processed, with a dot at
  every sample (so the real sampling rate is visible); the dashed black line is the physical delay of the network.
* *Offset plot*: true offset (blue), estimated offset (orange), optionally the baseline (pink, dashed) for comparison:
  the **unmodified firmware** (baseline PI 0.7/0.3, `clock.c` constants, no command clamp) on the same scenario and seed.
  Vertical dotted red lines mark servo steps/resets; vertical dashed grey lines (live mode) mark parameter changes.
* *Rate diagnostics*: the ppb commanded by the servo vs the actual rate error of the clock relative to the GM
  (includes the oscillator error).
* *PI terms* (ppm): at every servo update the proportional term **P = kp·e**, the integrator **I** and the **controller
  output** (P + I for the PI laws, before the command clamp); the **command applied** to the clock (held, green) shows the
  clamp and the return to nominal at every servo reset. Steps and rejected outliers have no update, so no point.
  Horizontal lines mark the active limits: actuator ±50 000 ppm, command clamp, integrator limit (`pi_anti_windup`),
  controller limit (`sat_ppb` of `pi_time_aware`); they do not enter the y auto-range. Use it to size an anti-windup
  limit: how large I gets while the command is clamped, and the steady I it must still be able to hold.
* **Transient-end line** (dashed vertical, labelled): the instant at which the true offset enters the settling band
  and stays there for the dwell time (§4). On the Offset plot the line of the baseline overlay is drawn in pink.
* **View** selector: *Full* (whole run), *Transient* (from t=0 — or from just before the servo start after a forced
  alignment — to 1.3 × the transient end), *Steady state* (from the transient end to the end of the run). The y axis
  always fits the visible data. "Fit view" resets to the full run.
* **Units** (ns / µs / ms) apply to the plots. The metrics table chooses the most readable unit itself.

**Metrics table** (Exploration mode): see §4. Columns: true offset, estimated offset and — with the overlay on — the
same for the baseline.

## 3. Parameter reference

Each entry gives: *what it is*, *unit/range*, and the JSON path in a saved configuration (§6). Values that the
firmware reads from Kconfig/devicetree are noted.

### 3.1 Tab "Run"

**Mode**
* **Exploration**: any parameter change recomputes the *whole* trajectory from the same initial conditions and the same
  seed. Use it to compare tunings.
* **Live**: the trajectory continues in (simulated) time; changes apply **from the current instant**, marked by a dashed
  line. Buttons **Start / Pause / Reset**; changes made before Start restart the session from t = 0.
* **Playback speed** (live): simulated seconds per real second (0.1×–500×). It only changes how much simulated time is
  requested per tick; the simulation maths is identical at any speed.
* **Integrator policy when gains change** (live): what happens to the controller's integral state when you change gains
  or switch controller: *keep* (leave it untouched / carry the value over), *reset* (zero it), *bumpless* (recompute it so the
  output does not jump). Nothing is ever reset silently.
* **Follow / Live window**: keep the time axis on the last N seconds.

**Metrics settings** (also in §4)
* **Settling band (±)** [ns]: the transient is over when |offset| stays inside ±band.
* **Dwell time** [s]: how long it must stay inside to count.
* **Steady-state window (last)** [s]: the last W seconds used for steady-state statistics.
* **Steady state = after the transient end**: use everything after the transient end instead of the last W seconds.

**Files**: *Save config/seed* writes the JSON of the current scenario (seed included); *Load config* restores it exactly
(values outside a widget's range or per-message-type settings are preserved until you edit that control); *Export CSV*
writes `params.json`, `metrics.json`, `servo_samples.csv`, `delay_samples.csv`, `true_offset.csv`, `rate.csv`, `events.csv`.
`servo_samples.csv` holds, per servo sample, the controller output `cmd_ppb`, the `integral`, the `action` and — last two
columns — the command accepted by the driver `cmd_applied_ppb` (after the clamp) and the proportional term `p_ppb`.

### 3.2 Tab "Controller"

**Controller** — which law drives the clock rate. All of them output an **absolute** frequency correction in ppb
(positive = faster) from the **estimated offset** only.

* `baseline_pi` — port of the firmware (`precision_pi_update`): `integral += ki·e; u = kp·e + integral`, `e = −offset [ns]`.
  * **kp** [ppb/ns]: proportional gain (firmware default 0.7 = `PRECISION_TIMING_PI_KP` 700/1000).
  * **ki** [ppb/ns per update]: integral gain, applied **per sample, with no dt** (firmware default 0.3).
    Consequence: the loop's damping depends on the Sync interval (the Kconfig help says the defaults suit ≈1 s).
  * No saturation: a command beyond ±50 000 ppm is rejected by the NXP driver and the firmware resets the servo
    (unless the experimental *command clamp* below is on; the integrator then winds up, there is no anti-windup).
* `pi_time_aware` — experimental PI with explicit time and anti-windup.
  * **wn** [rad/s]: closed-loop natural frequency; `kp = 2ζ·wn` [1/s], `ki = wn²` [1/s²].
  * **zeta** []: damping ratio (1 = critically damped).
  * **sat_ppb** [ppb]: clamp of the command (must stay below 50 000 ppm = 50 000 000 ppb); the integrator is frozen while
    saturated and the error pushes further in (anti-windup).
  * **wn_ts_max** [rad]: caps the bandwidth at `wn·dt ≤ wn_ts_max` so the sampled loop stays stable (`kp·dt < 2`).
  * **dt_clamp** []: the interval measured from consecutive `t1` is clamped to `dt_clamp ×` the nominal Sync interval
    (protects against lost messages).
* `pi_per_second` — `pi_anti_windup` with the integral gain scaled by the Sync interval: `integral += (ki·dt/t_ref)·e`.
  Use it when you change the Sync interval and want the loop shape to stay the same: **kp** and **ki** are the gains
  tuned at **t_ref**, and `ki/t_ref` [s⁻²] is held constant (**kp** is not scaled: ppb/ns is already 1/s).
  * **t_ref_s** [s]: interval at which kp, ki are tuned (default 1 s, the interval the firmware gains are tuned for).
    At `dt = t_ref_s` the law is identical to `pi_anti_windup`; at 250 ms the integrator adds ki/4 per update.
  * **dt_max_s** [s]: the measured interval (from consecutive `t1`, so a lost Sync gives a longer step) is clamped to
    this absolute value (default 10 s). Keep it ≥ the nominal Sync interval, or regular steps are clamped too.
    **i_max_ppm** as in `pi_anti_windup`.
  * Not guarded: `kp·dt < 2` (as in the baseline); at Sync = 2 s the run diverges in the quiet 100 µs scenario (cause not analysed).
* `pi_anti_windup` — the firmware PI law (same per-update **kp**, **ki**, no dt) with an integrator limit:
  `integral += ki·e; integral = clamp(integral, ±i_max); u = kp·e + integral`.
  * **i_max_ppm** [ppm]: integrator limit; **0 = off**, and the controller is then identical to `baseline_pi`.
    It bounds the windup while the command clamp saturates. It must stay **above the steady frequency correction**
    (oscillator error + drift): with 20 ppm of oscillator error and i_max = 10 ppm, P must supply the other 10 ppm and
    the offset settles at 10 ppm / kp ≈ 14.3 µs instead of 0.

**Firmware servo (clock.c)** — options of the servo around the controller (they apply to every controller):
* **Command clamp** (`firmware.cmd_clamp_ppm`, default 0 = off) — **not in the firmware**: the command is saturated to
  ±this value before the driver, instead of being rejected (→ servo reset) when it exceeds the actuator limit. Capped at
  50 000 ppm in the GUI (above it the driver still rejects). A NaN/infinite command is not clamped: it still resets the servo.
* **Step threshold |offset|** (`firmware.step_threshold_ns`, firmware 1 s = `SYNC_SERVO_STEP_THRESHOLD_NS`): above it
  the firmware steps the clock (forced alignment, §3.4) and resets the servo. The `|delay| > 1 s` rejection is unchanged.

Both can be changed in live mode; the baseline overlay keeps the unmodified firmware values.

Changing the controller in live mode builds the new one with its defaults (plus the values shown); the integrator policy
decides what is carried over.

### 3.3 Tab "PTP intervals"

Intervals follow the PTP rule **interval = 1 s × 2ⁿ**; the resulting value is shown under each control. The GUI offers
n ∈ [−4, 2] — this is a GUI choice, **not** a protocol limit (the firmware accepts `logMessageInterval` in [−10, 22]).

* **Sync: exponent n** (`intervals.sync_log`, default −2 = 0.25 s): Sync and Follow_Up period. The slave learns it from the
  message header (it is not used to time anything else).
* **Delay_Req mode**
  * *Independent interval* (the **firmware behaviour**): the slave sends Delay_Req from its own `k_timer`. The **first**
    request is at a random time uniform in (0, 2·2ⁿ] s after start, then every 2ⁿ s. The timer is a **monotonic clock
    that is not disciplined**: its rate does not follow the servo (see *k_timer error*).
  * *Every N Sync* (**not in the firmware**, offered for study): a Delay_Req after every N-th processed Sync/Follow_Up pair.
    Changing the Sync interval keeps N; in the independent mode, changing the Sync interval does **not** change the Delay_Req
    period (no hidden 8:1 ratio).
* **Delay_Req: exponent n** (`intervals.delay_log`, default 1 = 2 s): the interval **advertised by the GM** in the
  Delay_Resp; the slave adopts it (`log_min_delay_req_interval`) at the next Delay_Resp.
* **Every N Sync: N** (`intervals.delay_every_n`, default 8).
* **Timer re-armed at handling** (`intervals.delay_rearm_from_handling`): the firmware restarts its timer when it *handles*
  the expiry, so handler latency accumulates (cumulative drift); unchecked = jitter around a fixed nominal grid, which
  matches the observed traces.

### 3.4 Tab "Scenario"

* **Initial offset** (`oscillator.initial_offset_ns`; shown in µs; range ±2×10⁹ s with a log-scale slider): slave − GM at t = 0.
  * |offset| ≤ step threshold (1 s in the firmware, Controller tab): the PI handles it (above ≈ 50 ms the baseline asks
    > 50 000 ppm, the driver rejects it and the servo resets in a loop — a real firmware weakness the simulator
    reproduces — unless the command clamp is on).
  * |offset| > step threshold: the firmware performs a **forced alignment** (`clock_step`): it sets the PHC to *now − offset*,
    clears the stored timestamps and the delay estimate and resets the servo. The servo restarts only after a **new
    Delay_Resp** (up to one Delay_Req interval later). The step uses the offset estimated at the first Sync/Follow_Up
    pair that has a delay estimate, so it also needs the first Delay_Resp (random, up to 2·2ⁿ s after start).
  * Real PHCs often start at ~0 while the GM is at the PTP epoch (~1.7×10¹⁸ ns): use **"Slave PHC starts at 0"** to set
    the offset to −epoch. The simulator keeps this exact (integer arithmetic), so the residual after the step is
    ns-accurate.
* **Frequency error** (`oscillator.freq_error_ppb`, shown in ppm): the slave oscillator's rate error relative to the GM
  at t = 0 (what the servo has to compensate).
* **Duration** (`duration_s`): length of the run (in live mode: the end of the session).
* **Seed** (`seed`): all random disturbances derive from it. The *same* seed gives the *same* noise whatever the controller,
  so comparisons are fair.
* **Oscillator drift** (`oscillator.drift_ppb_per_s`): linear frequency drift, applied in 1 s steps
  (`oscillator.update_period_s`); `oscillator.walk_ppb_per_sqrt_s` adds a random walk (JSON only).
* **k_timer error** (`oscillator.timer_error_ppb`, in ppm): rate error of the monotonic timer that schedules the Delay_Req
  relative to physical time. Independent of the PHC.

### 3.5 Tab "Network and noise"

* **Mean delay** and **Asymmetry** (`network.delay_ms_ns`, `network.delay_sm_ns`): one-way delays GM→slave and
  slave→GM are `mean ± asymmetry/2`. PTP assumes they are equal: the estimated offset then has a bias of
  `asymmetry/2` and the **true** offset converges to `−asymmetry/2` (`correctionField`s and `delay_asymmetry` exist in JSON).
* **Path jitter** (`network.jitter_ms/sm`): random extra delay per message and direction (one-sided exponential, mean = value).
  It corrupts `t2`/`t4` directly → noise on the offset estimate.
* **Send jitter** (`tx_jitter.*`): random shift (normal, σ = value, clipped at ±4σ) of the **emission instant** of **each**
  message type around its nominal instant, not cumulative. With hardware timestamps it does **not** corrupt `t1..t4`; it makes
  the sampling irregular and shifts when messages are processed. (JSON: per message type and kind none/uniform/normal/exponential.)
* **Follow_Up latency**, **Delay_Resp latency** (`latency.*`): software delay at the GM between the event that enables the
  message (Sync TX timestamp / Delay_Req reception) and its emission.
* **Command application latency** (`latency.command_ns`): time between the servo's decision and the new rate being
  effective in the hardware. (The driver's accept/reject answer is immediate.) JSON also has `rx_processing_ns` (frame arrival →
  processed by the PTP thread) and `tx_timestamp_cb_ns` (Delay_Req TX → TX timestamp callback; if the Delay_Resp is processed
  first, the firmware uses the pre-filled software time as `t3`).
* **Clock step latency** (`latency.step_ns`): time between reading and setting the PHC inside `clock_step`; the clock is
  that much behind right after a forced alignment (then the servo removes it).
* **Timestamp noise** (`timestamps.gm_noise_sigma_ns`, `slave_noise_sigma_ns`): Gaussian noise on the hardware timestamps.
* **GM timestamp quantisation** (`timestamps.gm_quantum_ns`): resolution of `t1`/`t4`. The slave's quantum is the timer tick
  for the NXP actuator (JSON `timestamps.slave_quantum_ns`; null = automatic).
* **Message loss** (`loss.*`): independent probability for each message type.
* **Noisy preset**: send jitter σ = 20 µs, path jitter 200 ns, timestamp noise 3 ns.

### 3.6 Tab "Actuator"

* **Ideal**: realises exactly the requested rate ratio, inside ±50 000 ppm (the same window as the NXP driver; outside it
  the request is rejected and the servo resets). Isolates the control dynamics.
* **NXP ENET (PR #121108)**: models the i.MX RT ENET 1588 timer: `ATINC.INC` (whole ns of the tick), `INC_CORR`, `ATCOR`.
  Each tick adds `INC`, except one tick in `ATCOR+1` that adds `INC_CORR`; the driver searches the pair that best
  approximates the requested average tick. The achievable rates are therefore **discrete** — very fine at 100 MHz, ≈ 63 ppm
  apart near nominal at 24 MHz. Timestamps are quantised to one timer tick. Average-rate model (the staircase of the counter is
  not reproduced; see model.md).
* **1588 timer clock root**: 100 MHz (SoC default), 98.304 / 196.608 MHz (audio PLL), 24 MHz (OSC_24M), 25 MHz. The panel shows the register
  triple `INC / INC_CORR / ATCOR` of the end of the run (`actuator.clock_hz`, `actuator.max_ratio_ppm` in JSON; the latter =
  `CONFIG_PTP_CLOCK_NXP_ENET_MAX_RATIO_PPM`, default 50 000).

## 4. Metrics (table)

All metrics use full-resolution data. For the true offset and the estimated offset separately (and the baseline overlay):

* **Transient end (settling)**: first instant after which |x| stays ≤ band for the rest of the run, valid only if the remaining
  time ≥ dwell. "–" with a reason otherwise (still outside / in band for less than the dwell time). If the offset never leaves the band,
  0.
* **Overshoot**: largest excursion on the *opposite* side of the offset at the **servo start** (first PI command after the last
  forced alignment), in % and absolute. The clock runs free until the first valid delay sample (≈ 3 s), so the reference is
  not the initial offset. "–" if the reference is zero.
* **Peak |error|** (from the servo start; exact for the true offset).
* **Steady state** (last W seconds, or after the transient end): **median, minimum, maximum, median |x|, RMS, mean (bias)**.
* **Diverged**: non-finite, |x| beyond 1 s after the servo start, or a large final RMS.
* **Delay estimate (steady)**: median / min / max of the firmware's delay estimate in the same window, next to the physical delay.
* **Saturation / resets**: clamps (controller limit or command clamp, once per update), out-of-range resets (command rejected → `clock_servo_reset`), servo resets, clock
  steps, outliers rejected (offset > 100 ms after lock).
* **Compute time**: simulation and metrics time; GUI latency from the change to the end of the repaint.

## 5. Recipes

* **Large initial offset**: Scenario → Initial offset 100 ms (the PI is overwhelmed: resets) or 3 s (forced alignment); or press
  *Slave PHC starts at 0*. Use View → *Transient*.
* **Compare controllers**: select `pi_time_aware`, tick *Overlay baseline*; both see the same noise.
* **Size the anti-windup**: Scenario → Initial offset 100 ms; Controller → Command clamp 1000 ppm; tick *PI terms*. With
  `baseline_pi` the integrator winds up to ≈ 6×10⁶ ppm and the offset overshoots to ≈ −100 ms; select `pi_anti_windup`
  and raise **i_max_ppm** from just above the oscillator error (20 ppm here): at 100 ppm the overshoot is ≈ 44 µs.
* **Asymmetry bias**: Network → Asymmetry 1000 ns: true offset steady-state median → −500 ns, estimated → 0.
* **Rate granularity**: Actuator → NXP, 24 MHz, Frequency error 5 ppm: the true offset dithers by µs.
* **Live tuning**: Mode → Live, Start, speed 20×, change kp while it runs; choose the integrator policy first.

## 6. Configuration file (JSON)

Everything is in `SimConfig` (`ptpsim/config.py`). Save from the GUI or write by hand; run with
`ptpsim-run run --config file.json`, open with `ptpsim-gui file.json`. Top level: `duration_s`, `seed`, `epoch_ns` (absolute
time base of all timestamps, default 1.7×10¹⁸ ns; must be > 0), and the groups `intervals`, `oscillator`, `network`, `latency`,
`tx_jitter`, `timestamps`, `loss`, `actuator`, `controller` (`name`, `params`), `firmware`.
`firmware` holds the constants of `clock.c`: `step_threshold_ns` (1 s), `lock_offset_ns` (10 ms), `lock_samples` (3),
`outlier_ns` (100 ms), `outlier_samples` (2), `delay_req_clear_ns` (3 s), plus the experimental `cmd_clamp_ppm` (0 = off,
not in the firmware).
A jitter object is `{ "kind": "none|uniform|normal|exponential", "scale_ns": x, "clip_sigma": 4 }`.

Command line: `ptpsim-run run [--config f | --preset default|noisy] [--set key=value …] --out dir`;
`ptpsim-run compare --out results` (baseline vs variant); `ptpsim-gui [file.json] [--lang en|it]`.

## 7. Glossary

**GM** Grandmaster (reference). **PHC** PTP hardware clock of the slave. **ppb/ppm** parts per billion/million (1 ppm = 1000 ns/s).
**Servo** the control loop (PI + lock/outlier/step logic). **Step** forced phase alignment. **Overshoot / settling** see §4.
**Epoch** the absolute time base (~1.7×10¹⁸ ns) of the timestamps; the simulator never stores it in a float.
