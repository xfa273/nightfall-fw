# mini_r3 mode4 suction tuning seed (2026-09-21)

The user reports that fan-off exploration and mode2 shortest running work on
mini_r3. Mode4 now starts the suction tuning phase at 50% duty (`500/1000`),
with mini_r2 mode4's configured turn speeds. Tune version:
`mini-r3-mode4-fan50-t0.15`. This is an initial case0 tuning profile; no suction
motion, firmware flash, or NVM operation was performed for this change.

## Simulator and selected parameters

Used [PR #21](https://github.com/xfa273/nightfall-fw/pull/21), exact simulator
commit `ab377c8b937db2fdbc515d2d55970becd16866be`, from its existing worktree.
`turn_simulator.simulation()` with no empirical artifact supplies the ideal
response, 1 kHz discretization, rounding scale 1.2 and omega cap 2200 deg/s.
The historical fitted model covers mini_r2 right135-in at500 mm/s; it is not
used as an r3 suction model. Speed and exact accumulated angle stay fixed.
The firmware alpha field is the profile coefficient, not the peak physical
angular acceleration (the unsaturated cosine derivative is about1.309×alpha).

| case0 sub | Turn | Speed mm/s | alpha deg/s² | Entry mm | Exit mm | Ideal endpoint error mm | Raw body gap mm |
|---|---|---:|---:|---:|---:|---:|---:|
| 0 | small90 | 800 | 50000 | 0.6 | 1.0 | 0.580 | 11.497 |
| 1 | large90 | 1000 | 20500 | 3.6 | 3.7 | 0.448 | 16.033 |
| 2 | large180 | 1000 | 18000 | 1.0 | 1.5 | 0.951 | 14.480 |
| 3 | 45-in | 1200 | 47250 | 2.1 | 20.8 | 0.981 | 7.894 |
| 4 | 45-out | 1200 | 47250 | 20.8 | 2.2 | 0.988 | 8.053 |
| 5 | V90 | 1200 | 115250 | 17.5 | 18.5 | 0.863 | 7.588 |
| 6 | 135-in | 1200 | 47500 | 16.6 | 9.5 | 0.176 | 7.801 |
| 7 | 135-out | 1200 | 50250 | 12.1 | 20.1 | 0.931 | 7.875 |

Selection is a geometric initial design, explicitly requested for physical
case0 adjustment. The simulator's built-in uncalibrated `tune()` bounds alpha
at40000, which cannot close several800/1200 mm/s turns. Instead, sweep the
same PR's simulation model at250 deg/s² intervals within these declared
alpha bounds (in table order): 50000..110000,18000..41000,14000..21000,
40000..100000,40000..85000,60000..150000,35000..75000,35000..75000.
For each core, solve positive entry/exit distances for `canonical_target(code)`;
quantize to0.1 mm and retain the PR simulation's endpoint error <=1 mm.
For180 degrees, require the core lateral error <=1 mm and scan entry1..39 mm
at2 mm intervals, solving exit from the forward error. Offsets must be
0.5..60 mm. Rank using the existing `turn_clearance` canonical wall/post scene
and70×39 mm body (same r3 envelope recorded in `MINI_R3_COMMISSIONING.md`).
Choose the lowest alpha within0.25 mm of the best geometric gap, breaking ties
by endpoint error. No motor/traction limits are inferred from this search.

Rechecked the actual C initializers through PR #21's `simulation()` for all
16 left/right primitives, plus swept clearance with0.5 mm corner interpolation:
endpoint <=0.988 mm, heading error <0.1 degree, no nominal body intersection.
Subtracting5 mm position/model uncertainty plus mechanical/heading/interpolation
allowances leaves diagonal effective gaps1.61..2.08 mm, below the3 mm target.
Thus these are **unvalidated tuning seeds**, not qualified maze-clearance
recommendations; `safe_recommendation_available` remains false. V90 reaches the
2200 deg/s omega cap. Suction tracking, saturation and real wall clearance need
case0 trace/video measurements before using the new diagonal settings in a maze.
Local calculation artifacts are in `build/mini_r3_mode4/` (not versioned).

## case0 operation

Select mode4 → case0 → sub0..9 using the normal OP UI. After optical START,
IMU calibration completes with the fan off. Control then holds the original
zero distance and zero angle for20 ms. Fan PWM starts at1/1000 and increases
at a fixed rate:50% in600 ms,100% in1200 ms. At the configured50% target,
it then stays at50% for300 ms before departure.
The ramp uses elapsed milliseconds and updates the running PWM compare every
10 ms guarded wait; it does not stop/restart the PWM between updates.
Control stays active into the first straight without recalibration or re-zeroing
its angle. There is no additional pose-settling timeout before departure.
The fan remains on through deceleration and turns off before optical STOP/log tail.
Stop-switch and sensor/encoder/IMU guard aborts, timeout and PWM-start failure
all lead through cleanup; failed preflight never starts the fan or motors.

| sub | Actual path (half-cell straight / diagonal steps) |
|---|---|
| 0 | S3, small R90, S3 |
| 1 | S4, large R90, S3 |
| 2 | S4, large R180, S3 |
| 3 | S4, R45-in, DS3 |
| 4 | S4, R45-in, DS3, L45-out, S3 |
| 5 | S4, R45-in, DS3, L-V90, DS3 |
| 6 | S4, R135-in, DS3 |
| 7 | S4, R135-in, DS3, L135-out, S3 |
| 8 | S9 with case1 straight settings |
| 9 | S9 with case5 straight settings |

Every path also has the existing10 mm first section and45 mm stop tail.
Extra straights permit braking: a1200 mm/s diagonal test decelerates to the
case's stoppable goal-entry speed before the45 mm tail. Exact accumulated
angles are enabled, front-wall/wall-end correction disabled, and existing
wall-control behavior retained. The case arrays remain the mini_r2 baseline.
The straight cap remains1500 mm/s; only suction mode4's turn/diagonal caps
become1200 mm/s. This change does not enable an r3 KERI precomputed table.

F413 currently selects the already tuned `FAN_OFF` gains regardless of fan
state. This behavior is deliberately retained for the initial suction tune;
legacy untested `FAN_ON` gains are not activated. No control-loop code changed.
Search/mode2, other modes, r2 parameters and its case0 mapping, F405, LUTs,
identity/calibration/maze/log schemas, `.ioc` and stable snapshots are unchanged.
Mode4's new small90 entry moves its front-distance target to89.4 mm, within the
current40..110 mm LUT; case0 does not use this front correction.

## Verification

- ASan/UBSan actual mode4 dispatch + production path preflight: all10 subs;
  800/1000/1200 mm/s turn selection and unchanged other-mode caps.
- ASan/UBSan production session with mocked hardware: normal finish,
  calibration/control before fan,20 ms control lead, monotonic ramp at25/50/75/100%
  taking300/600/900/1200 ms respectively,300 ms wait after reaching50%,
  elapsed-time ramp under delayed waits and SysTick wrap, continuous control,
  latest-log pose observation regression (no extra startup timeout), failed
  fan start/duty update, every guard abort before fan/during ramp/stabilization/
  driving, stuck-distance timeout, pressed switch, invalid path and busy-trace
  refusal; fan-off mode2 retains departure immediately after control start.
- ASan/UBSan production1 kHz control tick: stationary output zero, both yaw
  directions and both fore/aft displacements produce opposing motor outputs;
  initial angle remains referenced to its pre-fan origin into the first straight.
- Production fan PWM helper with mocked HAL:50% compare, uninterrupted PWM
  across500 duty updates, idle/capability/stop/range refusal, PWM failure cleanup.
- Runtime machine/identity/profile and front-distance reference tests,
  existing225 path-linear checks,14210 route-table numeric checks, F413/F405
  Debug builds, diff check. The mode2 route table values are unchanged; only
  input SHA metadata is refreshed for the shared runner source. The local
  metadata also accounts for the pre-existing uncommitted r2 case3/4 edits;
  committed metadata excludes those unrelated edits.
- No agent-run live HIL; the new ramp and physical startup heading remain unverified.

## Fan startup heading fix (t0.13 history; settle gate removed in t0.14)

The user observed a small chassis rotation when suction started. Latest local
trace: `tools/logging/logs/trace_bin_20260921_215000.csv`, mode4/case0/sub2
(large R180), test80,2762 records, SHA256
`58156d3bc31fd15ea2c342dae30a0461de66bb14da696036d46359b25d298635`.
No firmware SHA is recorded in this decoded CSV; live firmware identity was not
queried. Relative to the first record, gyro/angle/velocity/right PWM first
become nonzero at1321 ms and commanded velocity at1324 ms. VBAT ADC drops from
2031 at0 ms to1814 at40 ms, consistent with fan loading before wheel control.
The trace does not contain a fan-duty channel, so that drop is supporting
evidence rather than an exact fan-start timestamp.

In t0.12, the path runner started the fan, waited100 ms, then called
`f413_ctrl_start()`. That call first performs blocking IMU stationary-offset
calibration (200 ms settle plus500 samples/SPI/delays), resets the angle origin,
and only then enables wheel control. Consequently the fan's startup impulse
occurred with wheel control off, and motion during calibration could contaminate
the offset. `f413_ctrl_tick()` does not sample motion while stopped: the zero
angle samples in this interval are stale values, not proof of physical stillness.
The actual startup yaw cannot be recovered from this log.

Starting control before the fan removes that uncontrolled interval. It uses the
existing controller's distance hold from `start()` and an explicit angle-zero
hold; calling `set_velocity(0)` here would disable position feedback. Powered
holding is marked with the existing MOTOR_COAST trace flag so automatic NVM
flushes cannot occupy shared SPI2 during IMU control. No trace schema changes.
The same continuous control records fan-start yaw rather than hiding it with a
second reset. Suction duty, gains, turn speed/alpha/offset and fan-off sequencing
are unchanged. The timing fix applies to any F413 path mode with nonzero fan
power; currently the r3 mode4 profile is the configured user of this path.

Validation is host-only: the controller test verifies correction direction,
not the real traction or disturbance magnitude. No live command, flash, motor,
fan or NVM operation was performed. The next recorded trial below exposed the
settling failure that those direction-only host tests did not establish.

## Startup timeout correction (t0.14 history; ramp/wait extended in t0.15)

The user then reported that the fan spun but the run ended before moving.
Latest local trace `tools/logging/logs/trace_bin_20260921_220753.csv` has2777
records, mode4/case0/sub2/test80, SHA256
`1fc9ddc358bdec59a14df974b1dc36828ca04beaae68fc2cb29f9de1fa49c8e7`.
No firmware SHA is embedded in the decoded CSV and no live ID was queried.
The following times are relative to its first record:

- 1205 ms: angle-target flag/control observations become active; VBAT drops after
  approximately1225 ms, consistent with the20 ms control lead before fan start.
- 1325..2324 ms: commanded distance remains0, no forward path phase, but actual
  angle ranges-1.717..+2.664 degrees and velocity samples range-18..+25 mm/s.
- 2325 ms: angle hold is released into cleanup, exactly1120 ms after control
  start (20 ms lead +100 ms fan wait +1000 ms failed settling).
- 2386 ms: `flags=0x9480` includes the shared timeout/encoder-class abort bit;
  no switch/wall/IMU fault bit. The phase timing and unmet angle gate identify
  the added startup timeout. The trace bit alone cannot distinguish an encoder
  fault from timeout.

At the nominal zero-angle crossing (1450 ms), target omega is still-26.6 deg/s;
at1700 ms the angle is-1.714 degrees and target omega+18.6 deg/s. The existing
cascaded integral control is responding to the disturbance and reversing;
this is not evidence that stationary holding has settled. The t0.13 hard gate
assumed a stationary precision that had not been demonstrated on the robot and
blocked departure. Host direction-of-correction tests did not cover that
physical behavior.

Remove that added gate, retaining normal guarded startup and the pre-fan
reference. Replace the abrupt50% duty step with the300 ms ramp above to spread
the reaction impulse; after reaching50%, keep the existing100 ms stabilization
wait before advancing. This is a command-sequence correction, not a claim that
physical yaw is now within0.5 degree. Gains, angle reference, turn geometry,
non-suction mode sequencing and F405 remain unchanged. Physical heading during
the ramp and into the first straight still requires a new trace/video.

The session regression injects the observed timeout pose (+2.655 degrees,
-1 deg/s, approximately0.666 mm, -3 mm/s) to verify that it cannot recreate an
extra startup timeout. It does not simulate the physical response to the ramp.
All startup phases retain switch/sensor/IMU/encoder guards and failure cleanup.
F413/F405 builds, production PWM/session/control host tests, all10 mode4 paths,
225 path-linear and14210 route numeric checks pass. No UART, flash, reset,
fan/motor command or NVM operation was performed by the agent.

## Slower fan startup (t0.15)

The user reports substantial improvement with t0.14 but some remaining
instability, and requests half the ramp rate plus a300 ms post-target wait.
Use `ceil(target_per_mille *1200 /1000)` milliseconds for the ramp duration:
50% takes600 ms and100% takes1200 ms, rather than assigning one duration to
every target. Keep the elapsed-time interpolation and10 ms guarded PWM updates.
Set mini_r3's runtime `SUCTION_FAN_STABILIZE_DELAY_MS` to300 ms; its currently
configured target remains50%. Thus departure occurs about900 ms after fan
startup (600+300 ms), with continuous pose feedback and the original heading.
The startup message reports the actual ramp duration and post-target wait.

The existing host session tests check25/50/75/100% endpoint times, the300 ms
post-target wait, delayed polls, timer wrap, all-phase aborts and cleanup.
Runtime machine/profile and route-table tests and F413/F405 builds pass.
No live hardware command or firmware flash was performed; the user's requested
slower startup remains to be evaluated physically.
