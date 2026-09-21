# mini_r3 mode4 suction tuning seed (2026-09-21)

The user reports that fan-off exploration and mode2 shortest running work on
mini_r3. Mode4 now starts the suction tuning phase at 50% duty (`500/1000`),
with mini_r2 mode4's configured turn speeds. Tune version:
`mini-r3-mode4-fan50-t0.12`. This is an initial case0 tuning profile; no suction
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

Select mode4 → case0 → sub0..9 using the normal OP UI. The fan starts after the
optical START signal, stabilizes for the existing100 ms, then drive starts.
It remains on through deceleration and turns off before optical STOP/log tail.
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
  failed fan start, every guard abort during spinup and driving, stuck-distance
  timeout, pressed switch, invalid path and busy-trace refusal.
- Production fan PWM helper with mocked HAL:50% compare, capability/stop/range
  refusal, PWM failure cleanup.
- Runtime machine/identity/profile and front-distance reference tests,
  existing225 path-linear checks,14210 route-table numeric checks, F413/F405
  Debug builds, diff check. The mode2 route table values are unchanged; only
  input SHA metadata is refreshed for the shared runner source. The local
  metadata also accounts for the pre-existing uncommitted r2 case3/4 edits;
  committed metadata excludes those unrelated edits.
- No live HIL; physical suction turns and floor/maze runs remain untested.
