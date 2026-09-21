# F413 search stop approach, 2026-09-21

## Current: t0.11 near-wall recovery instead of proximity abort

The user's 20:02 event log (`trace_bin_20260921_200223.csv`) again selects
back at (1,0,S), smap9, but ends with wall_fault after138 ms instead of the
previous5000 ms timeout. At abort the recorded speed is168 mm/s, FR/FL2862/2696
convert to41.75/43.18 mm, and neither front-match result is present. This is
consistent with the t0.10 proximity abort; the binary event dump has no FW SHA.
The user explicitly requests recovery even when the robot is too close and
bounded shutdown only when correction cannot complete (including contact
with a miscalibrated wall reading).

Changes in `mini-r3-2s-fanoff-t0.11`:

- Remove the42.5 mm hard abort from search stopping. Before an alignment-enabled
  back turn, either front channel at/inside the45 mm target, valid front-sum
  at target, or front ADC saturation latches a handoff. Cancel the encoder
  target and wait for speed<=10 mm/s for20 observations; near extrapolation or
  loss of range-validity during braking no longer cancels the handoff.
- Front alignment accepts finite, strong front signals without requiring the
  LUT range-valid bit. Either channel closer than target minus the existing
  position tolerance (r3:43.5 mm), or front saturation, triggers **retreat**, not
  failure. This is a recovery trigger, not a replacement exclusion limit.
- Retreat takes priority over yaw alignment: command-30 mm/s, omega0, discard
  the old distance profile, clear accumulated velocity feedback, and reset the
  optical filter/controller after recovery. Preserve IMU and odometry state.
  Require both channels recovered for20 observations plus at least0.5 mm of
  signed encoder retreat before returning to normal alignment. Maximum absolute
  displacement per retreat is12 mm; exceeding it disables drive and aborts.
- The complete alignment attempt, including retreat and any retries, has a
  wall-clock1000 ms deadline. Timeout now disables drive and propagates abort
  to prevent a subsequent spin; previously the ordinary alignment timeout
  could return success. The continuous alignment test also limits each active
  acquisition to1000 ms (holding or motor-off wall-lost waiting is not pushing).
  The preceding encoder/braking approach retains its5000 ms timeout.
- Missing/nonfinite samples, zero/stale sensor sequence20 ms, stop switch,
  IMU/encoder faults remain stop conditions. No proximity-only abort remains.
  No synchronous diagnostic output is emitted at braking/retreat entry or
  retreat completion; deadline/fault shutdown precedes diagnostic output.
- F413 mini_r2 also uses this shared policy with its own target7 mm/tolerances.
  The legacy `F_ALIGN_TOO_CLOSE_MM` profile field remains for compatibility but
  is not used by F413 search/alignment. F405 sources, gains, LUTs, offsets,
  IMU policy, geometry, map rules, NVM/log schemas and stable checkpoint unchanged.

Verification (host only): production wait and alignment loops plus actual
front-match state machine test38/41.75/42.49/43.4/45/50 mm starts, saturation,
the latest asymmetric41.75/43.18 mm case, retreat then completion, stalled
forward/reverse correction,12 mm travel limit, continuous-test timeout,
stale/missing/nonfinite data, switch abort, and handoff validity loss under
ASan/UBSan. The real control tick confirms clearing prior positive integration
allows negative motor output while preserving odometry. Real-LUT replay of all
three failed ADC pairs now requests handoff rather than wall_fault. Machine
profile tests, NVM guard,225 path-linear checks, solver internal sample, F413
and both F405 builds pass.

No hardware command/flash/reset, motor/fan, maze run or NVM write performed.
Physical retreat/braking, calibration error, and successful180-degree maze
turn remain unverified; host models cannot guarantee mechanical clearance or
escape from a jam. Next deployment check is a supervised low-speed repeat.

## Historical t0.10: evidence and scope

User logs `trace_bin_20260921_193455.csv` and
`trace_bin_20260921_193623.csv` both end at (1,0), heading south, during
`mode1 case2` full exploration. The planner still has a reachable target
(`smap=9`) and chooses a back turn. The motion times out at exactly 5000 ms;
`completed=0`, `route_failed=0`, `abort=timeout`, with neither front-match
result present. No full-complete/unreachable phase is logged.

The back turn first performs a 45 mm encoder-based deceleration. Only after
that succeeds does it save the map, align to the front wall, and spin.
At timeout, target/encoder distances are 3860.384/3854 mm and
3709.473/3706 mm (encoder distance in these records is rounded). Front ADC
is FR/FL=2901/3174 and 3086/3021, approximately 40 mm on the current LUT.
Those readings are not a measurement of physical contact, and the event
log does not contain continuous approach samples or a firmware SHA.

## Historical t0.10 changes (proximity policy superseded above)

- The F413 velocity profile clears acceleration at the target velocity.
  Previously it retained negative acceleration after the reference reached
  zero, including negative acceleration/static feedforward.
- Preserve the original acceleration direction separately for nonzero-speed
  feedback clamps. Clearing acceleration must not accidentally reverse the
  clamp or suppress positive position correction at a zero-speed endpoint.
- A finished zero-speed profile holds the exact requested distance endpoint,
  rather than the slightly short discrete integral. Position correction is
  bounded to +/-30 mm/s; its feedforward uses the correction velocity and
  zero acceleration. Starting/cancelling profiles is atomic against TIM5.
- Search zero-speed segments require the profile to finish and position error
  within 1 mm, with speed within 10 mm/s for 20 consecutive 1 ms observations.
  A stationary 3--6 mm error is **not** silently accepted; the timeout remains.
- Specifically before an alignment-enabled back turn, a valid front-sum
  reading at or inside the alignment target (r3:45 mm, r2:7 mm) cancels the
  encoder approach and brakes. After settling at low speed, the existing
  front alignment is allowed to run. The old distance target is discarded.
- During all search zero-speed approaches, either front channel below the
  existing near threshold (r3:42.5 mm, r2:4.5 mm), front raw-ADC saturation,
  missing/nonfinite data, or 20 ms without a new sensor sequence aborts and
  disables drive. Near-side extrapolation can stop motion but cannot authorize
  a successful handoff. Loss of valid front data after handoff also aborts.
- Successful encoder stops cancel residual position drive before the next
  operation. Failure cannot proceed to map saving, alignment, or a spot spin.
- UART `[SEARCH-STOP]` reports handoff/completion/near fault/timeout without a
  trace-schema change. Existing binary motion-end/abort records remain valid.

These changes affect **shared F413 control and search**, including mini_r2;
they are not confined to unit002. F405 sources are unchanged. PID/FF gains,
IMU policy, LUTs, offsets, turn geometry, maze rules, NVM and stable checkpoint
are unchanged. mini_r3 tune label becomes `mini-r3-2s-fanoff-t0.10` to distinguish
the changed motion behavior; the accepted t0.8 checkpoint remains immutable.

## Historical t0.10 verification and deployment boundary

- `sh tools/hil/run_f413_stop_approach_tests.sh`: ASan/UBSan production search
  wait loop with mocked hardware, plus the actual control tick with HAL stubs.
  Covers 331.662->0 over45 mm, the two logged distance gaps, exact endpoint,
  correction/FF sign and limits, acceleration/deceleration to nonzero speed,
  stopped/high-speed/not-yet-finished states, wall handoff/loss, too-close and
  near-extrapolated values, saturation, missing/stale data, stall and switch.
- Machine tests exercise the actual LUT converter against both logged ADC
  pairs (stop fault) and the45 mm pair (handoff), along with all previous
  r2/r3 identity, range and interpolation checks.
- F413 and both F405 builds, NVM guard tests, 225 path-linear host checks,
  the solver host's internal open-maze exploration and diff checks pass.

No probe, UART, reset, flash, motor/fan, maze run, or NVM operation was executed.
The host follower is a regression model, **not** a qualification of physical
braking distance, wall calibration, friction or full-maze operation. Application
flashing and low-speed maze verification were pending. The subsequent20:02
run and explicit user recovery request supersede the near/saturation-abort
policy, as detailed in the t0.11 section above.
