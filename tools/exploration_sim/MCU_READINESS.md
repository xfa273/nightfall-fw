# MCU integration and benchmark inputs

This audit covers the simulator branch at `c4c910b` and the normal checkout at
`3910fa6`, read without changing that checkout. It describes integration seams;
it does not enable motion or install a policy in production firmware.

## Direction-selection seam

In `platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_search_step.c`:

| Responsibility | Simulator-branch source | Normal checkout `3910fa6` |
|---|---|---|
| Current wall snapshot → map | `f413_search_step_read_and_write_current_wall`, line 1561; called after movement at 2701 | same function at 2552; call at 4015 |
| Adachi direction | `f413_search_step_choose_next_relative`, line 940; run-loop call at 2593 | function at 1303; call at 3862 |
| Known-straight acceleration lookahead | `f413_search_step_forward_is_known_straight`, line 1030 | same helper, selector call at 1413 |
| Stop before phase restart | No equivalent phase restart | `f413_search_step_run_phase_restart_entry`, line 3235; called at 4048 |
| Guarded persistent map save | End-of-run save | `f413_search_step_try_save_map_safely`, line 1267 |

An opt-in policy must replace both current direction and its lookahead. Changing
only `next_rel` leaves `known_straight` and `next_is_turn90` derived from an Adachi
route the robot may no longer follow. Keep existing GOAL entry behavior until a
post-goal policy is deliberately selected. Before dispatch, independently check
bounds, current map epoch, and that the next edge is known open. Snapshot buffers
must stay immutable during a resumable solve. Additive observations increment
the map generation; resets, corrections and profile changes increment its epoch.
An older generation's result needs dependency and action revalidation against
the current observations. An older epoch's result is invalid.

The normal checkout has an additional 1485 inserted / 111 removed lines in the
search module/header relative to this branch, including front alignment,
wall-end telemetry, post-goal map-saving guards and stop/back-up/restart motion.
Port an integration seam by function responsibility rather than copying the old
run loop over these changes.

## A cell decision is currently made while moving

`f413_search_step_drive_segment` sets a velocity profile and returns after a
distance threshold. It leaves the requested exit velocity active. The next wall
snapshot and direction choice therefore do **not** imply the robot is stationary.
TIM5 continues running `f413_ctrl_tick()` at 1 kHz from the interrupt callback in
`main.c`; a long foreground planner also delays the foreground switch, wall,
encoder and IMU guard checks in `f413_run_session_guard_check()`.

Use initial motor-off benchmarking first. A future stationary replan needs an
explicit controlled stop with a defined geometric endpoint and a corresponding
restart; calling the existing half-cell `run_final_stop` shifts physical position
and cannot simply be inserted before a centre-based decision. For background
planning, bound each work slice, retain a valid cached action, and take the
existing guarded stop path before a new action is needed if no valid result is
ready. Do not perform the planner in TIM5, mask interrupts around it, or claim the
1 ms control period is its whole foreground budget.

`f413_ctrl_is_running()` exists. A motor-off benchmark should verify it is false
and verify motor standby before timing. This is separate from production motion
integration and does not need to write map, calibration, identity or trace NVM.

## Map representation

The simulator and MCU policy interface use `1 << direction`, where directions
are N/E/S/W = 0/1/2/3. Firmware `map[][]` instead stores N/E/S/W = 8/4/2/1 in
each nibble:

- Low nibble: observed walls. Unknown and known-open are both zero here.
- High nibble: observed walls **or unknown edges**.
- An edge is known exactly when its high and low bits agree.
- `visited[][]` is independent: knowing all four boundaries does not prove the
  robot visited that cell.

Thus firmware-to-canonical walls reverse the four low bits, while known bits
reverse `~(high ^ low) & 15`. `firmware_map()` in the exporter tests the inverse
conversion, including all four single-wall directions.

## Reproducible partial-map corpus

Use saved simulator replay JSON rather than hand-written almost-open maps:

```sh
python3 tools/exploration_sim/mcu_benchmark_cases.py \
  build/exploration_sim/classic2014-v2.json \
  --json build/exploration_sim/mcu-classic2014-cases.json \
  --header build/exploration_sim/mcu-classic2014-cases.h
python3 tools/exploration_sim/mcu_benchmark_cases.py \
  build/exploration_sim/half2023.json \
  --json build/exploration_sim/mcu-half2023-cases.json \
  --header build/exploration_sim/mcu-half2023-cases.h
```

The two headers are separate so a 16×16 SRAM benchmark need not include 32×32
inputs. Stages are initial observation, first goal, middle of the still-uncertain
post-goal period, first certificate, and full truth. Missing milestones are
reported, never fabricated. The `full_truth` case is evaluation-only; its visited
array still represents actual visits. It must never be supplied to a live policy.

Replay deltas rebuild every ordinary case. Each includes pose, map hash,
provenance, known/wall/visited/goal arrays, optimistic and conservative projections
and the equivalent firmware map. A C header contains const arrays and typed
fixture descriptors; it is not automatically added to any firmware build.
Define `NF_BENCH_CASE_INDEX=0` through `4` when compiling one fixture to keep
the remaining const arrays out of the SRAM image. An invalid index is a compile
error. This matters for the 32×32 inputs, which otherwise add five maps per case.

`--mcu-expected` also exports exact status, costs, requirement/input checksums,
queue counters and source hashes from the compact C kernel on the host:

```sh
python3 tools/exploration_sim/mcu_benchmark_cases.py \
  build/exploration_sim/half2023.json \
  --mcu-expected build/exploration_sim/mcu-half2023-bench-expected.json
```

These expected rows deliberately use F413 mode 2 case 8 for both maze sizes.
A classic competition maze is only a 16×16 map fixture in this test, not a claim
that the compact oracle models a classic machine. Regenerate expectations after
kernel changes; queue counter changes can occur without changing exact costs.

## Compact timing tables

Run `tools/solver_host/generate_mcu_slalom_tables.sh` to regenerate the fixed
F413 mode 2 case 8 tables from the existing generic planner and firmware profile.
The generator checks the geometric requirements and goal-crossing order across
all three speed variants. Stop-crossing times use a triangular index and a
dictionary of exact integer microseconds, without approximate interpolation.
The generated table object occupies 49,944 bytes of read-only data, including
its input SHA256 fingerprint. `generate_mcu_slalom_tables.sh --check` regenerates
to a temporary file and fails if the checked-in output is stale. The fingerprint
covers compiler-discovered local source/header dependencies, including profile,
geometry and motion models, and both generator files. Profile changes therefore
require regeneration even when their effect on the generated numbers is zero.
The oracle
API, kernel memory layout and exactness constraints are documented separately in
[`tools/solver_host/MCU_SLALOM_ORACLE.md`](../solver_host/MCU_SLALOM_ORACLE.md).

`tools/hil/exploration_bench/planner_bench.c` runs each selected fixture with
optimistic and conservative projections. Its mailbox base case ID is
`(fixture_index << 1) | projection` (0 optimistic, 1 conservative). The base row
records solve cycles, maximum `step(1)` call cycles, call count, costs and required
edge checksum. A row with bit 16 set measures `begin` separately and includes
the input checksum. A row with bit 17 set contains counters: `slices` is actual
work units, `required_edges` is relaxed edges, and `checksum` is heap peak.
Each per-call 32-bit DWT difference is accumulated into a 64-bit total. A status
with bit 31 set means the benchmark slice cap was reached while pending; bit 30
means an exact result failed to provide complete requirements. The benchmark
never converts a pending result into an exact answer.

## Timing evidence to collect

For every MCU case/projection, record CPU/clock and compiler configuration,
fixture hash, planner status, elapsed DWT cycles, wall time, number of work slices,
maximum slice cycles, interrupt/control deadline statistics, abort latency,
workspace bytes and stack high-water mark. At 100 MHz, 100,000 cycles are 1 ms;
check counter-wrap handling if a call approaches 42.9 s. Measure cold and retained
planner states separately. Report worst observed sample and sample count; a small
sample is not a worst-case execution-time proof.

Observe-trigger cost, map conversion, direction selection, optimistic solve and
known-route solve should be separate metrics. Also measure end-to-end decision
latency. Keep UART serialization outside the timed interval. The nominal cell
transit included in fixtures is only a useful scale, not an approved deadline.

An optional host measurement path bypasses the Python result cache and reports
thread CPU plus wall times, including ctypes/JSON overhead but excluding the
shared-library build:

```sh
python3 tools/exploration_sim/mcu_benchmark_cases.py \
  build/exploration_sim/classic2014-v2.json \
  --host-benchmark build/exploration_sim/mcu-classic2014-host-timing.json \
  --repeats 3
```

On the current macOS arm64 host, the original orthogonal oracle's optimistic
CPU-time medians were 105.68 ms initial, 77.46 ms first goal, 62.78 ms uncertain
middle, 59.68 ms certificate and 56.57 ms full truth (three samples each). These
are evidence about the host implementation only. They do not predict the
integer MCU implementation, Cortex-M4 timing, or whether motion deadlines pass.

## Actual C policy replay

The host bridge in `tools/solver_host/mcu_exploration_bridge.c` links the same
portable C policy and compact C oracle as firmware. Python supplies only the
current `Knowledge` arrays; truth remains inside observation and collision
checks. Policy work counts actual `step(1)` units in C, and oracle work counts
the kernel's own units. Final certificates are checked independently with fresh
optimistic, conservative and truth solves. Those three audit solves are excluded
from policy/oracle workload totals.

```sh
python3 tools/exploration_sim/mcu_policy.py 32MM2023HX \
  --output build/exploration_sim/mcu-policy-half2023.json
python3 tools/exploration_sim/mcu_policy.py 16MM2014CX \
  --output build/exploration_sim/mcu-policy-classic2014.json
python3 tools/exploration_sim/mcu_policy.py 32MM2023HX --predictive \
  --policy-units-per-ms 256 --oracle-units-per-ms 20 \
  --output build/exploration_sim/mcu-policy-half2023-budget20.json
python3 -m unittest tools.exploration_sim.test_mcu_policy
```

The default machine in these experiments is **mini_r2, including on the classic
competition's 16×16 layout**. Every comparison policy uses the same fixed F413
mode 2 case 8 shortest objective. This is deliberately distinct from the UI's
classic-machine experiment. C navigation uses 5 ms score quantization and an
acceleration guard, so exact route equality with Python is a checked result for
these examples, not an assumption for all mazes.

In the earlier synchronous-drain reference, the 2023 half layout baseline took
1338 simulated moves / 454.880 s;
Python relevant and C drain each took 1073 / 325.959 s, with identical route
order and an independently checked 18.540092 s shortest certificate. C drain
performed 88 oracle solves, 489,981,862 oracle units and 11,485,386 policy units.
On the 2014 classic layout, the corresponding baseline was 434 / 167.058 s and
both relevant variants were 251 / 92.010 s, with a 13.223795 s certificate.
These durations include the kinematic movement estimate only. Draining the
oracle synchronously is an algorithm reference, not a viable moving-firmware
schedule.

This certificate applies only to the pinned compact mini_r2 mode 2 case 8 graph.
It does **not** certify a different or newer machine graph, including one with
wall-dependent primitive enablement that breaks optimistic-map monotonicity.
The fresh projection checks below validate these experiment instances; they do
not authorize substituting another graph or runtime profile.

Predictive mode plans from the expected next pose while the robot traverses the
current cell. On arrival, the real C `apply_result` rejects an unobserved or newly
blocked first edge, an epoch mismatch or an unavailable decision. The simulator
then takes the observed-open Adachi fallback. A prediction never marks a goal
visited. The portable acceleration guard postpones a turn/completion if the
previous accelerated move promised another forward cell.

After a rejected or missed prediction, the shared C progress guard holds
Adachi until actual known/wall/visited observations change. A validated exact
certificate still passes through. This prevents cancellation at every short
straight from alternating Adachi and relevant turns indefinitely. The host and
production adapter both call this guard after arrival validation and before
constructing the fallback and checking acceleration.

The scheduler exposes independent policy/oracle work ceilings per nominal
travel millisecond. These are **not** measured MCU microseconds: they exclude
initialization/copy cost, interrupt load and integration overhead, and their two
CPU shares must fit together in a real implementation. A nominal straight
window uses cell length divided by the known-section speed cap; it never uses
future truth to enlarge the available window. The oracle receives its remaining
allowance independently even if navigation is unfinished or a surrogate proposal
is already ready. No stationary waiting duration is fabricated for an unfinished
experiment.

## Primary bounded-work results: 64 policy units/ms

The primary scenarios use 64 policy units per nominal travel ms and either
20 or 50 oracle units/ms. Motor-off HIL averages provide scale only: the pruned
policy's 185,056-unit sample took 403.661 ms (about 2.18 us/unit); the cached
oracle's first-goal sample took 11.118 s for 1,556,515 units (about 7.14 us/unit),
at 100 MHz. Thus 64 policy units/ms is approximately 14% CPU and 20/50 oracle
units/ms approximately 14%/36% CPU **at those averages**. These omit adapters,
initialization, memory copies, interrupts, sensor work and variation between
states. They are scenarios, not measured foreground reservations or real-time
validation. The earlier 256-policy-unit allowance was aggressive and is only a
secondary historical experiment.

All twelve final runs below reached a certificate before the 6000-step cap.
Every certificate independently matched fresh optimistic, conservative and
truth solves: 24.679023 s (2019), 34.733206 s (2022), 18.540092 s (2023).
Reported durations are movement estimates, not measured robot runs.

| Half final maze | Baseline moves / s | Oracle units/ms | Exact + Adachi fallback, moves / s | Pending surrogate ON, moves / s |
|---|---:|---:|---:|---:|
| 2019 | 1525 / 517.181 | 20 | 1346 / 468.633 | 1428 / 511.737 |
| 2019 | 1525 / 517.181 | 50 | 1439 / 517.242 | 1400 / 491.719 |
| 2022 | 1491 / 434.250 | 20 | 931 / 275.499 | 1054 / 312.308 |
| 2022 | 1491 / 434.250 | 50 | 836 / 248.281 | 884 / 265.627 |
| 2023 | 1338 / 454.880 | 20 | 1185 / 407.452 | 1310 / 466.338 |
| 2023 | 1338 / 454.880 | 50 | 1153 / 403.214 | 1258 / 445.276 |

Use surrogate OFF as the default finite-work configuration. The optional cheap
cardinal-route probe was not uniformly better: it was slower than OFF in five
of six scenarios, and slower than baseline in 2023 at 20 units/ms. Even OFF
was 0.061 s slower than baseline in 2019 at 50 units/ms. More oracle throughput
changes when directions are selected and does not guarantee a shorter search.

The initial 64-policy/50-oracle hybrid run exposed a real liveness issue:
after step 1086 no new wall facts appeared, while short-window Adachi moves and
long-window relevant moves reversed each other. It hit 6000 steps without a
certificate. The failing replay is preserved as
`build/exploration_sim/mcu-policy-32MM2023HX-policy64-oracle50-hybrid1.json`.
The shared progress guard resolved that case at 1258 steps; a regression test
runs the entire scenario when the pinned dataset is locally cached. The fixed
results in the table include this guard and all its performance cost.

Regenerate the compact report and complete traces in ignored build storage:

```sh
python3 tools/exploration_sim/mcu_policy.py \
  --grid 32MM2019HX 32MM2022HX 32MM2023HX \
  --policy-units-per-ms 64 --oracle-rates 20 50 --max-steps 6000 \
  --trace-directory build/exploration_sim/policy64-progress \
  --output build/exploration_sim/mcu-policy-budget-grid-policy64-progress.json
```

The report includes all twelve cases, map provenance, profile/config values,
oracle/policy work totals, source hashes and the three fresh certificate checks.
Full traces additionally include each observation, ungated/applied decision,
progress-recovery count and per-window work usage. The recorded report SHA256 is
`576d29cd6876593fa9f36866e3a06a80b4ece6b7b6129e63431712b5975b75c1`.
Its explicitly overridden surrogate OFF/ON cases used these C source hashes:

- Policy: `447bf07d999c542795799f0a4aa9defaefadee75c590fde27625efa1477ea07b`.
- Compact oracle: `97feabdaa9115102f737f12465c173c0000807b7c17816b70e6b62230edce988`.
- Tables: `69b66f565cd253b80d7e6beb44a826179cd43f1631a8dd3b74cb850f1c3cdc41`.

The report also includes the matching headers. Compiler/host timing and source
changes can change artifact hashes; compare outcomes and work counts alongside
the hashes rather than treating a hash alone as a performance test.
