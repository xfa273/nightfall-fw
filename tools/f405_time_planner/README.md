# F405 classic orthogonal time planner

A fixed-memory, allocation-free 16x16 planner prepared on the `v1.0.0` classic
competition baseline. It emits only straight, small 90, large 90 and large 180
legacy path codes. Classic shortest runs now use it by default via
`solver_build_path()`. Mini retains the original solver. See
[the preparation and hardware return plan](../../docs/CLASSIC_R1_2026_PREPARATION.md).

## Run host checks

```sh
tools/f405_time_planner/run.sh
tools/solver_host/run_route_motion_tests.sh
tools/f405_time_planner/run_integration.sh
tools/f405_time_planner/run.sh tools/solver_host/testdata/16MM2014CX_kerilab.maze 2 4
tools/f405_time_planner/run.sh --matrix tools/solver_host/testdata/16MM2014CX_kerilab.maze
```

The default build uses AddressSanitizer and UndefinedBehaviorSanitizer. Set
`SANITIZE=0` for host timing. `CC` may select another C compiler. Host milliseconds
are not an F405 runtime benchmark. The CLI reads S/G markers in the maze; the
firmware adapter instead uses the compiled classic start and goal macros.

`--matrix` accepts multiple `.maze` files and checks all 54 classic mode/case
profiles. For cases 3--9, which use nominal angles, it compares the objective
against the existing general PC Dijkstra implementation. Cases 1/2 are checked
by independent legacy path replay; mode 2 uses corrected profile angles there,
which the general reference does not support. Modes 3..7 use nominal angles
through their normal OP entries. Tie paths may differ;
all successful objective comparisons agree to the microsecond. Only canonical
orthogonal path codes are accepted by the independent replay.

The nine historical mazes used on 2026-10-07 are `16MM2012CX.maze` through
`16MM2020CX.maze` from
[kerikun11/micromouse-maze-data](https://github.com/kerikun11/micromouse-maze-data/tree/762ed2b68735ea29148c6a1251a90ed0651ff26b/data),
revision `762ed2b68735ea29148c6a1251a90ed0651ff26b` (MIT). They were read from the
existing pinned cache in the original worktree; the 2014 fixture was already
tracked by Nightfall. Additional downloaded mazes and matrix output belong in
`build/`, not in a stable firmware manifest.

## ARM memory check

```sh
cmake -S . -B build/classic-preview -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DNIGHTFALL_F405_ORTHOGONAL_PREVIEW=ON
cmake --build build/classic-preview --target nightfall_classic_r1_0
```

`NIGHTFALL_CLASSIC_TIME_PLANNER` defaults to ON for classic.
`NIGHTFALL_F405_ORTHOGONAL_PREVIEW` remains an optional link-only switch (OFF).
When either is ON, the linker explicitly retains
`f405_orthogonal_preview`; memory figures therefore include the real planner
workspace and code. Mini's binary
remains unaffected. There is no 32x32 compact implementation in this branch.

The workspace is 36,876 bytes, with 3,073 maximum states and no heap allocation.
The 1,536-byte connector cache is on the foreground stack. The `.su` files under
the build tree expose static function frames; `check_memory.py` reserves at
least 8 KiB beyond the original linker reservations and protects the calibration
sector-9 boundary. The old control, logging and calibration code is retained, and the legacy
solver remains available in an OFF build. Unused legacy solver arrays are
discarded by the linker in an enabled build. Linking does not establish
hardware behavior.

## API and model contract

`nf_compact_orthogonal_plan` takes an explicit maze, run configuration, request,
caller-owned workspace and output. `f405_orthogonal_preview` is a foreground,
non-reentrant convenience wrapper using a private workspace and the unchanged
classic profile. It takes map bytes supplied by the caller and performs no
NVM read/write, peripheral access or motion. Do not pass the active runner's
path buffer while moving. Input/output/result regions must be disjoint.

Success means a **nominal-model plan** was derived and encoded. Costs use first
goal entry; output may continue along a known corridor to stop. Time equivalence
to the original F405 runner's per-code acceleration/wall-end/terminal logic is
not asserted. The production integration check runs the real `run_shortest`,
`solver_build_path`, and `run` with drive/HAL primitives stubbed. It checks
load/conversion of the 16-bit map, canonical dispatch, zero-speed termination,
no diagonal calls and no motor/fan start after planning failure. It does not
simulate controller dynamics or sensor correction. New-route hardware timing,
clearance and floor running still require validation.

Unknown and contradictory walls are closed by the F405 adapter. An already
satisfied goal yields an empty path; the caller must not start a run. Failures
leave output and result untouched, so callers must check status and never run
an old buffer after a failed plan. No-path is not automatically downgraded to
a different speed, motion set or legacy solver.

## Imported reference sources

The following existing Nightfall files were copied byte-for-byte from
`2677cae38e64d70d8863b1d21adb7ef9611c52b2`:

- `common/route/motion_time.{c,h}` (shared nominal motion calculations)
- `common/route/orthogonal_time_planner.{c,h}` (general host-only reference;
  its large route object and dynamic allocations never enter the ARM build)
- `tools/solver_host/maze_ascii.{c,h}`, `route_motion_tests.c`, its test script,
  and the three maze fixtures

Only `motion_time.c` enters the optional MCU target. The compact implementation
uses separate packed storage, queue, parent reconstruction and validation.
The common motion arithmetic is deliberately shared, so differential tests
establish search/encoding agreement, not independent physical model validation.

## Use on the retuning branch

```sh
cd /Users/xfa273/workspace/micromouse/nightfall-fw-classic-baseline
cmake --preset Release -DNIGHTFALL_CLASSIC_TIME_PLANNER=ON
cmake --build --preset Release --target nightfall_classic_r1_0
```

After a successful build, use the existing USB-UART procedure. The integrated
image is below 256 KiB, so the previously used sector 0..5 erase range still
covers it. No device was flashed as part of integration.

`[TimePath]` reports the nominal goal/stop times, computation milliseconds,
expanded state count, goal, stopping extension and path codes. Failure clears
all of `path[]`, reports the status and enters the existing pre-run error halt.
Unknown/contradictory walls are closed. Cases 8/9 also remain orthogonal; no
additional diagonal conversion is applied. Small-turn-only cases retain their
existing restriction. The real angle-accumulation flag selects nominal versus
corrected angles; mode 3..7 cases 1/2 also use nominal angles through the OP UI.

The 19 previously documented 2015-maze no-path cases still reject the run.
Acceleration is not relaxed and there is no silent speed/solver fallback.

For an explicit comparison with the legacy solver, use a separate build:

```sh
cmake -S . -B build/classic-legacy -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DNIGHTFALL_CLASSIC_TIME_PLANNER=OFF -DNIGHTFALL_F405_ORTHOGONAL_PREVIEW=OFF
cmake --build build/classic-legacy --target nightfall_classic_r1_0
```

The pre-integration, user-validated tuning is retained at commit `664d880`.
