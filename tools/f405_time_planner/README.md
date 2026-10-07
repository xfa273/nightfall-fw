# F405 classic orthogonal time planner preview

A fixed-memory, allocation-free 16x16 planner prepared on the `v1.0.0` classic
competition baseline. It emits only straight, small 90, large 90 and large 180
legacy path codes. Normal firmware still calls the original solver. This is a
**preview API**, not an enabled motion mode. See
[the preparation and hardware return plan](../../docs/CLASSIC_R1_2026_PREPARATION.md).

## Run host checks

```sh
tools/f405_time_planner/run.sh
tools/solver_host/run_route_motion_tests.sh
tools/f405_time_planner/run.sh tools/solver_host/testdata/16MM2014CX_kerilab.maze 2 4
tools/f405_time_planner/run.sh --matrix tools/solver_host/testdata/16MM2014CX_kerilab.maze
```

The default build uses AddressSanitizer and UndefinedBehaviorSanitizer. Set
`SANITIZE=0` for host timing. `CC` may select another C compiler. Host milliseconds
are not an F405 runtime benchmark. The CLI reads S/G markers in the maze; the
firmware adapter instead uses the compiled classic start and goal macros.

`--matrix` accepts multiple `.maze` files and checks all 54 classic mode/case
profiles. For cases 3--9, which use nominal angles, it compares the objective
against the existing general PC Dijkstra implementation. Cases 1/2 use the
actual corrected profile angles, unsupported by that general reference, and
are checked by independent legacy path replay instead. Tie paths may differ;
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

The option defaults to OFF. When ON, the linker explicitly retains
`f405_orthogonal_preview`; memory figures therefore include the real planner
workspace and code, even though no OP/UART/run entry invokes it. Mini's binary
remains unaffected. There is no 32x32 compact implementation in this branch.

The workspace is 36,876 bytes, with 3,073 maximum states and no heap allocation.
The 1,536-byte connector cache is on the foreground stack. The `.su` files under
the build tree expose static function frames; `check_memory.py` reserves at
least 8 KiB beyond the original linker reservations and protects the calibration
sector-9 boundary. The old control, logging, solver and calibration code is
retained. Linking does not establish hardware behavior.

## API and model contract

`nf_compact_orthogonal_plan` takes an explicit maze, run configuration, request,
caller-owned workspace and output. `f405_orthogonal_preview` is a foreground,
non-reentrant convenience wrapper using a private workspace and the unchanged
classic profile. It takes map bytes supplied by the caller and performs no
NVM read/write, peripheral access or motion. Do not pass the active runner's
path buffer while moving. Input/output/result regions must be disjoint.

Success means a **nominal-model plan** was derived and encoded. It does not
qualify that path for motor execution. Costs use first goal entry; output may
continue along a known corridor to stop. Time equivalence to the original
F405 runner's per-code acceleration/wall-end/terminal logic is not asserted.
That adapter validation precedes future motion integration; it is intentionally
not hidden behind an automatic switch to this planner.

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
