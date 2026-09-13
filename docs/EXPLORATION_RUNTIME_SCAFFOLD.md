# F413 runtime exploration port scaffold

This branch is based on `3910fa6eaed4ed48550336b413799bd21256fec8` and preserves
that revision's runtime machine migration, optical synchronization, phase
restart, post-goal map handling and destructive-NVM diagnostic guard. It was
prepared in an isolated worktree; the normal checkout and hardware were not
changed.

`NIGHTFALL_F413_EXPLORATION_ENABLED` defaults to OFF. ON compiles **fallback-only
integration seams**: the adapter reports an unsupported runtime route model,
returns the existing Adachi decision unchanged, and never certifies completion.
It links no pinned oracle or generated timing tables, and borrows no workspace.
This is not an operational improved-exploration port.

The separate `codex/f413-exploration-ready` branch contains the portable
cooperative policy and exact integer oracle. The oracle implementation was
recorded at `cc56c84`; it models the generic F413 mode2/case8 graph from
`4fb45ed`, not the runtime execution graph at this branch's base.

## Port seams

The calls are inside the existing foreground search flow: initialization after
the optical START guard while control is stopped, cooperative polling after the
existing guarded wait, decision/arrival preparation, and cleanup after control
stops. Actual GOAL arrival is observed before phase completion and the subsequent
restart entry moves the pose. No phase-restart, control, motor, NVM or trace
implementation is replaced.

A future validated adapter must reuse the existing
`f413_trace_log_try_borrow_idle_scratch` /
`f413_trace_log_release_idle_scratch` lease. Its memory size and owner must be
checked. The runtime machine's precomputed-route compatibility flag alone does
not identify or validate the separate exploration oracle's model.

## Remaining model work

The pinned generator rejects this base because the start boundary changes from
100 to 282.842712475 mm/s. Orthogonal acceleration changes from1000 to4000 mm/s²,
straight maximum speed from1000 to1500 mm/s, and turn timings also change.
An isolated audit that derives the third speed slot from the latest generic
context regenerated all56 turn templates,168 sufficient dependencies and176
goal crossings. This establishes that numeric table generation is adaptable;
it does not establish compatibility with the execution planner. The adapted
integer kernel passed20 recorded optimistic/conservative projections against
the latest generic planner, including independent required-edge sufficiency
(18 exact results and2 no-path results). These audit changes are intentionally
excluded from this fallback-only port.

The runtime planner additionally has LOW cardinal-wall states for small90,
nominal-only normal turns, execution-specific connector/terminal rules, and a
second recovery pass. Merely disabling the global recovery pass is not yet a
proved lower-bound model: the primary pass itself admits a small terminal turn
when the corresponding large turn cannot brake but the small turn can
(`f413_rp_small_required_for_goal_stop`). Closing a wall can change that gate.
A primary-only certificate needs either a proof that these conditional edges
cannot lower the objective, or a lower graph covering their possible cost.
No counterexample to objective monotonicity is asserted here; edge-deletion
monotonicity alone is insufficient evidence. A separate bounded audit found
the paired optimistic large-turn goal entry no later than the newly admitted
small turn in3936 tested gates. That supports investigating a primary cost
dominance proof; it is not an implemented execution-graph adapter.

One bounded design candidate is a monotone union of every executable action
that the primary or recovery graph may admit, paired with a separately
execution-validated known route as an upper bound. That needs explicit objective
semantics, complete dependency output, and tests of state/connector/terminal
parity. It has not been implemented or validated on this branch. Changing the
new-machine shortest planner's behavior is outside this scaffold.

## Build evidence

Both builds succeeded with GNU Arm15.2.1, Debug, target
`nightfall_stm32f413`. ON emitted the explicit unsupported-model configure
message. Destructive NVM diagnostics stayed OFF. The source additions do not
change F405 targets. No image was flashed and no hardware command was run.

| Option | RAM | Flash | ELF SHA256 |
|---|---:|---:|---|
| OFF |274120 B|367140 B|`275e52fe48dece93af966972aa81fb5c38277fdb688b90fe9d298f909031982d`|
| ON |274120 B|367564 B|`6ca9d5e3e2a318d1f1e93760f755a9c45ed70f61ccaa4b3a713fed24573ce379`|

The fallback API was also compiled on the host with warnings as errors: every
target value retained caller decision fields, null optional outputs were safe,
goal observation succeeded, and only mode1/case1 initialization emitted the
unsupported message. `git diff --check` passed.

Reproduce the builds with:

```sh
cmake --preset Debug -DNIGHTFALL_F413_EXPLORATION_ENABLED=OFF
cmake --build --preset Debug-stm32f413
cmake -S . -B build/Debug-on -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \
  -DCMAKE_BUILD_TYPE=Debug -DNIGHTFALL_F413_EXPLORATION_ENABLED=ON
cmake --build build/Debug-on --target nightfall_stm32f413
```

ELFs and logs remain ignored under the parent task's
`build/exploration_mcu/latest-check/` and `build/exploration_mcu/latest-*.log`.
The hashes above describe the tested images before this documentation commit;
normal build metadata can change their hashes on a later rebuild.
