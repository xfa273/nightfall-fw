# Allocation-free F413 time oracle

`common/route/mcu_slalom_time_planner.{c,h}` implements the **primary goal-entry
minimum** of the generic slalom planner's F413 mode2/case8 graph, with KERI
patterns1..5 and nominal/low/crawl speeds500/300/100 mm/s. The source graph and
geometric trace are pinned to `4fb45ed`, already imported under `common/route`.
`mcu_slalom_tables.c` is generated from those implementations. No floating-point
operation, allocation, HAL access, or global mutable workspace occurs in the
MCU solver. The generic simulator oracle remains available for other profiles.

The certificate is limited to this finite motion graph, commanded kinematics,
and sampled centre-line clearance. It is not a physical optimality or body
clearance guarantee. Stop time describes a feasible complete braking tail after
the optimized goal crossing. It is not independently minimized over all routes.
This implementation may select a different equal-cost route: it omits the
generic planner's secondary turn-count tie breaker to save RAM. Primary integer
microsecond costs and the admissible motion graph are preserved.

## Call contract

1. Obtain `nf_mcu_slalom_workspace_bytes_for(width,height)` and provide that
   many bytes with8-byte alignment. Dimensions2..32 are supported, including
   rectangular maps. The maximum declaration is200 KiB.
2. `nf_mcu_slalom_begin` copies walls and goals into that workspace. Walls are
   symmetric cell masks N/E/S/W=1/2/4/8, with closed exterior boundaries. A
   nonzero goal byte selects its cell. Start heading is0..3 in NESW order.
3. Repeatedly call `nf_mcu_slalom_step(context,budget)`. Budget0 only polls;
   exhaustion returns`PENDING`. The caller may abandon the context at any time
   and restart it in the same workspace. There is no cancellation cleanup.
4. `nf_mcu_slalom_result` copies status/statistics and optionally a flat,
   symmetric required-open mask. **Only`EXACT` has
   `requirements_complete=true`**. The requirements include the complete
   stopping tail, successful KERI topology checks, and sampled geometric
   crossings. Incomplete/error results clear a supplied requirement array.

Unknown-open/unknown-closed construction, map generation/epoch tracking, start
and goal selection, profile identity, and exclusive ownership of workspace
belong to the caller. Do not let trace capture overwrite a leased workspace.
An old optimistic exact result stays a valid lower bound under additive wall
observations. It is usable as a feasible route only if none of its required
edges has become a wall. A certificate additionally requires every required
edge to be known open and the policy's goal/epoch/profile conditions to hold.

Invalid/asymmetric inputs, insufficient workspace, no path, no feasible
stopping tail, and arithmetic overflow have distinct statuses. All require
fallback rather than a certificate. Distances use exact rounded integer
microseconds below`UINT32_MAX`; an overflowing relaxation fails closed. No
speed quantization, beam restriction, reduced state graph, or queue truncation
is used.

## Memory and queue

For width`w` and height`h`, there are
`12*(w*h+(w-1)*h+(h-1)*w)` states: each legal cardinal centre pose or diagonal
wall-anchor pose has three speeds. The32×32 graph has36,096 states;16×16 has
8,832. State distances consume4 bytes each; settled flags consume1 bit each.

A binary heap stores the minimum unsettled state for each group of eight
contiguous states. Each group has three16-bit words (heap entry, position,
representative). Decrease-key checks the representative and sifts at most13
levels on32×32. Removing a representative inspects at most eight distances,
then sifts the heap. Every finite unsettled state is represented, without
per-state heap positions or stale duplicate entries. The arrays are sized for
the entire graph; a difficult maze cannot exhaust a smaller queue capacity.

A16-bit mask per legal pose caches all14 kind/side feasibility predicates.
Bit15 marks completion. On a cache miss,14 cooperative steps test one action
each; all three speed states and every connector prefix then reuse that mask.
This costs24,064 bytes on32×32. Table-invalid and cached-infeasible edges are
skipped in the original edge order, without removing a feasible motion.

Predecessors are recovered after Dijkstra finishes by enumerating inverse
turn/connector edges and checking`distance(u)+cost(u,v)==distance(v)`. Positive
edge costs strictly decrease distance during reconstruction. Only reconstructed
selected edges are recorded as dependencies. This avoids parent and turn-count
arrays while retaining a sufficient route certificate.

Host64-bit workspace usage is51,440 B for16×16 and202,528 B for32×32; the
32-bit ARM context is slightly smaller. `workspace_bytes_for` is authoritative
for the build's ABI. Tables occupy about50 KB of read-only memory and normally
reside in Flash. The opt-in platform adapter may lease an existing idle trace
buffer; the common module does not change trace capacity.

## Budget interpretation

A work unit is one terminal candidate, candidate motion edge, cache predicate,
heap pop, connector half-step, inverse-edge check, or small initialization block. It is **not** a CPU cycle or fixed-time
quantum. One motion edge can include a heap sift and at most42 cheap mask/timing-table
probes to skip infeasible candidates. A cache predicate checks the fixed
required-edge conjunction and endpoint validity. Forward
rays, braking tails, reverse verification, and dependency reconstruction yield
after each connector half-step. Distance/group/settled initialization yields
after at most32 elements; it is part of the step budget. `begin` still copies
and validates up to1024 wall/goal cells and clears the small context; copying
requirements in`result` is also outside the step budget. Those costs need
separate measurement in the caller's foreground schedule.

`work_units` counts completed dispatches. `expanded_states` counts Dijkstra
settlements; `relaxed_edges` counts strict distance improvements.
`heap_peak` counts active eight-state groups, not queued individual states.
`action_count` counts reconstructed compound connector/turn actions, including
the final stopping action, excluding the launch offset. The32-bit statistics
are diagnostic counters, not correctness inputs.

Host work/time must not be extrapolated to Cortex-M4 deadlines. A foreground
slice must be measured with DWT on the exact build and bounded so switch,
sensor and run-session guards keep executing. Full-solve latency can span many
cell observations even when every slice is short. Pending work therefore
requires a guarded exploration fallback, not waiting inside a moving cell
boundary handler.

## Reproduction and verification

```sh
tools/solver_host/generate_mcu_slalom_tables.sh
python3 -m unittest tools.exploration_sim.test_mcu_oracle -v
python3 tools/exploration_sim/benchmark_mcu_oracle.py --catalog \
  --output build/exploration_sim/mcu-catalog-check.json
tools/hil/exploration_bench/build.sh planner \
  build/exploration_mcu/planner16.elf \
  build/exploration_sim/mcu-classic2014-cases.h
```

`tools/exploration_sim/mcu_oracle.py` compiles exactly the MCU C files into a
host shared library for tests. The tests check full32×32 state support,
obstructed maps, nonzero start poses, rectangular maps, budget invariance,
immutable snapshots, capacity rejection and incomplete-result behavior. Every
successful test also closes all edges outside the reported requirement set
and asks the generic oracle to recover the same goal-entry cost.

The exported16MM2014CX and32MM2023HX initial/first-goal/mid/certificate/full-truth
snapshots each passed optimistic and conservative parity against the generic
F413 mode2/case8 oracle (20 projections). Requirements were independently
checked by the closed-except-required replay. The16×16 maze is used here with
the **F413 half-size timing profile** for oracle equivalence, not the classic
machine's physical exploration speed. Ignored detailed measurements are in
`build/exploration_sim/mcu-oracle-parity.json`. Hardware timing and safety
records belong to the root task's HIL work log.

## Integer timing and generated-profile identity

The generic oracle already performs Dijkstra with`uint64_t` integer
microseconds. `nf_motion_seconds_to_us` computes
`floorl((long double)seconds*1000000+0.5)` for each primitive component. The
MCU generator calls that same conversion on the same connector plans, full
turn durations, sampled goal-crossing times, stopping-prefix times, and launch
offset. Runtime adds those saved components in the same order. Thus the MCU
uses the generic graph's existing rounding; it does not introduce another
microsecond rounding approximation. Switching storage from64 to32 bits is
exact until the explicit overflow guard rejects a solve.

Optimistic and conservative bounds are consequently bounds on that integer
model. Nearest-microsecond rounding is not directed lower/upper rounding of
continuous physical time. Each independently rounded component can differ
from its computed double duration by up to0.5 microseconds, in addition to the
much larger unmodelled geometry and control effects. Do not describe a model
certificate as an exact continuous-time physical bound.

The active model is named by`NF_MCU_SLALOM_PROFILE_ID`. It is fixed at compile
time; changing exploration cruise/turn speed does not silently alter this
shortest-run objective. Regenerating timing tables or changing the selected shortest objective
requires a new caller profile epoch so cached certificates cannot be promoted
across models. Runtime configuration for arbitrary shortest speeds is not
accepted by this fixed-profile API. `generate_mcu_slalom_tables.sh --check`
regenerates and rejects stale output. The exported
`nf_mcu_slalom_inputs_sha256[65]` hashes generator/source/schema and all
compiler-discovered repository dependencies, including the profile and
geometry inputs. Generation/checking must accompany changes to those files.

All65 valid complete mazes in the68-entry KERI catalog passed primary-cost/status
parity;64 produced sufficient exact requirements. `32MM2009HX` returned
`no-feasible-terminal` in both implementations. Three source files contain
unknown edges and were excluded by the dataset validator. No queue capacity
failure occurred. Detailed ignored results are
`build/exploration_sim/mcu-catalog-parity.json`.


## Exact replacement of procedural turn guards

The generated turn requirements now combine the KERI topology guard and the
sampled geometric trace. For each of56 valid source-class/heading/kind/side
templates and each of three speeds, generation closes each internal edge of a
32×32 reference maze individually. An edge is required exactly when closing it
makes the combined predicate false with all other edges open. Generation then
closes **every other internal edge** and checks that the combined predicate
still succeeds. All168 pooled edge requirements pass this check; the prior
geometry-only pool contained144.

Both source predicates are monotone in open edges: topology uses conjunctions
and alternatives of open-edge checks; the centre-line trace rejects closed
crossings. A single-edge failure proves that edge necessary under every more
restrictive wall assignment. Success with every non-required edge closed
proves the conjunction sufficient under every less restrictive assignment.
Together these establish exact predicate equivalence for all wall assignments,
without enumerating all2^1984 assignments. Runtime source/destination validity
and translated edge bounds account for the actual rectangular maze boundary.

The pool symbol is now`nf_mcu_required_turn_edges`, so a kernel expecting full
requirements cannot link against an old geometry-only table. Feasibility is
independent of incoming speed after the connector; equality across the three
turn variants is checked during generation before using a shared pose mask.
The selected action's requirements are replayed when constructing the final
certificate; a cached true bit alone never substitutes for dependency output.

Regressions after this optimization cover65 complete catalog mazes plus20
partial-map projections, with exact status/primary cost parity and independent
closed-except-required sufficiency. Detailed results are in
`build/exploration_sim/mcu-complete-cached-parity.json`. The SRAM benchmark
validator is read-only and checks MCU status, entry/stop microseconds,
dependency count and FNV checksum, settled/improved-state counters, work units,
and peak heap groups against the exact same C running on the host. Host and
MCU workspace sizes may differ because pointers have different sizes:

```sh
python3 tools/hil/exploration_bench/validate_planner.py \
  --cases build/exploration_sim/mcu-half2023-cases.json \
  --result build/exploration_mcu/planner32-cached/result.json \
  --output build/exploration_mcu/planner32-cached/validation.json
```


## Motor-off MCU timing after exact caching

Root-task SRAM-only measurements ran on F413 at100 MHz, with code/tables and
workspace in SRAM. Ten optimistic/conservative projections of32MM2023HX
matched the host validator exactly. The full-Flash SHA256 stayed unchanged and
motors/FRAM were not accessed. These are measured samples, not a production
foreground deadline or worst-case execution-time proof.

| Snapshot | Optimistic CPU time | Conservative CPU time |
|---|---:|---:|
| Initial observation |1.252 s|0.0067 s|
| First goal |11.118 s|0.199 s|
| Uncertain middle |5.613 s|0.212 s|
| Certificate |2.178 s|0.654 s|
| Full truth |1.115 s|1.115 s|

The largest measured`step(1)` was20.94 microseconds. The prior already-sliced
procedural implementation took52.216 seconds for the first-goal optimistic
snapshot; exact table predicates and pose caching reduced that to11.118
seconds without changing the objective. A complete solve therefore still
needs meaningful CPU time and should run cooperatively with guarded fallback.
Do not convert these figures directly into exploration travel-time savings.
Current ignored hardware evidence and strict validation are under
`build/exploration_mcu/planner32-cached/`.

The same final source also passed all ten16MM2014CX projections on F413,
using the same fixed half-size F413 timing model:

| Snapshot | Optimistic CPU time | Conservative CPU time |
|---|---:|---:|
| Initial observation |0.455 s|0.0018 s|
| First goal |0.093 s|0.055 s|
| Uncertain middle |0.379 s|0.067 s|
| Certificate |0.240 s|0.087 s|
| Full truth |0.164 s|0.164 s|

Largest measured`step(1)` was20.21 microseconds; largest`begin` was0.394 ms.
Device workspace was51,368 bytes, versus51,440 bytes on the host ABI. Evidence
and exact mailbox validation are under`build/exploration_mcu/planner16-final/`.
The final32 run is under`build/exploration_mcu/planner32-final/`; all ten rows
also passed the current host validator.

The API audit confirmed caller-owned allocation, bounded array initialization,
immutable snapshots, and no cancellation resources. Input/output buffers must
not overlap workspace, and one context must not be executed concurrently.
Discard/restart is safe after`step` returns. `result` clears a reused caller's
certificate fields even for a null context or insufficient dependency-buffer
capacity; its failure paths cannot retain a previous successful certificate.
