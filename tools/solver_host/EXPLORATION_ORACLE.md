# Exploration simulator time oracle

`tools/exploration_sim/oracle.py` exposes the existing time-based shortest
planners to the partial-observation simulator. It compiles a local shared
library on first use; subsequent launches reuse a content-addressed binary in
`build/solver_host`. A C compiler and Python 3 are sufficient. No hardware,
network, sibling worktree, or runtime git object is used.

```python
from tools.exploration_sim.oracle import TimeOracle

oracle = TimeOracle("mini_r2_0", mode=2, case=8, planner="slalom")
result = oracle.solve(walls, goals, start=(0, 0), heading=0)
# result["goal_entry_s"], result["stop_s"], result["required_edges"]
```

The exploration motion profile is separate: changing mode 1 search speed does
not silently change the eventual shortest-run objective.

## Imported implementation and scope

The active base checkout did not yet contain the user's time planners. These
files were imported byte-for-byte from local branch
`codex/diagonal-time-planner`, commit `4fb45ed`:

- `common/route/{motion_time,orthogonal_time_planner,route_clearance,slalom_time_planner,slalom_plan_legacy_codec,legacy_path_codec}.{c,h}`
- `tools/solver_host/{slalom_profile_baseline,slalom_time_plan_host,maze_ascii}.{c,h}`

They keep their original paths to ease a future merge of that branch. None of
these additions is wired into firmware CMake targets. Existing F405/F413
firmware sources, hardware runners, and parameter files are unchanged.

The adapter's shortest-run defaults are:

| Machine | Planner | Shortest profile | Geometry and timing basis |
| --- | --- | --- | --- |
| `mini_r2_0` | Slalom | Mode 2 case 8 | Imported F413 profile; patterns 1–5, low/crawl variants, straight and diagonal acceleration, feasible stopping tail |
| `classic_r1_0` | Orthogonal | Mode 2 case 1 | Compiled classic shortest table; small/large 90° and large 180° turns, straight acceleration, feasible stopping tail |

Mini shortest case 1 has no diagonal straight limits in the table and is
rejected for the slalom planner. Classic diagonal geometry has no matching
calibrated baseline in the imported branch, so the adapter rejects classic
slalom instead of silently substituting mini geometry. Orthogonal planning is
also available for mini. Supported shortest modes are 2–5 and cases 1–9;
individual table entries may be unavailable.

For higher mini modes the corresponding imported mini profile is used with
the F413 5 mm launch offset and 2200 deg/s angular-rate cap. The diagonal
centre-line seeds remain provisional, exactly as documented in the imported
planner. Body swept clearance is not part of the exploration certificate.

## Coordinates and return contract

Input `walls[y][x]` uses y increasing north and four bits `1 << d`, where
`d = 0,1,2,3` means north/east/south/west. The C adapter translates this to the
planner's north=8/east=4/south=2/west=1 convention. It validates consistency
and closed exterior boundaries; it does not invent walls or goals.

Successful output has `status: "ok"`, `goal_entry_s`, `stop_s`,
`expanded_states`, `actions`, and model/profile/source metadata. The
optimized criterion is **first goal entry time**; `stop_s` describes the
selected feasible stopping tail and is not an independently minimized cost.
Non-success status is explicit and has no numeric cost.

`required_edges` contains canonical `[x,y,d]` north/east edges. It includes
primitive topology guards, sampled centre-line crossings and the complete
stopping tail, including required edges that lie beyond first goal entry.
For slalom, the adapter temporarily blocks individual open edges and replays
the exact selected route with the prepared trajectory cache. It then verifies
that opening **only** the resulting required set still permits the route. If
alternative guard branches invalidate that sufficient-set check, it falls
back to the original map's complete open-edge set. This is conservative and
cannot cause an early information-sufficiency certificate.

`route_points` is a display polyline in fractional cell-centre coordinates:
`[0,0]` is the start-cell centre. Slalom turn points use the imported geometry;
orthogonal points represent logical cells, not a predicted physical turn
centre-line. `details=False` skips these two potentially larger outputs.

## Why optimistic/conservative bounds are valid

Both planners run exact Dijkstra search over their finite motion graphs.
With unknown edges open, the graph contains every route feasible in the true
maze; its optimal first-entry time is a lower bound. With unknown edges
closed, any returned route is executable in every completion of the known
map; its cost is an upper bound. Equality certifies optimality **within this
planner's motion graph, geometry and time model**, not physical global
optimality or a calibrated real-machine elapsed time.

A valid optimistic route need not be replanned when a wall elsewhere is
observed. If the returned sufficient required-open set stays open, that route
still realizes the old lower bound and newly added walls cannot improve the
optimum. If every required edge is known open, the same route realizes an
upper bound equal to the lower bound, so all-maze exploration is unnecessary.

## Host acceleration and checks

The shared-library build injects `slalom_trace_cache.inc` after the unchanged
planner's turn-trace definition in an ignored generated C file. The cache is
thread-local, lasts for one solve and memoizes the full exact trace result by
anchor, heading, primitive, speed variant and side, checking destination and
exit heading too. It changes no graph edges, motion costs, tie-breaks or
stopping rules. Requirement extraction runs after this cache is destroyed,
because it changes walls during replay. Allocation failure falls back to
uncached replay.

```sh
python3 -m unittest tools.exploration_sim.test_oracle -v
tools/solver_host/run_solver_host.sh --explore-sim --max-steps 512
```

Oracle checks compare the complete cached/uncached plan JSON, verify monotonic
cost under wall additions, reopen only reported requirements and verify equal
optima for both machine models, and exercise coordinate/profile failures. Independent instances are also run
concurrently through ctypes (which releases the Python GIL) and compared
against sequential results to check the thread-local replay cache.
The existing 16MM2014CX reference gives the same mini slalom first-entry result
as the source branch: `13.223795 s`.

On the development Mac, open 32×32 slalom planning fell from approximately
9.7 s to 2.0 s with the host trace memoizer. A fully known 32MM2023HX solve with
requirements took approximately 0.21 s; 100 uncached cost-only calls on that
maze took 10.096 s in total (0.101 s per call). Wall-clock solve performance is
machine-dependent and is not added to the simulated robot driving clock.
