# tools/tuning

Host-side tuning helpers live here.

## `turn_tune.py`

`turn_tune.py` simulates and replays F413 turn trajectories in the same
local coordinate frame:

- `x_mm`: right side of the mouse is positive
- `y_mm`: forward is positive
- `theta_deg`: left turn is positive, right turn is negative

The simulator reproduces the current F413 smooth omega-profile turn shape:
in-offset straight segment, constant-velocity omega-profile turn core, and
out-offset straight segment. It reads F413 preorder params by default.

Examples:

```sh
python3 tools/tuning/turn_tune.py simulate --runner shortest --mode 2 --code 501
python3 tools/tuning/turn_tune.py simulate --runner search --search-index 0 --side right
python3 tools/tuning/turn_tune.py replay tools/logging/logs/trace_bin_20260620_210552.csv --runner shortest --mode 2 --code 502 --compare-sim
python3 tools/tuning/turn_tune.py fit --runner shortest --mode 2 --code 501 --target-x 104 --target-y 96 --target-theta -90
```

Notes:

- `simulate` is the ideal firmware plan from params.
- `replay` integrates the logged `target_velocity_mm_s` / `target_omega_mdps`
  and `real_velocity_mm_s` / `real_omega_mdps` streams. The logged target omega
  is the controller reference, so it may include heading correction outside the
  pure omega profile.
- `fit` prints suggested C initializer assignments only. It does not edit params.
  By default it varies velocity, alpha, in/out offsets, and the angle field.
