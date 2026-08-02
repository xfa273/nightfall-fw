# Slalom parameter closure baseline

## Purpose and scope

This document freezes a **PC-only geometric and timing audit baseline** for the
diagonal slalom planner.  It is not a firmware parameter update and it does
not claim that a mouse can safely run the provisional values below.

The primary profile is STM32F413 `f413_preorder`, shortest-run mode 2.  F405
`mini_r1_0` modes 2 through 5 are comparison profiles.  At the time this
baseline was recorded,
`params/f413_preorder/shortest_run_params_split.c` and
`params/mini_r1_0/shortest_run_params_split.c` were byte-identical.

The machine-independent data are in
`tools/solver_host/slalom_profile_baseline.h/.c`.  The executable audit is
`tools/solver_host/slalom_profile_audit.c` and is run with:

```sh
tools/solver_host/run_slalom_profile_audit.sh
```

The runner first verifies that the F413 and F405 mini parameter source tables
remain byte-identical, then compiles the real parameter table and compares
every copied turn field against it.  It builds with C11,
`-Wall -Wextra -Werror -Wpedantic`, ASan, and UBSan.  A successful baseline run
reports `all 882 checks passed`.

## Model assumptions

The audit calls `nf_motion_turn_plan()` from `common/route/motion_time.c`.
For a turn angle `theta`, angular acceleration parameter `alpha`, and
rounding scale `s = 1.2`, the nominal profile starts from:

```text
omega_peak = sqrt((2 / 3) * alpha * theta)
t_acc      = (omega_peak / alpha) * s
t_cruise   = max(theta / omega_peak - t_acc, 0)
t_angular  = 2 * t_acc + t_cruise
```

F413 is audited with its `2200 deg/s` angular-rate cap; none of the mode-2
entries in this baseline reaches that cap.  F405 comparison profiles are
uncapped.  The center velocity is constant throughout `dist_in`, the angular
profile, and `dist_out`.  The heading-dependent displacement during rotation
is integrated in two dimensions with the same 4096-interval Simpson model as
`motion_time.c`.

Those equations cannot simultaneously reproduce every tuned orthogonal
turn's logical endpoint, configured boundary velocity, and total duration.
For example, the mode-2 small-90 current model needs `90.632 mm` of travel in
`0.302107 s`, while every nonnegative-offset, monotonically turning
raised-cosine curve that closes at `(45,45)` is shorter than `90 mm`.  The host
planner therefore makes the approximation explicit:

- the current orthogonal spec supplies calibrated total duration and boundary
  velocity;
- a provisional seed supplies one exact-closing centre line for every turn;
- that line is parameterized over the calibrated duration by
  `v(t) = v_boundary + A*sin^2(pi*t/T)`, where
  `A = 2*(L_geometry/T - v_boundary)`.

The planner rejects a non-positive resulting speed.  The profile starts and
ends at the configured velocity and its distance integral is exactly the
geometry length.  Diagonal turns use the same provisional seed for duration
and geometry, so their correction is zero apart from numerical rounding.
This composite trajectory is a coherent PC surrogate, not a claim about the
unmeasured within-turn velocity of the real machine.

The audit deliberately uses the **logical turn angles** 45, 90, 135, and 180
degrees.  This matches the default F413 shortest-run feature
`angle_accum_mode=true`, and the F405 shortest modes/cases that enable angle
accumulation.  F405 mode-2 cases 1 and 2 do not enable that mode in their
selector and can therefore use stored tuned angles such as 89 degrees for
large 90 and 179 degrees for large 180.  This baseline does not claim to
reproduce those non-accumulating headings; in particular, a stored 89-degree
large-90 command has a -1-degree logical heading residual.

This is a center-line kinematic closure check only.  It excludes body sweep,
wall clearance, tire slip, feedback delay, the firmware control tick, sensor
correction, and HIL behavior.

## Logical geometry

Coordinates are in the local frame at turn entry: `x` is forward and positive
`y` is left.  Right turns mirror `y` and heading signs while preserving `x`.
Let `h = 45 mm`.

| Primitive | Logical angle | Target `(x, y)` expression | Target `(x, y)` mm |
| --- | ---: | --- | ---: |
| small 90 | 90 | `(h, h)` | `(45, 45)` |
| large 90 | 90 | `(2h, 2h)` | `(90, 90)` |
| large 180 | 180 | `(0, 2h)` | `(0, 90)` |
| 45 in | 45 | `(2h, h)` | `(90, 45)` |
| 45 out | 45 | `(3h/sqrt(2), h/sqrt(2))` | `(95.459415, 31.819805)` |
| V90 | 90 | `(h*sqrt(2), h*sqrt(2))` | `(63.639610, 63.639610)` |
| 135 in | 135 | `(h, 2h)` | `(45, 90)` |
| 135 out | 135 | `(h/sqrt(2), 3h/sqrt(2))` | `(31.819805, 95.459415)` |

### Logical diagonal distance is not the command distance

The geometry uses:

```text
logical diagonal half-section = 45 * sqrt(2)
                              = 63.639610306789 mm
```

The current firmware constant is separately defined as:

```text
DIST_D_HALF_SEC = 67.279 mm
```

The command constant is `3.639389693211 mm` (`5.718749%`) longer.  The audit
prints and checks both values separately.  `67.279` must not silently replace
`45*sqrt(2)` in topology or logical closure calculations.  Conversely, this
baseline does not authorize changing the existing command constant.

## Current turn parameters

Each cell is `velocity / alpha / dist_in / dist_out` in
`mm/s / deg/s^2 / mm / mm`.  The angles used by this audit are the logical
angles in the geometry table, not the independently stored tuning angles.

| Profile | small90 | large90 | large180 | 45in | 45out | V90 | 135in | 135out |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| F413 mode2 | `300/8920/10/14.2` | `500/4700/5/15` | `500/4697/12/19` | `500/6360/0/28` | `500/7700/15/0` | `500/12200/2/28` | `500/6888/9/17` | `500/6950/0/12` |
| F405 mode2 | same as F413 mode2 | same | same | same | same | same | same | same |
| F405 mode3 | `600/36000/4/14` | `1000/17300/1/6` | `1000/15000/0/0` | `1000/27200/0/35` | `1000/28000/17/0` | `1000/43000/6/23` | `1000/26500/5/25` | `1000/29000/0/26` |
| F405 mode4 | `800/51500/4/14` | `1000/17300/1/6` | `1000/15000/0/0` | `1200/16422/0/20` | `1200/16422/18/2` | `1200/25838/5/7` | `1200/17395/8/2` | `1200/17736/2/10` |
| F405 mode5 | `800/51500/4/14` | `1200/24800/2/8` | `1200/21000/0/4` | unavailable | unavailable | unavailable | unavailable | unavailable |

Mode-5 diagonal fields are absent/zero in the current profile.  They are
treated as **unavailable**, not as executable zero-valued turns.

## Current geometric residuals

Residual is `modeled endpoint - logical target`; norm is Euclidean distance in
millimeters.  Calibrated orthogonal parameters remain the duration source even
when this simplified continuous model has a visible closure residual.  Every
primitive has a separate PC-only exact-closure centre-line seed: leaving even
a sub-5-mm residual would make the next exact graph edge begin at a different
pose.  Current diagonal values are audited for context, but provisional seeds
supply both their geometry and timing.

| Profile | Primitive | Residual `(dx, dy)` mm | Norm mm | Classification |
| --- | --- | ---: | ---: | --- |
| F413/F405 mode2 | small90 | `(+4.043967, +8.243967)` | 9.182410 | current timing retained |
| F413/F405 mode2 | large90 | `(+4.647088, +14.647088)` | 15.366607 | current timing retained |
| F413/F405 mode2 | large180 | `(-7.000000, -2.672291)` | 7.492739 | current timing retained |
| F413/F405 mode2 | 45in | `(+11.729505, +8.735720)` | 14.625119 | current audit; provisional timing/geometry |
| F413/F405 mode2 | 45out | `(-5.998413, -0.977048)` | 6.077465 | current audit; provisional timing/geometry |
| F413/F405 mode2 | V90 | `(-5.997294, +20.002706)` | 20.882427 | current audit; provisional timing/geometry |
| F413/F405 mode2 | 135in | `(-10.256355, +13.192288)` | 16.710155 | current audit; provisional timing/geometry |
| F413/F405 mode2 | 135out | `(-2.709449, +3.789764)` | 4.658693 | current audit; provisional timing/geometry |
| F405 mode3 | small90 | `(-2.129949, +7.870051)` | 8.153182 | current timing retained |
| F405 mode3 | large90 | `(+4.452744, +9.452744)` | 10.448986 | current timing retained |
| F405 mode3 | large180 | `(0.000000, +7.734233)` | 7.734233 | current timing retained |
| F405 mode3 | 45in | `(+13.984234, +12.569155)` | 18.802725 | current audit; provisional timing/geometry |
| F405 mode3 | 45out | `(-0.364057, +0.528351)` | 0.641633 | current audit; provisional timing/geometry |
| F405 mode3 | V90 | `(+1.636662, +18.636662)` | 18.708389 | current audit; provisional timing/geometry |
| F405 mode3 | 135in | `(-19.170915, +20.641198)` | 28.170606 | current audit; provisional timing/geometry |
| F405 mode3 | 135out | `(-13.395002, +11.791546)` | 17.845634 | current audit; provisional timing/geometry |
| F405 mode4 | small90 | `(+2.331276, +12.331276)` | 12.549710 | current timing retained |
| F405 mode4 | large90 | `(+4.452744, +9.452744)` | 10.448986 | current timing retained |
| F405 mode4 | large180 | `(0.000000, +7.734233)` | 7.734233 | current timing retained |
| F405 mode4 | 45in | `(+46.511431, +19.829157)` | 50.561930 | current audit; provisional timing/geometry |
| F405 mode4 | 45out | `(+46.324094, +20.281430)` | 50.569339 | current audit; provisional timing/geometry |
| F405 mode4 | V90 | `(+33.123243, +35.123243)` | 48.278271 | current audit; provisional timing/geometry |
| F405 mode4 | 135in | `(+18.619136, +49.104900)` | 52.516316 | current audit; provisional timing/geometry |
| F405 mode4 | 135out | `(+19.591542, +47.972265)` | 51.818594 | current audit; provisional timing/geometry |
| F405 mode5 | small90 | `(+2.331276, +12.331276)` | 12.549710 | current timing retained |
| F405 mode5 | large90 | `(+5.663529, +11.663529)` | 12.965857 | current timing retained |
| F405 mode5 | large180 | `(-4.000000, +9.120604)` | 9.959187 | current timing retained |
| F405 mode5 | 45in/45out/V90/135in/135out | unavailable | unavailable | provisional timing/geometry |

The large mode-4 diagonal residuals are a useful warning: parameter values
that execute a turn are not automatically geometrically compatible with the
logical path lattice.

## PC-only provisional exact-closure seeds

> **These values are analytical seeds for host-side planning experiments.
> They are not firmware parameters, are not HIL-qualified, and must not be
> copied into a machine profile without the normal tuning and safety process.**

All seeds retain the current primitive's centre velocity where one exists and
solve the continuous model for geometric closure.  Values are rounded for
readability; the executable audit requires every seed's residual norm to be at
most `0.001 mm`.  Orthogonal current parameters remain the duration source;
their seeds only provide the continuous centre line.  Diagonal seeds provide
both duration and geometry.  The final sampled pose is canonicalized to the
logical endpoint only after passing this sub-micron gate, preventing topology
from changing with the requested sampling interval.

Each entry is `velocity / alpha / dist_in / dist_out` in the same units as the
current-parameter table.

| Profile | small90 | large90 | large180 | 45in / 45out | V90 | 135in / 135out |
| --- | --- | --- | --- | --- | --- | --- |
| F413/F405 mode2 | `300/8920/5.956033/5.956033` | `500/4700/0.352912/0.352912` | `500/4422.213141/15.5/15.5` | `500/7234.4/0/18.640` / reversed | `500/12200/7.997/7.997` | `500/8500/18.932/11.212` / reversed |
| F405 mode3 | `600/36000/6.129949/6.129949` | `1000/20000/3.083924/3.083924` | `1000/17688.852564/0/0` | `1000/28937.6/0/18.640` / reversed | `1000/43000/4.363/4.363` | `1000/34000/18.932/11.212` / reversed |
| F405 mode4 | `800/51500/1.668724/1.668724` | `1000/20000/3.083924/3.083924` | `1000/17688.852564/0/0` | `1200/41670.1/0/18.640` / reversed | `1200/53720.2/0/0` | `1200/48960/18.932/11.212` / reversed |
| F405 mode5 | `800/51500/1.668724/1.668724` | `1200/30000/4.839985/4.839985` | `1200/25471.947692/2/2` | `1200/41670.1/0/18.640` / reversed | `1200/53720.2/0/0` | `1200/48960/18.932/11.212` / reversed |

“reversed” swaps `dist_in` and `dist_out`.  Mode-5 diagonal seeds need an
assumed velocity because no current diagonal primitive exists; `1200 mm/s`
was borrowed from mode-5 large turns solely to make host experiments explicit
and reproducible.

## Straight-run parameter baseline

The time planner also needs the case-specific straight limits.  The compact
form below is `acceleration_straight / acceleration_straight_dash /
velocity_straight`; the diagonal column is
`acceleration_d_straight / acceleration_d_straight_dash /
velocity_d_straight`.  Units are `mm/s^2 / mm/s^2 / mm/s`.

| Mode | Cases | Orthogonal | Diagonal |
| ---: | --- | --- | --- |
| 2 | 1-2 | `2000/2000/1500` | unavailable |
| 2 | 3 | `1000/3000/3000` | unavailable |
| 2 | 4 | `1000/3000/4000` | unavailable |
| 2 | 5 | `1000/3500/4000` | unavailable |
| 2 | 6, 8 | `1000/2000/1000` | `1000/2000/1000` |
| 2 | 7, 9 | `1000/3500/4000` | `1000/3000/3000` |
| 3 | 1-2 | `4000/4000/1000` | unavailable |
| 3 | 3 | `8000/4000/2500` | unavailable |
| 3 | 4 | `10000/4000/2500` | unavailable |
| 3 | 5 | `12000/4000/2500` | unavailable |
| 3 | 6 | `4000/4000/4000` | `4000/8000/2500` |
| 3 | 7 | `4000/20000/4000` | `4000/8000/2500` |
| 3 | 8 | `4000/10000/3000` | `4000/8000/2500` |
| 3 | 9 | `4000/14000/3000` | `4000/8000/2500` |
| 4 | 1-3 | `6000/3000/2000` | unavailable |
| 4 | 4 | `8000/6000/2500` | unavailable |
| 4 | 5 | `8000/6000/3000` | unavailable |
| 4 | 6 | `7111.11/16000/3500` | `4000/8000/2000` |
| 4 | 7 | `7111.11/18000/3500` | `4500/9000/2500` |
| 4 | 8 | `7111.11/12000/2000` | `4000/8000/2000` |
| 4 | 9 | `7111.11/12000/3000` | `5000/8000/2000` |
| 5 | 1-3 | `8000/5000/2000` | unavailable |
| 5 | 4 | `12000/7000/2500` | unavailable |
| 5 | 5 | `14000/8000/3000` | unavailable |
| 5 | 6 | `11111.11/28000/5000` | `6000/12000/3000` |
| 5 | 7 | `11111.11/30000/5200` | `7000/14000/3500` |
| 5 | 8 | `11111.11/20000/4000` | `6000/12000/3000` |
| 5 | 9 | `11111.11/23000/4500` | `7000/14000/3500` |

The F413 mode-2 row is the primary current planner profile.  The F405 mode-2
row is numerically identical today; modes 3-5 remain comparison data rather
than F413 firmware commitments.

Mode 4's `7111.11f` is a rounded representation of the acceleration intended
to brake `800 mm/s` in one `45 mm` half-cell.  Its stored value requires
`45.000007896 mm`.  The PC motion model applies its documented sub-micron,
sub-ppm consistency snap for this boundary case; it does not change the
firmware parameter table.

## Mirror and regression checks

For every available current primitive and every emitted seed, the audit checks
the documented left-turn endpoint and then applies the firmware contract for a
right turn:

```text
right endpoint = (left x, -left y)
right target   = (left target x, -left target y)
```

It verifies mirrored forward/lateral residuals and equal residual norms.  It
also locks current endpoints to a `1e-6 mm` regression tolerance, requires a
PC geometry seed for every primitive, retains current orthogonal centre
velocity, and enforces seed closure at `<=0.001 mm`.

Passing this host audit is a prerequisite for using the data in path/time
planning experiments.  It is not a substitute for discrete path-code
validation, control-rate simulation, collision-envelope checking, or the HIL
process in `docs/ai/HIL_SAFETY.md`.
