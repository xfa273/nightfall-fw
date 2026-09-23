#!/usr/bin/env python3
"""Load explicit measured turns and the historical video calibration catalog.

No firmware, hardware, camera, or checkout state is modified. Coordinates are
vehicle-right x, forward y, and CCW-positive theta. Legacy captures are always
diagnostic: inferred alignment cannot identify absolute controller delay.
"""
from __future__ import annotations

import bisect
import csv
import hashlib
import json
import math
import re
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

try:
    from .turn_tune import Constants, TurnSpec, build_profile, simulate_turn
except ImportError:
    from turn_tune import Constants, TurnSpec, build_profile, simulate_turn


@dataclass(frozen=True)
class MeasuredSample:
    t_s: float
    x_mm: float
    y_mm: float
    theta_deg: float | None = None


@dataclass
class MeasuredRun:
    id: str
    group_id: str
    spec: TurnSpec
    constants: Constants
    samples: list[MeasuredSample]
    provenance: dict[str, Any]
    warnings: list[str] = field(default_factory=list)
    diagnostic_only: bool = True

    @property
    def time_s(self): return [s.t_s for s in self.samples]
    @property
    def x_mm(self): return [s.x_mm for s in self.samples]
    @property
    def y_mm(self): return [s.y_mm for s in self.samples]
    @property
    def theta_deg(self):
        return None if any(s.theta_deg is None for s in self.samples) else [s.theta_deg for s in self.samples]
    @property
    def turn(self): return self.spec
    @property
    def group(self): return self.group_id


@dataclass
class Dataset:
    runs: list[MeasuredRun]
    warnings: list[str]
    exclusions: list[dict[str, str]]

    @property
    def trials(self): return self.runs
    @property
    def excluded(self): return self.exclusions


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _resolve(path: str, root: Path) -> Path:
    result = Path(path)
    if result.is_absolute():
        # Historical reports contain the original checkout's absolute path.
        # Rebase the known project-relative artifacts when a data root is given.
        for anchor in ("sessions", "tools/logging/logs"):
            marker = "/" + anchor + "/"
            if marker in str(result):
                rebased = root / (anchor + "/" + str(result).split(marker, 1)[1])
                if rebased.exists(): return rebased
        return result
    return root / result


def _finite(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result): raise ValueError(f"{name} must be finite")
    return result


def _spec(params: dict[str, Any], selection: dict[str, Any], label: str) -> TurnSpec:
    angle = params.get("signed_angle_deg")
    if angle is None:
        angle = params.get("effective_angle_deg", params.get("angle_deg"))
        if angle is None: raise ValueError("params require signed_angle_deg or effective_angle_deg")
        angle = abs(_finite(angle, "angle")) * (-1 if selection.get("side") == "right" else 1)
    spec = TurnSpec(
        runner=selection.get("runner", "shortest"), label=label,
        signed_angle_deg=_finite(angle, "angle"),
        alpha_deg_s2=_finite(params["alpha_deg_s2"], "alpha"),
        velocity_mm_s=_finite(params["velocity_mm_s"], "velocity"),
        dist_in_mm=_finite(params.get("dist_in_mm", 0), "dist_in"),
        dist_out_mm=_finite(params.get("dist_out_mm", 0), "dist_out"), source_fields={})
    if spec.velocity_mm_s <= 0 or spec.alpha_deg_s2 <= 0 or not 0 < abs(spec.signed_angle_deg) <= 360:
        raise ValueError("velocity/alpha must be positive; angle magnitude must be in (0,360]")
    if min(spec.dist_in_mm, spec.dist_out_mm) < 0: raise ValueError("offsets must be nonnegative")
    return spec


def _constants(data: dict[str, Any]) -> Constants:
    c = Constants(_finite(data["rounding_scale"], "rounding_scale"),
                  _finite(data["omega_cap_deg_s"], "omega_cap_deg_s"))
    if c.rounding_scale <= 0 or c.omega_cap_deg_s <= 0: raise ValueError("constants must be positive")
    return c


def _parameter_group(params: dict, spec: TurnSpec, constants: Constants, machine: str, regime: str) -> str:
    values = {"machine": machine, "runner": spec.runner, "angle": spec.signed_angle_deg,
              "velocity": spec.velocity_mm_s, "alpha": spec.alpha_deg_s2,
              "in": params.get("dist_in_mm", 0), "out": params.get("dist_out_mm", 0),
              "rounding": constants.rounding_scale, "cap": constants.omega_cap_deg_s}
    return hashlib.sha256(json.dumps(values, sort_keys=True).encode()).hexdigest()[:20] + ":" + regime


def _read_csv(path: Path, columns: dict[str, str] | None = None) -> tuple[list[tuple], dict]:
    with path.open(encoding="utf-8-sig") as stream:
        reader = csv.DictReader(line for line in stream if not line.lstrip().startswith("#"))
        names = reader.fieldnames or []
        columns = dict(columns or {})
        def choose(role, candidates, required=True):
            name = columns.get(role) or next((n for n in candidates if n in names), None)
            if required and name not in names: raise ValueError(f"{path}: missing {role} column")
            return name
        time = choose("time", ("t_s", "time_s", "video_pts_s"))
        x = choose("x", ("x_mm", "x_right_mm"))
        y = choose("y", ("y_mm", "y_forward_mm"))
        heading = choose("theta", ("theta_deg", "yaw_deg_unwrapped", "yaw_deg", "heading_deg"), False)
        rows = []; invalid = 0; total = 0
        for row in reader:
            total += 1
            try:
                if float(row.get("pose_valid", row.get("tracking_valid", "1")) or 0) < .5:
                    invalid += 1; continue
                values = [_finite(row[name], name) for name in (time, x, y)]
                yaw = None
                if heading and row.get(heading) and float(row.get("heading_valid", "1") or 0) >= .5:
                    yaw = _finite(row[heading], heading)
                rows.append((*values, yaw))
            except (ValueError, KeyError, TypeError): invalid += 1
    if len(rows) < 8: raise ValueError(f"{path}: fewer than eight valid measured poses")
    if any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    if heading:
        previous = None
        for i, row in enumerate(rows):
            if row[3] is None: continue
            yaw = row[3]
            if previous is not None: yaw = previous + (yaw - previous + 180) % 360 - 180
            rows[i] = (*row[:3], yaw); previous = yaw
    return rows, {"csv_rows": total, "valid_rows": len(rows), "invalid_rows": invalid,
                  "valid_fraction": len(rows) / total, "columns": {"time": time, "x": x, "y": y, "theta": heading}}


def _interp(times: list[float], values: list[float], t: float) -> float:
    i = bisect.bisect_right(times, t)
    if i == 0: return values[0]
    if i >= len(times): return values[-1]
    w = (t-times[i-1])/(times[i]-times[i-1])
    return values[i-1] + w*(values[i]-values[i-1])


def _normalize(rows: list[tuple], start: float, end: float, heading: float,
               frame: str, trusted: bool, origin: tuple[float, float] | None = None) -> list[MeasuredSample]:
    times = [r[0] for r in rows]
    if not (times[0] <= start < end <= times[-1]): raise ValueError("segment is outside available trajectory")
    if frame not in ("board", "local"): raise ValueError("coordinate_frame must be board or local")
    xs = [r[1] for r in rows]; ys = [r[2] for r in rows]
    ox, oy = origin or (_interp(times, xs, start), _interp(times, ys, start))
    h = math.radians(heading)
    selected = [(start, _interp(times,xs,start), _interp(times,ys,start), None)]
    selected += [r for r in rows if start < r[0] < end]
    selected += [(end, _interp(times,xs,end), _interp(times,ys,end), None)]
    if trusted:
        valid_heading = [(r[0],r[3]) for r in rows if r[3] is not None]
        if len(valid_heading) != len(rows): raise ValueError("trusted heading contains missing/invalid samples")
        yaw_times, yaw_values = map(list, zip(*valid_heading))
        selected = [(r[0],r[1],r[2],_interp(yaw_times,yaw_values,r[0])) for r in selected]
    result = []
    for t,x,y,yaw in selected:
        dx,dy = x-ox,y-oy
        if frame == "board": nx,ny = math.sin(h)*dx-math.cos(h)*dy, math.cos(h)*dx+math.sin(h)*dy
        else: nx,ny = math.cos(h)*dx+math.sin(h)*dy, -math.sin(h)*dx+math.cos(h)*dy
        result.append(MeasuredSample(t-start,nx,ny,None if not trusted else yaw-heading))
    if len(result) < 8: raise ValueError("segment contains fewer than eight samples")
    if max(b.t_s-a.t_s for a,b in zip(result,result[1:])) > .05:
        raise ValueError("selected trajectory has a pose gap exceeding 50 ms")
    return result


def _line_heading(rows: list[tuple]) -> tuple[float, float]:
    # The capture includes long approach/exit straights. An ordered PCA fit over
    # the first 8-28% of its traveled distance supplies a position-only heading.
    distance = [0.0]
    for a,b in zip(rows,rows[1:]): distance.append(distance[-1]+math.hypot(b[1]-a[1],b[2]-a[2]))
    fit = [r for r,s in zip(rows,distance) if .08*distance[-1] <= s <= .28*distance[-1]]
    if len(fit) < 5 or math.hypot(fit[-1][1]-fit[0][1],fit[-1][2]-fit[0][2]) < 25:
        raise ValueError("historical approach does not contain a 25 mm straight heading baseline")
    mx=statistics.mean(r[1] for r in fit); my=statistics.mean(r[2] for r in fit)
    xx=sum((r[1]-mx)**2 for r in fit); yy=sum((r[2]-my)**2 for r in fit)
    xy=sum((r[1]-mx)*(r[2]-my) for r in fit)
    h=.5*math.atan2(2*xy,xx-yy)
    if math.cos(h)*(fit[-1][1]-fit[0][1])+math.sin(h)*(fit[-1][2]-fit[0][2]) < 0: h+=math.pi
    residuals=sorted(abs(-math.sin(h)*(r[1]-mx)+math.cos(h)*(r[2]-my)) for r in fit)
    p95=residuals[min(len(residuals)-1,int(.95*len(residuals)))]
    if p95 > 3: raise ValueError(f"historical approach heading residual {p95:.2f} mm exceeds 3 mm")
    return math.degrees(h),p95


def _historical_segment(rows: list[tuple], report: dict, spec: TurnSpec, constants: Constants):
    moving=[r for r in rows if report['motion_start_s'] <= r[0] <= report['motion_end_s']]
    if len(moving)<12: raise ValueError("historical motion window is too short")
    heading,residual=_line_heading(moving)
    observed=[r for r in moving if r[3] is not None]
    if len(observed)<12: raise ValueError("historical turn alignment needs measured headings")
    sim=simulate_turn(spec,constants)
    ts=[0.0]+[s.t_ms/1000 for s in sim.samples]
    angles=[0.0]+[s.theta_deg for s in sim.samples]
    duration=sim.profile.t_total_s
    # Fit angular *shape* with nuisance heading bias and gain. Camera yaw is
    # used only for registration, never as trusted body heading training data.
    times=[r[0] for r in observed]; ys=[r[3] for r in observed]
    ym=statistics.mean(ys)
    lo=times[0]; hi=times[-1]-duration
    if hi<=lo: raise ValueError("recording is shorter than nominal turn core")
    candidates=[]
    count=max(1,int((hi-lo)/.002))
    for j in range(count+1):
        start=lo+(hi-lo)*j/count
        xs=[_interp(ts,angles,t-start) for t in times]
        xm=statistics.mean(xs)
        var=sum((x-xm)**2 for x in xs)
        if var<1e-9: continue
        gain=sum((x-xm)*(y-ym) for x,y in zip(xs,ys))/var
        if not .5<gain<1.5: continue
        error=sum((y-ym-gain*(x-xm))**2 for x,y in zip(xs,ys))/len(xs)
        candidates.append((error,start,gain))
    if not candidates: raise ValueError("unable to align nominal turn shape")
    best=min(candidates); start=best[1]; end=start+duration+.08
    if start-.03<times[0] or end>times[-1]: raise ValueError("insufficient approach or post-turn coverage")
    near=[c[1] for c in candidates if c[0] <= best[0]+max(1.0,.1*best[0])]
    samples=_normalize(rows,start,end,heading,"board",False)
    return samples,{"sample_mode":"core", "core_duration_s":duration,
        "core_start_s":start,"segment_end_s":end,"initial_course_heading_deg":heading,
        "heading_fit_p95_mm":residual,"alignment_method":"nominal-angle-template-with-free-bias-and-gain",
        "alignment_rmse_deg":math.sqrt(best[0]),"alignment_yaw_gain":best[2],
        "alignment_uncertainty_s":max(.004,(max(near)-min(near))/2),
        "alignment_limitation":"absolute yaw delay is confounded with inferred onset; position-derived course is not body heading"}


def _historical(data: dict, root: Path) -> Dataset:
    runs=[]; warnings=list(data.get("known_limitations",[])); excluded=[]; seen={}
    constants=Constants(1.2,2200.0)
    for item in data['datasets']:
        if item.get('contact') is not False:
            excluded.append({'id':item['id'],'reason':'contact or unknown contact status'}); continue
        try:
            report_path=_resolve(item['report'],root)
            if sha256_file(report_path)!=item['report_sha256']: raise ValueError("report SHA256 mismatch")
            report=json.loads(report_path.read_text())
            records=[]
            for trial in report['trials']:
                p=_resolve(trial['path'],root)
                relative='sessions/'+trial['path'].split('/sessions/',1)[1]
                records.append(f'{sha256_file(p)}  {relative}\n')
            digest=hashlib.sha256(''.join(sorted(records)).encode()).hexdigest()
            if digest!=item.get('trajectory_set_sha256'):
                raise ValueError('trajectory-set SHA256 mismatch')
            values=item['params']
            param_text='|'.join(format(values[k],'.9g') for k in (
                'velocity_mm_s','alpha_deg_s2','configured_angle_deg','effective_angle_deg','dist_in_mm','dist_out_mm'))
            if hashlib.sha256(param_text.encode()).hexdigest()!=item.get('params_record_sha256'):
                raise ValueError('parameter-record SHA256 mismatch')
            selection={**data.get('turn',{}),**report.get('analysis',{}).get('turn_selection',{})}
            original=dict(item['params']); core={**original,'dist_in_mm':0,'dist_out_mm':0}
            spec=_spec(core,selection,item['id']); regime='zero-entry' if original['dist_in_mm']==0 else 'positive-entry'
            group=_parameter_group(original,spec,constants,data.get('machine','unknown'),regime)
            for number,trial in enumerate(report['trials'],1):
                run_id=f"{item['id']}/{number}"
                try:
                    path=_resolve(trial['path'],root); sha=sha256_file(path)
                    capture=next((part for part in path.parts if re.search(r'_manual-\d+-(?:one-shot-)?run\d+',part)),sha)
                    if sha in seen or capture in seen:
                        excluded.append({'id':run_id,'reason':f'duplicate capture of {seen.get(capture,seen.get(sha))}'}); continue
                    rows,quality=_read_csv(path)
                    samples,alignment=_historical_segment(rows,trial,spec,constants)
                    run_warnings=['historical video is diagnostic only; calibration/camera integrity contract absent',
                                  'inferred core alignment cannot identify absolute yaw delay; heading is excluded from fitting']
                    provenance={'machine':data.get('machine'),'source_csv':str(path),'csv_sha256':sha,
                        'report':str(report_path),'report_sha256':item['report_sha256'],
                        'params_source_commit':item.get('params_source_commit'),'params_provenance':item.get('params_provenance'),
                        'original_params':original,'execution_regime':regime,'parameter_record_sha256':item.get('params_record_sha256'),
                        'coordinate_method':item.get('coordinate_method'),'contact':False,'sample_mode':'core',
                        'constants_source':'historical params.h/f413_path_run.h: rounding 1.2, omega cap 2200',
                        'trajectory_set_verified':True,'capture_id':capture,**quality,**alignment}
                    runs.append(MeasuredRun(run_id,group,spec,constants,samples,provenance,run_warnings,True)); seen[sha]=run_id; seen[capture]=run_id
                except (ValueError,KeyError,OSError) as exc: excluded.append({'id':run_id,'reason':str(exc)})
        except (ValueError,KeyError,OSError) as exc: excluded.append({'id':item.get('id','unknown'),'reason':str(exc)})
    warnings.append('Historical catalog covers one 500 mm/s right D135 turn; modes/speeds/machines outside this scope are unvalidated.')
    return Dataset(runs,warnings,excluded)


def _explicit(data: dict, root: Path) -> Dataset:
    runs=[]; excluded=[]; warnings=[]; seen={}
    for item in data['runs']:
        run_id=item.get('id','unknown')
        try:
            if item.get('contact') is not False: raise ValueError('contact must be explicitly false')
            path=_resolve(item['csv'],root); sha=sha256_file(path)
            if item.get('csv_sha256') and item['csv_sha256'] != sha: raise ValueError('CSV SHA256 mismatch')
            if sha in seen: raise ValueError(f'duplicate trajectory of {seen[sha]}')
            params=item['params']; spec=_spec(params,item.get('selection',{}),run_id)
            constants=_constants(item.get('constants',data.get('constants',{})))
            rows,quality=_read_csv(path,item.get('columns'))
            frame=item.get('coordinate_frame',data.get('coordinate_frame','local'))
            segment=item.get('segment',{})
            start=_finite(segment.get('start_s',rows[0][0]),'start_s'); end=_finite(segment.get('end_s',rows[-1][0]),'end_s')
            heading=segment.get('heading_deg',0 if frame=='local' else None)
            if heading is None: raise ValueError('board coordinates require explicit segment.heading_deg')
            origin=None
            if 'origin_x_mm' in segment or 'origin_y_mm' in segment:
                origin=(_finite(segment['origin_x_mm'],'origin_x_mm'),_finite(segment['origin_y_mm'],'origin_y_mm'))
            trusted=item.get('heading_trusted',False)
            samples=_normalize(rows,start,end,_finite(heading,'heading_deg'),frame,trusted,origin)
            mode=item.get('sample_mode','full')
            if mode not in ('full','core'): raise ValueError('sample_mode must be core or full')
            if mode=='core' and (spec.dist_in_mm or spec.dist_out_mm): raise ValueError('core sample_mode requires zero spec offsets')
            original_params=item.get('original_params',params)
            original_spec=_spec(original_params,item.get('selection',{}),run_id)
            if (original_spec.velocity_mm_s,original_spec.alpha_deg_s2,original_spec.signed_angle_deg) != (spec.velocity_mm_s,spec.alpha_deg_s2,spec.signed_angle_deg):
                raise ValueError('original_params must match core speed, alpha and effective angle')
            provenance={**data.get('provenance',{}),**item.get('provenance',{}),**quality,
                'machine':item.get('machine',data.get('machine')),'source_csv':str(path),'csv_sha256':sha,
                'original_params':original_params,'contact':False,'sample_mode':mode,'coordinate_frame':frame,
                'heading_trusted':trusted,'segment':segment}
            regime=item.get('execution_regime','zero-entry' if original_spec.dist_in_mm==0 else 'positive-entry')
            if regime not in ('zero-entry','positive-entry') or (regime=='zero-entry') != (original_spec.dist_in_mm==0):
                raise ValueError('execution_regime must match original_params entry offset')
            provenance['execution_regime']=regime
            canonical=_parameter_group(original_params,spec,constants,str(provenance['machine']),regime)
            # Parameter duplicates must stay together even when a manifest gives
            # each repetition a different label. Session labels are provenance.
            group=canonical
            if item.get('group_id'): provenance['declared_group_id']=item['group_id']
            run_warnings=[]
            required=('machine','fw_git_sha','calibration_sha256')
            missing=[k for k in required if not provenance.get(k)]
            diagnostic=item.get('diagnostic_only',True) or bool(missing) or not item.get('csv_sha256')
            if missing: run_warnings.append('missing provenance: '+', '.join(missing))
            if not item.get('csv_sha256'): run_warnings.append('CSV content hash was not bound in manifest')
            if quality['valid_fraction']<.99: run_warnings.append('valid pose fraction below 99%')
            runs.append(MeasuredRun(run_id,group,spec,constants,samples,provenance,run_warnings,diagnostic)); seen[sha]=run_id
        except (ValueError,KeyError,OSError,TypeError) as exc: excluded.append({'id':run_id,'reason':str(exc)})
    return Dataset(runs,warnings,excluded)


def load_dataset(manifest_path: Path | str, data_root: Path | str | None = None) -> Dataset:
    path=Path(manifest_path).resolve(); data=json.loads(path.read_text())
    schema=data.get('schema')
    root=Path(data_root).resolve() if data_root else path.parent
    if schema=='nightfall_turn_calibration_manifest_v1':
        if data_root is None and len(path.parents)>2 and path.parent.name=='data': root=path.parents[3]
        return _historical(data,root)
    if schema=='nightfall_turn_dataset_v1': return _explicit(data,root)
    raise ValueError(f'unsupported dataset schema: {schema}')


def summarize(dataset: Dataset) -> dict:
    return {'runs':len(dataset.runs),'parameter_groups':len({r.group_id for r in dataset.runs}),
            'diagnostic_only_runs':sum(r.diagnostic_only for r in dataset.runs),
            'samples':sum(len(r.samples) for r in dataset.runs),
            'execution_regimes':{regime:sum(r.provenance.get('execution_regime')==regime for r in dataset.runs)
                for regime in sorted({r.provenance.get('execution_regime','unknown') for r in dataset.runs})},
            'warnings':dataset.warnings,'exclusions':dataset.exclusions}
