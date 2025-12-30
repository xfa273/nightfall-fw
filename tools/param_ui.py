#!/usr/bin/env python3

import argparse
import csv
import json
import os
import shutil
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict, List, Tuple
from urllib.parse import parse_qs, urlparse


TURN_FLOAT_FIELDS = [
    "velocity_turn90",
    "alpha_turn90",
    "acceleration_turn",
    "dist_offset_in",
    "dist_offset_out",
    "val_offset_in",
    "angle_turn_90",
    "dist_wall_end",
    "velocity_l_turn_90",
    "alpha_l_turn_90",
    "angle_l_turn_90",
    "dist_l_turn_in_90",
    "dist_l_turn_out_90",
    "velocity_l_turn_180",
    "alpha_l_turn_180",
    "angle_l_turn_180",
    "dist_l_turn_in_180",
    "dist_l_turn_out_180",
    "velocity_turn45in",
    "alpha_turn45in",
    "angle_turn45in",
    "dist_turn45in_in",
    "dist_turn45in_out",
    "velocity_turn45out",
    "alpha_turn45out",
    "angle_turn45out",
    "dist_turn45out_in",
    "dist_turn45out_out",
    "velocity_turnV90",
    "alpha_turnV90",
    "angle_turnV90",
    "dist_turnV90_in",
    "dist_turnV90_out",
    "velocity_turn135in",
    "alpha_turn135in",
    "angle_turn135in",
    "dist_turn135in_in",
    "dist_turn135in_out",
    "velocity_turn135out",
    "alpha_turn135out",
    "angle_turn135out",
    "dist_turn135out_in",
    "dist_turn135out_out",
    "accel_switch_velocity",
]

TURN_UINT16_FIELDS = [
    "fan_power",
    "wall_end_thr_r_high",
    "wall_end_thr_r_low",
    "wall_end_thr_l_high",
    "wall_end_thr_l_low",
]

TURN_INT_FIELDS = [
    "makepath_type_case3",
    "makepath_type_case47",
]

CASE_FLOAT_FIELDS = [
    "acceleration_straight",
    "acceleration_straight_dash",
    "velocity_straight",
    "acceleration_d_straight",
    "acceleration_d_straight_dash",
    "velocity_d_straight",
    "kp_wall",
    "kp_diagonal",
]

CASE_FIELDS = [
    *CASE_FLOAT_FIELDS,
    "solver_profile",
]

SOLVER_PROFILE_OPTIONS = [
    "SOLVER_PROFILE_STANDARD",
    "SOLVER_PROFILE_STRAIGHT_STRONG",
    "SOLVER_PROFILE_STRAIGHT_WEAK",
]


def _repo_root() -> str:
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _params_dir() -> str:
    return os.path.join(_repo_root(), "params")


def _variants() -> List[str]:
    root = _params_dir()
    if not os.path.isdir(root):
        return []
    out: List[str] = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if not os.path.isdir(d):
            continue
        if os.path.isfile(os.path.join(d, "shortest_turn_bank.csv")) and os.path.isfile(
            os.path.join(d, "shortest_case_params.csv")
        ):
            out.append(name)
    return out


def _csv_path(variant: str, kind: str) -> str:
    if kind == "turn_bank":
        return os.path.join(_params_dir(), variant, "shortest_turn_bank.csv")
    if kind == "case_params":
        return os.path.join(_params_dir(), variant, "shortest_case_params.csv")
    raise ValueError(f"unknown kind: {kind}")


def _read_csv(path: str) -> Tuple[List[str], List[Dict[str, str]]]:
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        if r.fieldnames is None:
            raise RuntimeError("CSV header is missing")
        rows: List[Dict[str, str]] = []
        for row in r:
            # Skip empty lines (e.g. trailing blank line from spreadsheet editors)
            if (row.get("mode") or "").strip() == "":
                continue
            rows.append({k: (row.get(k) or "") for k in r.fieldnames})
        return list(r.fieldnames), rows


def _write_csv(path: str, header: List[str], rows: List[Dict[str, str]]) -> None:
    ts = time.strftime("%Y%m%d_%H%M%S")
    bak = f"{path}.{ts}.bak"
    shutil.copy2(path, bak)

    with open(path, "w", newline="\n") as f:
        w = csv.DictWriter(f, fieldnames=header, lineterminator="\n")
        w.writeheader()
        for row in rows:
            w.writerow({k: row.get(k, "") for k in header})


def _validate_and_normalize_row(kind: str, header: List[str], row: Dict[str, Any]) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for k in header:
        if k in row:
            out[k] = str(row[k])

    if kind == "turn_bank":
        if "mode" not in out or "bank" not in out:
            raise ValueError("mode/bank is required")
        int(out["mode"], 0)
        int(out["bank"], 0)

        for k in TURN_FLOAT_FIELDS:
            if k not in out:
                raise ValueError(f"missing field: {k}")
            float(out[k])

        for k in TURN_UINT16_FIELDS:
            if k not in out:
                raise ValueError(f"missing field: {k}")
            v = int(out[k], 0)
            if not (0 <= v <= 0xFFFF):
                raise ValueError(f"uint16 out of range: {k}={v}")

        for k in TURN_INT_FIELDS:
            if k not in out:
                raise ValueError(f"missing field: {k}")
            int(out[k], 0)

    elif kind == "case_params":
        if "mode" not in out or "case" not in out:
            raise ValueError("mode/case is required")
        int(out["mode"], 0)
        int(out["case"], 0)

        for k in CASE_FLOAT_FIELDS:
            if k not in out:
                raise ValueError(f"missing field: {k}")
            float(out[k])

        if "solver_profile" not in out:
            raise ValueError("missing field: solver_profile")
        sp = out["solver_profile"].strip()
        if sp in SOLVER_PROFILE_OPTIONS:
            pass
        else:
            int(sp, 0)

    else:
        raise ValueError(f"unknown kind: {kind}")

    return out


def _turn_bank_meta() -> Dict[str, Any]:
    fields: List[Dict[str, Any]] = []

    def add(name: str, label: str, unit: str, typ: str, section: str = "") -> None:
        fields.append({"name": name, "label": label, "unit": unit, "type": typ, "section": section})

    add("mode", "モード", "", "int", "基本")
    add("bank", "bank", "", "int", "基本")

    add("velocity_turn90", "小回り90 速度", "mm/s", "float", "小回り90°")
    add("alpha_turn90", "小回り90 alpha", "deg/s^2", "float", "小回り90°")
    add("acceleration_turn", "ターン加速度", "mm/s^2", "float", "小回り90°")
    add("dist_offset_in", "オフセット in", "mm", "float", "小回り90°")
    add("dist_offset_out", "オフセット out", "mm", "float", "小回り90°")
    add("val_offset_in", "位置補正値", "", "float", "小回り90°")
    add("angle_turn_90", "小回り90 角度", "deg", "float", "小回り90°")
    add("dist_wall_end", "壁切れ後 直進距離", "mm", "float", "壁切れ")

    add("velocity_l_turn_90", "大回り90 速度", "mm/s", "float", "大回り90°")
    add("alpha_l_turn_90", "大回り90 alpha", "deg/s^2", "float", "大回り90°")
    add("angle_l_turn_90", "大回り90 角度", "deg", "float", "大回り90°")
    add("dist_l_turn_in_90", "大回り90 入り", "mm", "float", "大回り90°")
    add("dist_l_turn_out_90", "大回り90 出", "mm", "float", "大回り90°")

    add("velocity_l_turn_180", "大回り180 速度", "mm/s", "float", "大回り180°")
    add("alpha_l_turn_180", "大回り180 alpha", "deg/s^2", "float", "大回り180°")
    add("angle_l_turn_180", "大回り180 角度", "deg", "float", "大回り180°")
    add("dist_l_turn_in_180", "大回り180 入り", "mm", "float", "大回り180°")
    add("dist_l_turn_out_180", "大回り180 出", "mm", "float", "大回り180°")

    add("velocity_turn45in", "45in 速度", "mm/s", "float", "斜め45°(in)")
    add("alpha_turn45in", "45in alpha", "deg/s^2", "float", "斜め45°(in)")
    add("angle_turn45in", "45in 角度", "deg", "float", "斜め45°(in)")
    add("dist_turn45in_in", "45in 入り", "mm", "float", "斜め45°(in)")
    add("dist_turn45in_out", "45in 出", "mm", "float", "斜め45°(in)")

    add("velocity_turn45out", "45out 速度", "mm/s", "float", "斜め45°(out)")
    add("alpha_turn45out", "45out alpha", "deg/s^2", "float", "斜め45°(out)")
    add("angle_turn45out", "45out 角度", "deg", "float", "斜め45°(out)")
    add("dist_turn45out_in", "45out 入り", "mm", "float", "斜め45°(out)")
    add("dist_turn45out_out", "45out 出", "mm", "float", "斜め45°(out)")

    add("velocity_turnV90", "V90 速度", "mm/s", "float", "斜めV90°")
    add("alpha_turnV90", "V90 alpha", "deg/s^2", "float", "斜めV90°")
    add("angle_turnV90", "V90 角度", "deg", "float", "斜めV90°")
    add("dist_turnV90_in", "V90 入り", "mm", "float", "斜めV90°")
    add("dist_turnV90_out", "V90 出", "mm", "float", "斜めV90°")

    add("velocity_turn135in", "135in 速度", "mm/s", "float", "斜め135°(in)")
    add("alpha_turn135in", "135in alpha", "deg/s^2", "float", "斜め135°(in)")
    add("angle_turn135in", "135in 角度", "deg", "float", "斜め135°(in)")
    add("dist_turn135in_in", "135in 入り", "mm", "float", "斜め135°(in)")
    add("dist_turn135in_out", "135in 出", "mm", "float", "斜め135°(in)")

    add("velocity_turn135out", "135out 速度", "mm/s", "float", "斜め135°(out)")
    add("alpha_turn135out", "135out alpha", "deg/s^2", "float", "斜め135°(out)")
    add("angle_turn135out", "135out 角度", "deg", "float", "斜め135°(out)")
    add("dist_turn135out_in", "135out 入り", "mm", "float", "斜め135°(out)")
    add("dist_turn135out_out", "135out 出", "mm", "float", "斜め135°(out)")

    add("accel_switch_velocity", "加速度切替速度", "mm/s", "float", "その他")

    add("fan_power", "ファン出力", "0-1000", "uint16", "その他")
    add("wall_end_thr_r_high", "壁切れ(R) High", "", "uint16", "その他")
    add("wall_end_thr_r_low", "壁切れ(R) Low", "", "uint16", "その他")
    add("wall_end_thr_l_high", "壁切れ(L) High", "", "uint16", "その他")
    add("wall_end_thr_l_low", "壁切れ(L) Low", "", "uint16", "その他")

    add("makepath_type_case3", "makePath type(case3)", "", "int", "その他")
    add("makepath_type_case47", "makePath type(case4-7)", "", "int", "その他")

    return {"kind": "turn_bank", "key": ["mode", "bank"], "fields": fields}


def _case_params_meta() -> Dict[str, Any]:
    fields: List[Dict[str, Any]] = []

    def add(name: str, label: str, unit: str, typ: str, options: List[str] = None) -> None:
        d: Dict[str, Any] = {"name": name, "label": label, "unit": unit, "type": typ}
        if options is not None:
            d["options"] = options
        fields.append(d)

    add("mode", "モード", "", "int")
    add("case", "case", "", "int")

    add("acceleration_straight", "直線 加速度", "mm/s^2", "float")
    add("acceleration_straight_dash", "直線 2段加速度", "mm/s^2", "float")
    add("velocity_straight", "直線 速度", "mm/s", "float")

    add("acceleration_d_straight", "斜め 加速度", "mm/s^2", "float")
    add("acceleration_d_straight_dash", "斜め 2段加速度", "mm/s^2", "float")
    add("velocity_d_straight", "斜め 速度", "mm/s", "float")

    add("kp_wall", "壁制御 kp", "", "float")
    add("kp_diagonal", "斜め壁制御 kp", "", "float")

    add("solver_profile", "solver_profile", "", "enum", SOLVER_PROFILE_OPTIONS)

    return {"kind": "case_params", "key": ["mode", "case"], "fields": fields}


INDEX_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Nightfall Params UI</title>
  <style>
    body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Helvetica,Arial,sans-serif;margin:16px;}
    .row{display:flex;gap:12px;flex-wrap:wrap;align-items:center;margin-bottom:12px;}
    select,input{padding:6px 8px;font-size:14px;}
    button{padding:8px 12px;font-size:14px;cursor:pointer;}
    .status{margin-left:8px;color:#333;}
    .grid{display:grid;grid-template-columns: 260px 1fr 120px;gap:8px 12px;max-width:900px;}
    .grid .h{font-weight:600;color:#222;}
    .muted{color:#666;font-size:12px;}
    .footer{margin-top:16px;color:#666;font-size:12px;}
    .err{color:#b00020;}
    .ok{color:#0a7a22;}
    .section{grid-column:1 / span 3;border-top:1px solid #ddd;margin-top:14px;padding-top:10px;font-weight:700;color:#222;}
    .section.first{border-top:none;margin-top:0;padding-top:0;}
  </style>
</head>
<body>
<h2>Nightfall パラメータUI</h2>

<div class="row">
  <label>variant <select id="variant"></select></label>
  <label>対象 <select id="kind">
    <option value="turn_bank">shortest_turn_bank</option>
    <option value="case_params">shortest_case_params</option>
  </select></label>
  <label>mode <select id="mode"></select></label>
  <label id="subLabel"><span id="subLabelText">bank</span> <select id="sub"></select></label>
  <button id="reload">読み込み</button>
  <button id="save">保存</button>
  <span id="status" class="status"></span>
</div>

<div id="form" class="grid"></div>

<div class="footer">
  <div class="muted">保存すると同一フォルダに .bak が作成されます。ビルドは別途実行してください。</div>
</div>

<script>
let meta = null;
let header = [];
let rows = [];

function setStatus(msg, cls){
  const el = document.getElementById('status');
  el.textContent = msg;
  el.className = 'status ' + (cls||'');
}

async function apiGet(path){
  const r = await fetch(path);
  if(!r.ok){
    throw new Error(await r.text());
  }
  return await r.json();
}

async function apiPost(path, body){
  const r = await fetch(path, {method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body)});
  if(!r.ok){
    throw new Error(await r.text());
  }
  return await r.json();
}

function getKey(row){
  if(meta.kind === 'turn_bank') return `${row.mode}|${row.bank}`;
  return `${row.mode}|${row.case}`;
}

function getCurrentSelector(){
  const variant = document.getElementById('variant').value;
  const kind = document.getElementById('kind').value;
  const mode = document.getElementById('mode').value;
  const sub = document.getElementById('sub').value;
  return {variant, kind, mode, sub};
}

function buildModeAndSubSelectors(){
  const modeSel = document.getElementById('mode');
  const subSel = document.getElementById('sub');
  const subLabelText = document.getElementById('subLabelText');

  modeSel.innerHTML = '';
  subSel.innerHTML = '';

  const modes = [2,3,4,5,6,7];
  for(const m of modes){
    const o = document.createElement('option');
    o.value = String(m);
    o.textContent = String(m);
    modeSel.appendChild(o);
  }

  if(meta.kind === 'turn_bank'){
    if(subLabelText){ subLabelText.textContent = 'bank'; }
    for(let b=0;b<5;b++){
      const o = document.createElement('option');
      o.value = String(b);
      o.textContent = String(b);
      subSel.appendChild(o);
    }
  }else{
    if(subLabelText){ subLabelText.textContent = 'case'; }
    const mode = parseInt(modeSel.value,10);
    const max = (mode===7)?5:9;
    for(let c=1;c<=max;c++){
      const o = document.createElement('option');
      o.value = String(c);
      o.textContent = String(c);
      subSel.appendChild(o);
    }
  }
}

function rebuildCaseCountOnModeChange(){
  if(!meta || meta.kind !== 'case_params') return;
  const mode = parseInt(document.getElementById('mode').value,10);
  const subSel = document.getElementById('sub');
  const prev = subSel.value;
  subSel.innerHTML = '';
  const max = (mode===7)?5:9;
  for(let c=1;c<=max;c++){
    const o = document.createElement('option');
    o.value = String(c);
    o.textContent = String(c);
    subSel.appendChild(o);
  }
  if(prev && parseInt(prev,10) <= max){
    subSel.value = prev;
  }
}

function findRow(mode, sub){
  for(const r of rows){
    if(meta.kind === 'turn_bank'){
      if(String(r.mode)===String(mode) && String(r.bank)===String(sub)) return r;
    }else{
      if(String(r.mode)===String(mode) && String(r.case)===String(sub)) return r;
    }
  }
  return null;
}

function renderForm(row){
  const form = document.getElementById('form');
  form.innerHTML = '';

  const h1 = document.createElement('div'); h1.textContent='項目'; h1.className='h';
  const h2 = document.createElement('div'); h2.textContent='値'; h2.className='h';
  const h3 = document.createElement('div'); h3.textContent='単位'; h3.className='h';
  form.appendChild(h1); form.appendChild(h2); form.appendChild(h3);

  let lastSection = null;
  for(const f of meta.fields){
    const sec = (f.section || '').trim();
    if(sec && sec !== lastSection){
      const s = document.createElement('div');
      s.textContent = sec;
      s.className = 'section' + (lastSection === null ? ' first' : '');
      form.appendChild(s);
      lastSection = sec;
    }
    const name = f.name;

    const label = document.createElement('div');
    label.textContent = f.label;
    form.appendChild(label);

    let input;
    if(f.type === 'enum'){
      input = document.createElement('select');
      for(const opt of f.options){
        const o = document.createElement('option');
        o.value = opt;
        o.textContent = opt;
        input.appendChild(o);
      }
      const cur = (row && row[name]) ? row[name] : f.options[0];
      input.value = cur;
    }else{
      input = document.createElement('input');
      input.value = (row && row[name] !== undefined) ? row[name] : '';
      input.type = (f.type === 'float') ? 'number' : 'number';
      input.step = (f.type === 'float') ? 'any' : '1';
    }

    input.dataset.field = name;

    if(name === 'mode' || name === 'bank' || name === 'case'){
      input.disabled = true;
    }

    form.appendChild(input);

    const unit = document.createElement('div');
    unit.textContent = f.unit || '';
    unit.className='muted';
    form.appendChild(unit);
  }
}

function collectForm(){
  const out = {};
  const inputs = document.querySelectorAll('#form input, #form select');
  inputs.forEach((el)=>{
    out[el.dataset.field] = el.value;
  });
  return out;
}

async function loadAll(){
  const {variant, kind} = getCurrentSelector();
  setStatus('読み込み中...', '');
  const m = await apiGet(`/api/meta?kind=${encodeURIComponent(kind)}`);
  meta = m;
  buildModeAndSubSelectors();
  const d = await apiGet(`/api/data?variant=${encodeURIComponent(variant)}&kind=${encodeURIComponent(kind)}`);
  header = d.header;
  rows = d.rows;
  setStatus('読み込み完了', 'ok');
  renderSelected();
}

function renderSelected(){
  const {mode, sub} = getCurrentSelector();
  const r = findRow(mode, sub);
  if(!r){
    setStatus('該当行が見つかりません', 'err');
    const blank = {mode:String(mode)};
    if(meta.kind === 'turn_bank') blank.bank = String(sub);
    else blank.case = String(sub);
    renderForm(blank);
    return;
  }
  renderForm(r);
}

async function saveCurrent(){
  const {variant, kind, mode, sub} = getCurrentSelector();
  const row = collectForm();
  row.mode = String(mode);
  if(kind === 'turn_bank') row.bank = String(sub);
  else row.case = String(sub);

  setStatus('保存中...', '');
  await apiPost('/api/save', {variant, kind, row});
  setStatus('保存完了', 'ok');
  await loadAll();
}

async function init(){
  const vs = await apiGet('/api/variants');
  const vsel = document.getElementById('variant');
  vsel.innerHTML = '';
  for(const v of vs.variants){
    const o = document.createElement('option');
    o.value = v;
    o.textContent = v;
    vsel.appendChild(o);
  }

  document.getElementById('kind').addEventListener('change', async ()=>{
    await loadAll();
  });

  document.getElementById('mode').addEventListener('change', ()=>{
    rebuildCaseCountOnModeChange();
    renderSelected();
  });

  document.getElementById('sub').addEventListener('change', ()=>{
    renderSelected();
  });

  document.getElementById('reload').addEventListener('click', async ()=>{
    await loadAll();
  });

  document.getElementById('save').addEventListener('click', async ()=>{
    try{
      await saveCurrent();
    }catch(e){
      setStatus(String(e), 'err');
    }
  });

  await loadAll();
}

init().catch((e)=>{
  setStatus(String(e), 'err');
});
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def _send_json(self, obj: Any, code: int = 200) -> None:
        b = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _send_text(self, text: str, code: int = 200, content_type: str = "text/plain; charset=utf-8") -> None:
        b = text.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self) -> None:
        u = urlparse(self.path)
        if u.path == "/":
            self._send_text(INDEX_HTML, 200, "text/html; charset=utf-8")
            return

        if u.path == "/api/variants":
            self._send_json({"variants": _variants()})
            return

        if u.path == "/api/meta":
            q = parse_qs(u.query)
            kind = (q.get("kind") or [""])[0]
            if kind == "turn_bank":
                self._send_json(_turn_bank_meta())
                return
            if kind == "case_params":
                self._send_json(_case_params_meta())
                return
            self._send_text("unknown kind", 400)
            return

        if u.path == "/api/data":
            q = parse_qs(u.query)
            variant = (q.get("variant") or [""])[0]
            kind = (q.get("kind") or [""])[0]

            if variant not in _variants():
                self._send_text("invalid variant", 400)
                return

            try:
                p = _csv_path(variant, kind)
                header, rows = _read_csv(p)
                self._send_json({"header": header, "rows": rows})
            except Exception as e:
                self._send_text(str(e), 500)
            return

        self._send_text("not found", 404)

    def do_POST(self) -> None:
        u = urlparse(self.path)
        if u.path != "/api/save":
            self._send_text("not found", 404)
            return

        try:
            n = int(self.headers.get("Content-Length", "0"))
        except Exception:
            n = 0
        body = self.rfile.read(n)

        try:
            req = json.loads(body.decode("utf-8"))
        except Exception:
            self._send_text("invalid json", 400)
            return

        variant = str(req.get("variant", ""))
        kind = str(req.get("kind", ""))
        row = req.get("row")

        if variant not in _variants():
            self._send_text("invalid variant", 400)
            return
        if kind not in ("turn_bank", "case_params"):
            self._send_text("invalid kind", 400)
            return
        if not isinstance(row, dict):
            self._send_text("row must be object", 400)
            return

        try:
            path = _csv_path(variant, kind)
            header, rows = _read_csv(path)
            norm = _validate_and_normalize_row(kind, header, row)

            key0 = "mode"
            key1 = "bank" if kind == "turn_bank" else "case"
            k0 = str(int(norm[key0], 0))
            k1 = str(int(norm[key1], 0))

            updated = False
            for i, r in enumerate(rows):
                if str(int(r.get(key0, "0"), 0)) == k0 and str(int(r.get(key1, "0"), 0)) == k1:
                    rows[i] = {**r, **norm}
                    updated = True
                    break

            if not updated:
                rows.append({k: norm.get(k, "") for k in header})

            seen = set()
            for r in rows:
                kk0 = str(int(r.get(key0, "0"), 0))
                kk1 = str(int(r.get(key1, "0"), 0))
                kk = (kk0, kk1)
                if kk in seen:
                    raise ValueError("duplicate key after save")
                seen.add(kk)

            rows.sort(key=lambda rr: (int(rr.get(key0, "0"), 0), int(rr.get(key1, "0"), 0)))

            _write_csv(path, header, rows)
            self._send_json({"ok": True})
        except Exception as e:
            self._send_text(str(e), 400)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()

    httpd = HTTPServer((args.host, args.port), Handler)
    print(f"open: http://{args.host}:{args.port}/")
    httpd.serve_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
