#!/usr/bin/env python3
"""viewer.py - 走行と地図生成の过程を見せる自己完結 HTML UI を生成する.

    ./viewer.py --maze 16MM2017CX --presets current,fixed_map,fixed_gain,topmouse \
        --out ../../out/exploration_sim/viewer_16MM2017CX.html

生成物は外部依存ゼロの HTML（canvas + JS）で、`open` でそのまま見られる。
左右のペインで「現状ソフト」と「改善版」を同じ時間軸で同期再生し、
壁を 4 状態（正しい既知壁 / 誤認壁 / 見落としている壁 / 未知）で色分けする。
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import motion as motion_mod
import trace_sim
from maze import Maze
import run_exploration_sim as cli

TEMPLATE = r"""<!doctype html>
<html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>
<style>
:root{--bg:#141414;--fg:#eaeaea;--mut:#9aa0a6;--ln:#333;--wallOk:#f2f2f2;--wallBad:#ff5c5c;
--wallMiss:#3f7fc4;--mouse:#ffd166;--trail:#4cc9f0}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font:13px/1.5 ui-monospace,SFMono-Regular,Menlo,monospace}
header{padding:10px 14px;border-bottom:1px solid var(--ln);display:flex;flex-wrap:wrap;gap:10px;align-items:center}
h1{font-size:14px;margin:0 12px 0 0}
select,button{background:#221f1f;color:var(--fg);border:1px solid var(--ln);border-radius:4px;padding:4px 8px;font:inherit}
button:hover{border-color:#666}
input[type=range]{flex:1;min-width:180px}
main{display:flex;flex-wrap:wrap;gap:14px;padding:14px}
section{border:1px solid var(--ln);border-radius:8px;padding:10px;background:#181818;flex:1 1 460px;min-width:380px}
section h2{margin:0 0 4px;font-size:13px}
section p.desc{margin:0 0 8px;color:var(--mut);font-size:11px;min-height:2.4em}
canvas{display:block;background:#101010;border-radius:6px;width:100%;image-rendering:pixelated}
.metrics{display:flex;flex-wrap:wrap;gap:10px;margin-top:8px;font-size:12px}
.metrics b{color:#fff;font-weight:600}
.legend{display:flex;flex-wrap:wrap;gap:14px;padding:0 16px 16px;color:var(--mut);font-size:12px;align-items:center}
.sw{display:inline-block;width:22px;height:3px;vertical-align:middle;margin-right:5px;border-radius:2px}
label{color:var(--mut)}
.kbd{color:var(--mut);font-size:11px}
.warn{color:#ffb3b3}
footer{padding:0 16px 20px;color:var(--mut);font-size:11px}
</style></head><body>
<header>
  <h1>__H1__</h1>
  <label>左 <select id="selA"></select></label>
  <label>右 <select id="selB"></select></label>
  <button id="play">▶ 再生</button>
  <button id="back">|◀</button><button id="fwd">▶|</button>
  <button id="nextPhantom">次の phantom</button>
  <button id="reset">⟲ 最初</button>
  <select id="speed"><option value="0.5">0.5x</option><option value="1" selected>1x</option>
    <option value="2">2x</option><option value="4">4x</option><option value="8">8x</option></select>
  <input type="range" id="scrub" min="0" max="1000" value="0">
  <span id="clock" style="min-width:96px">t = 0.00 s</span>
  <label><input type="checkbox" id="truth" checked> 真の迷路</label>
  <label><input type="checkbox" id="order"> 訪問順</label>
  <label><input type="checkbox" id="trail" checked> 軌跡</label>
</header>
<main>
  <section><h2 id="labA"></h2><p class="desc" id="descA"></p>
    <canvas id="cA"></canvas><div class="metrics" id="mA"></div>
    <canvas id="tA" height="26" style="margin-top:8px"></canvas></section>
  <section><h2 id="labB"></h2><p class="desc" id="descB"></p>
    <canvas id="cB"></canvas><div class="metrics" id="mB"></div>
    <canvas id="tB" height="26" style="margin-top:8px"></canvas></section>
</main>
<div class="legend">
  <span><i class="sw" style="background:var(--wallOk)"></i>既知の壁（正しい）</span>
  <span><i class="sw" style="background:var(--wallBad)"></i>誤って壁ありと信じている</span>
  <span><i class="sw" style="background:var(--wallMiss);opacity:.9"></i>実在するが見落としている壁</span>
  <span><i class="sw" style="background:#555"></i>真の迷路（参考）</span>
  <span><i class="sw" style="background:var(--trail)"></i>走行軌跡</span>
  <span style="color:var(--mouse)">▲ マウス</span>
  <span class="warn">赤いリング = 未知の壁への進入 (phantom)</span>
  <span><i class="sw" style="background:#f2f2f2"></i>タイムラインの白線 = 係員が機体を開始位置へ戻した所（ラン切替）</span>
  <span class="kbd">space 再生/停止 / ←→ ±0.2s / ↑↓ 速度</span>
</div>
<footer>__FOOTER__</footer>
<script>
const DATA = __PAYLOAD__;
const DIRS=[[0,1],[1,0],[0,-1],[-1,0]];      // N,E,S,W
const WBITS=[8,4,2,1];
let CELL=0;
let state={A:null,B:null,t:0,playing:false,speed:1,dur:0};
function snapAll(sc){
  const n=DATA.maze.n, m=new Uint8Array(n*n);
  sc.frames.forEach(f=>{ (f.c||[]).forEach(x=>{m[x[0]]=x[1];}); f._m=m.slice(); });
}
function frameAt(sc,t){
  const fr=sc.frames; if(!fr.length) return -1;
  if(t<=fr[0].t) return 0;
  let lo=0,hi=fr.length-1;
  if(t>=fr[hi].t) return hi;
  while(hi-lo>1){const mid=(lo+hi)>>1; if(fr[mid].t<=t) lo=mid; else hi=mid;}
  return lo;
}
function mousePos(sc,i,t){
  const fr=sc.frames,f=fr[i],[dx,dy]=DIRS[f.h];
  let fx=f.x,fy=f.y;
  if(f.a==='F'&&i+1<fr.length){
    const g=fr[i+1],k=Math.min(1,Math.max(0,(t-f.t)/Math.max(1e-6,g.t-f.t)));
    fx+=dx*k; fy+=dy*k;
  }
  return [fx,fy,f.h];
}
function PX(x){return CELL*x;}                      // 迷路座標 -> canvas（北が上）
function PY(y){return CELL*(DATA.maze.n-1-y);}
function seg(ctx,x,y,d){
  const px=PX(x),py=PY(y);
  ctx.beginPath();
  if(d===0){ctx.moveTo(px,py);ctx.lineTo(px+CELL,py);}
  if(d===1){ctx.moveTo(px+CELL,py);ctx.lineTo(px+CELL,py+CELL);}
  if(d===2){ctx.moveTo(px,py+CELL);ctx.lineTo(px+CELL,py+CELL);}
  if(d===3){ctx.moveTo(px,py);ctx.lineTo(px,py+CELL);}
  ctx.stroke();
}
function grid(ctx,W){
  ctx.strokeStyle='#242424'; ctx.lineWidth=1; ctx.setLineDash([]);
  for(let i=0;i<=DATA.maze.n;i++){
    ctx.beginPath();ctx.moveTo(CELL*i,0);ctx.lineTo(CELL*i,W);ctx.stroke();
    ctx.beginPath();ctx.moveTo(0,CELL*i);ctx.lineTo(W,CELL*i);ctx.stroke();
  }
  ctx.font=Math.round(CELL*0.44)+'px monospace'; ctx.textAlign='center';
  ctx.fillStyle='#8a8a8a';
  DATA.maze.goals.forEach(g=>ctx.fillText('G',PX(g[0])+CELL/2,PY(g[1])+CELL*0.68));
  ctx.fillStyle='#7fd0ff';
  ctx.fillText('S',PX(DATA.maze.start[0])+CELL/2,PY(DATA.maze.start[1])+CELL*0.68);
}
function countTo(sc,t){let c=0;for(const f of sc.frames){if(f.t>t)break;if(f.n==='phantom')c++;}return c;}
function getCss(v){return getComputedStyle(document.documentElement).getPropertyValue(v).trim();}
function ring(ctx,x,y,r,c){ctx.setLineDash([]);ctx.strokeStyle=c;ctx.lineWidth=2.5;ctx.beginPath();ctx.arc(x,y,r,0,7);ctx.stroke();}
function draw(pane,t){
  const sc=state[pane]; if(!sc||!sc.frames.length) return;
  if(t===undefined||t===null) t=state.t;      // 呼出側で t を省略したら現在時刻を使う
  const cv=document.getElementById('c'+pane), ctx=cv.getContext('2d');
  const n=DATA.maze.n, W=cv.width;
  ctx.clearRect(0,0,W,W); grid(ctx,W);
  const i=frameAt(sc,t), fr=sc.frames, f=fr[i], m=f._m;
  if(document.getElementById('truth').checked){
    ctx.strokeStyle='#4d4d4d'; ctx.lineWidth=1; ctx.setLineDash([]);
    for(let y=0;y<n;y++)for(let x=0;x<n;x++){const w=DATA.maze.walls[y][x];
      for(let d=0;d<4;d++) if(w&WBITS[d]) seg(ctx,x,y,d);}
  }
  ctx.setLineDash([3,3]); ctx.strokeStyle=getCss('--wallMiss'); ctx.lineWidth=1.8;
  for(let y=0;y<n;y++)for(let x=0;x<n;x++){const real=DATA.maze.walls[y][x],kn=m[y*n+x];
    for(let d=0;d<4;d++) if((real&WBITS[d])&&!(kn&WBITS[d])) seg(ctx,x,y,d);}
  ctx.setLineDash([]); ctx.lineWidth=3;
  for(let y=0;y<n;y++)for(let x=0;x<n;x++){const real=DATA.maze.walls[y][x],kn=m[y*n+x];
    for(let d=0;d<4;d++){ if(!(kn&WBITS[d])) continue;
      ctx.strokeStyle=(real&WBITS[d])?getCss('--wallOk'):getCss('--wallBad'); seg(ctx,x,y,d); }}
  if(document.getElementById('order').checked){
    ctx.fillStyle='#8fd4ff'; ctx.font=Math.round(CELL*0.34)+'px monospace'; ctx.textAlign='center';
    const seen=new Set();
    for(let k=0;k<=i;k++){const p=fr[k],key=p.y*n+p.x;
      if(!seen.has(key)){seen.add(key);ctx.fillText(seen.size,PX(p.x)+CELL/2,PY(p.y)+CELL*0.62);}}
  }
  if(document.getElementById('trail').checked){
    ctx.strokeStyle=getCss('--trail'); ctx.lineWidth=1.8; ctx.globalAlpha=.85; ctx.beginPath();
    let started=false;
    for(let k=0;k<=i;k++){const p=fr[k];
      if(p.a==='P'||p.a==='C'){ started=false; continue; }        // 係員の再配置では線を引きたくない
      if(!started){ctx.moveTo(PX(p.x)+CELL/2,PY(p.y)+CELL/2);started=true;}
      else ctx.lineTo(PX(p.x)+CELL/2,PY(p.y)+CELL/2);}
    ctx.stroke(); ctx.globalAlpha=1;
  }
  if(f.a==='C'){                                // 係機が機体を开始位置へ运ぶ间は非表示
    ctx.setLineDash([5,4]); ctx.strokeStyle='#8a8a8a'; ctx.lineWidth=2;
    ctx.beginPath(); ctx.moveTo(PX(f.from[0])+CELL/2,PY(f.from[1])+CELL/2);
    ctx.lineTo(PX(f.to[0])+CELL/2,PY(f.to[1])+CELL/2); ctx.stroke(); ctx.setLineDash([]);
    ctx.fillStyle='#bdbdbd'; ctx.font=Math.round(CELL*0.36)+'px monospace'; ctx.textAlign='center';
    ctx.fillText('運搬',PX(f.to[0])+CELL*1.6,PY(f.to[1])-CELL*0.15);
  }
  if(f.a==='P'){                                        // 機体を開始位置へ戻した瞬間
    ctx.setLineDash([4,3]); ctx.strokeStyle='#ffd166'; ctx.lineWidth=2;
    ctx.strokeRect(PX(f.x)+4,PY(f.y)+4,CELL-8,CELL-8); ctx.setLineDash([]);
    ctx.fillStyle='#ffd166'; ctx.font=Math.round(CELL*0.40)+'px monospace'; ctx.textAlign='center';
    ctx.fillText('P',PX(f.x)+CELL/2,PY(f.y)+CELL*0.66);
  }
  const q=mousePos(sc,i,t),cx=PX(q[0])+CELL/2,cy=PY(q[1])+CELL/2;
  if(f.a!=='P'&&f.a!=='C'){ ctx.save(); ctx.translate(cx,cy); ctx.rotate([0,90,180,270][q[2]]*Math.PI/180);
  ctx.fillStyle=getCss('--mouse'); ctx.beginPath();
  ctx.moveTo(0,-CELL*.34); ctx.lineTo(CELL*.24,CELL*.26); ctx.lineTo(-CELL*.24,CELL*.26);
  ctx.closePath(); ctx.fill(); ctx.restore(); }
  if(f.n==='phantom'||f.n==='stuck') ring(ctx,cx,cy,CELL*.5,'#ff5c5c');
  if(f.n==='goal') ring(ctx,cx,cy,CELL*.5,'#57d38a');
  const done=t>=sc.dur;
  document.getElementById('m'+pane).innerHTML='<span>t <b>'+f.t.toFixed(2)+'s</b>'+(done?' / 完了':'')+'</span>'
    +'<span>run <b>'+f.r+'</b></span><span>mode <b>'+(f.m?'goal':'explore')+'</b></span>'
    +'<span>既知 <b>'+(100*f.k/(n*n)).toFixed(1)+'%</b></span>'
    +'<span>phantom <b>'+countTo(sc,t)+'</b></span>'
    +'<span>誤認壁 <b>'+f.w+'</b> / 見落とし <b>'+f.s+'</b></span>'
    +'<span>action <b>'+(f.a||'-')+'</b></span>';
}
function drawStrip(pane){
  const sc=state[pane]; if(!sc) return;
  const cv=document.getElementById('t'+pane), ctx=cv.getContext('2d');
  ctx.clearRect(0,0,cv.width,cv.height); ctx.fillStyle='#1e1e1e'; ctx.fillRect(0,0,cv.width,cv.height);
  const tmax=sc.dur||1;
  sc.frames.forEach(f=>{const x=cv.width*f.t/tmax;
    if(f.n==='phantom'){ctx.fillStyle='#ff5c5c';ctx.fillRect(x,0,2,cv.height);}
    else if(f.n==='goal'){ctx.fillStyle='#57d38a';ctx.fillRect(x,0,2,cv.height);}
    else if(f.a==='C'||f.a==='P'){ctx.fillStyle='#f2f2f2';ctx.fillRect(x,0,2,cv.height);}
    else if(f.a==='S'||f.n==='stop'){ctx.fillStyle='#ffd166';ctx.fillRect(x,0,2,cv.height);}
    else if(f.m===0){ctx.fillStyle='#2f6fb5';ctx.fillRect(x,cv.height-6,1,6);}});
  ctx.fillStyle='#ffd166'; ctx.fillRect(cv.width*Math.min(1,state.t/state.dur)-1,0,2,cv.height);
}
function loadPane(pane,key){
  const sc=JSON.parse(JSON.stringify(DATA.scenarios[key]));
  snapAll(sc); sc.dur=sc.frames.length?sc.frames[sc.frames.length-1].t:1; sc.key=key;
  state[pane]=sc;
  document.getElementById('lab'+pane).textContent=sc.label;
  document.getElementById('desc'+pane).textContent=sc.desc;
  state.dur=Math.max(state.A?state.A.dur:0,state.B?state.B.dur:0);
  render();
}
function render(){['A','B'].forEach(p=>{draw(p,state.t);drawStrip(p);});
  document.getElementById('scrub').value=String(Math.round(1000*state.t/Math.max(1e-6,state.dur)));
  document.getElementById('clock').textContent='t = '+state.t.toFixed(2)+' s';}
let last=0;
function loop(ts){
  if(state.playing){const dt=(ts-last)/1000*state.speed; state.t=Math.min(state.dur,state.t+dt);
    if(state.t>=state.dur){state.playing=false;document.getElementById('play').textContent='▶ 再生';}
    render();}
  last=ts; requestAnimationFrame(loop);
}
function boot(){
  const keys=Object.keys(DATA.scenarios);
  const size=Math.min(560,Math.max(320,Math.floor((window.innerWidth-80)/2)));
  CELL=size/DATA.maze.n;
  ['A','B'].forEach(p=>{const c=document.getElementById('c'+p);c.width=size;c.height=size;
    const s=document.getElementById('t'+p);s.width=size;s.height=26;
    const sel=document.getElementById('sel'+p);
    keys.forEach(k=>{const o=document.createElement('option');o.value=k;
      o.textContent=DATA.scenarios[k].label;sel.appendChild(o);});
    sel.onchange=()=>loadPane(p,sel.value);});
  document.getElementById('selA').value=keys.includes('current')?'current':keys[0];
  document.getElementById('selB').value=keys.includes('fixed_gain')?'fixed_gain':keys[keys.length-1];
  loadPane('A',document.getElementById('selA').value);
  loadPane('B',document.getElementById('selB').value);
  document.getElementById('play').onclick=()=>{state.playing=!state.playing;
    document.getElementById('play').textContent=state.playing?'⏸ 停止':'▶ 再生';};
  document.getElementById('reset').onclick=()=>{state.t=0;render();};
  document.getElementById('back').onclick=()=>{state.t=Math.max(0,state.t-0.2);render();};
  document.getElementById('fwd').onclick=()=>{state.t=Math.min(state.dur,state.t+0.2);render();};
  document.getElementById('nextPhantom').onclick=()=>{
    for(const p of ['A','B']){const sc=state[p];if(!sc)continue;
      const f=sc.frames.find(g=>g.t>state.t+1e-6&&(g.n==='phantom'||g.n==='goal'));
      if(f){state.t=f.t;break;}}render();};
  document.getElementById('scrub').oninput=e=>{state.t=state.dur*(+e.target.value)/1000;
    state.playing=false;document.getElementById('play').textContent='▶ 再生';render();};
  ['truth','order','trail'].forEach(id=>document.getElementById(id).onchange=()=>render());
  document.getElementById('speed').onchange=e=>state.speed=+e.target.value;
  window.onkeydown=e=>{
    if(e.code==='Space'){e.preventDefault();document.getElementById('play').click();}
    if(e.key==='ArrowLeft'){state.t=Math.max(0,state.t-0.2);render();}
    if(e.key==='ArrowRight'){state.t=Math.min(state.dur,state.t+0.2);render();}
    if(e.key==='ArrowUp'){state.speed=Math.min(8,state.speed*2);}
    if(e.key==='ArrowDown'){state.speed=Math.max(0.5,state.speed/2);}
    document.getElementById('speed').value=String(state.speed);};
  render(); requestAnimationFrame(loop);
}
boot();
</script></body></html>
"""


def maze_payload(maze):
    return {"name": maze.name, "n": maze.n,
            "walls": [list(row) for row in maze.walls],
            "start": list(maze.start), "goals": [list(g) for g in maze.goals]}


def build(maze_file, presets, out_path, motion, runs=1):
    maze = Maze.load(maze_file)
    scen = {}
    for k in presets:
        tr = trace_sim.run_preset(maze, motion, k, {"runs": runs})
        scen[k] = {"label": tr.label, "desc": tr.desc, "frames": tr.frames, "meta": tr.meta}
        print("  %-11s frames=%5d t_goal=%-7s phantom=%-3d known=%5.1f%% wrong=%d" % (
            k, tr.meta["frames"], tr.meta["t_first_goal"], tr.meta["phantom"],
            tr.meta["known_pct"], tr.meta["wrong_walls"]))
    payload = {"maze": maze_payload(maze), "scenarios": scen}
    footer = []
    for k, s in scen.items():
        m = s["meta"]
        footer.append("%s: 初ゴール %s s / 累計 %s s / phantom %d / 踏破 %.1f%% / 誤認壁 %d" % (
            s["label"], m["t_first_goal"], m["t_total"], m["phantom"], m["known_pct"],
            m["wrong_walls"]))
    html = (TEMPLATE
            .replace("__TITLE__", "exploration viewer / %s" % maze.name)
            .replace("__H1__", "探索の歩き方と地図生成の可視化 — %s" % maze.name)
            .replace("__FOOTER__", "<br>".join(footer) +
                     "<br>time model: %s" % motion.describe())
            .replace("__PAYLOAD__", json.dumps(payload, ensure_ascii=False, separators=(",", ":"))))
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(html)
    print("  wrote %s (%.1f KB)" % (out_path, os.path.getsize(out_path) / 1024))
    return out_path


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--maze", default="16MM2017CX", help="迷路名またはパス")
    p.add_argument("--maze-dir")
    p.add_argument("--runs", type=int, default=1, help="シミュレートする走行本数（既定 1 本）")
    p.add_argument("--presets",
                   default="current,sensor_only,write_only,fixed_map,fixed_gain,topmouse")
    p.add_argument("--out", help="出力 HTML。省略時 out/exploration_sim/viewer_<maze>.html")
    p.add_argument("--all-classic", action="store_true", help="全日本決勝迷路 (16MM*CX) 分作る")
    p.add_argument("--board", default="classic_r1_0")
    a = p.parse_args()

    class A:
        time_model = "physics"
        params = None
        cell_mm = 180.0
        turn_settle_ms = 60.0
        v_max = accel = w90 = a90 = None
        no_45cut = False
        board = a.board
    motion = cli.build_motion(A())
    mdir = cli.find_maze_dir(a.maze_dir)
    presets = [s.strip() for s in a.presets.split(",") if s.strip() in trace_sim.PRESETS]

    if a.all_classic:
        files = sorted(glob.glob(os.path.join(mdir, "16MM20*CX.maze")))
    else:
        cands = [a.maze, os.path.join(mdir or "", a.maze),
                 os.path.join(mdir or "", a.maze + ".maze")]
        files = [c for c in cands if c and os.path.isfile(c)][:1]
        if not files:
            print("迷路が見つかりません:", a.maze)
            return 2
    default_dir = os.path.join(cli.REPO, "out", "exploration_sim")
    for f in files:
        name = os.path.basename(f).split(".")[0]
        out = a.out or os.path.join(default_dir, "viewer_%s.html" % name)
        print("# %s" % name)
        build(f, presets, out, motion, a.runs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
