'use strict';

const $ = (id) => document.getElementById(id);
const COLORS = { baseline: '#efba6b', relevant: '#54ddc6' };
const DXY = [[0, 1], [1, 0], [0, -1], [-1, 0]];
const PHASES = { goal: 'ゴールへ', verify: '経路を検証', full: '全域を探索', home: 'スタートへ帰還', done: '探索終了' };
const REASONS = { route_certified: 'ゴール到達・経路確定', all_reachable_visited: '到達可能区画の探索完了', step_limit: '歩数上限で中断', no_reachable_target: '到達可能な観測目標なし', no_feasible_route: '最短走行の実行可能経路なし', full_map_without_certificate: '全域探索後も経路未確定' };
const PARAMS = [
  ['search_speed_mm_s', '探索・小回り速度', 'mm/s', 1, 10, 'primary'],
  ['known_speed_mm_s', '既知直進区間の最高速度', 'mm/s', 1, 10, 'primary'],
  ['acceleration_mm_s2', '通常直進の加速度', 'mm/s²', 1, 100, 'advanced'],
  ['dash_acceleration_mm_s2', '既知区間の加速度', 'mm/s²', 1, 100, 'advanced'],
  ['turn_alpha_deg_s2', '小回り旋回の角加速度', '°/s²', 1, 1000, 'advanced'],
  ['turn_omega_cap_deg_s', '小回り角速度の上限（0 = 無制限）', '°/s', 0, 100, 'advanced'],
  ['turn90_s', '90°旋回時間の実測補正', 's', 0.001, 0.001, 'advanced'],
  ['uturn_s', 'Uターンの旋回・待ち時間', 's', 0.001, 0.01, 'advanced'],
  ['sensor_delay_s', '区画ごとの観測遅延', 's', 0, 0.001, 'advanced'],
];
const state = { catalog: null, family: 'half', machine: 'mini_r2_0', result: null, views: {},
  mode: 'time', cursor: 0, playing: false, lastFrame: 0, busy: false, chartBounds: null, extraProfileOverrides: {}, maxSteps: 12000 };

function text(id, value) { $(id).textContent = value; }
function finite(value) { return typeof value === 'number' && Number.isFinite(value); }
function seconds(value, decimals = 2) { return finite(value) ? `${value.toFixed(decimals)} s` : '未導出'; }
function clock(value, fractional = false) {
  if (!finite(value)) return '—';
  const m = Math.floor(Math.max(0, value) / 60);
  const s = Math.max(0, value) - m * 60;
  return `${String(m).padStart(2, '0')}:${(fractional ? s.toFixed(1) : String(Math.floor(s))).padStart(fractional ? 4 : 2, '0')}`;
}
function humanDuration(value) {
  if (!finite(value)) return '—';
  return `${Math.floor(value / 60)}分 ${String(Math.floor(value % 60)).padStart(2, '0')}秒`;
}
function element(tag, className, content) {
  const item = document.createElement(tag);
  if (className) item.className = className;
  if (content !== undefined) item.textContent = content;
  return item;
}
function status(message, kind = '') { text('job-status', message); $('job-status').className = `job-status ${kind}`; }
async function api(url, options) {
  const response = await fetch(url, options);
  const data = await response.json();
  if (!response.ok || data?.status === 'error') throw new Error(data?.error || `HTTP ${response.status}`);
  return data;
}

function populateYears() {
  const prior = $('year-select').value;
  const years = [...new Set(state.catalog.mazes.filter((m) => m.family === state.family).map((m) => m.year))].sort((a, b) => b - a);
  $('year-select').replaceChildren(new Option('すべて', 'all'), ...years.map((year) => new Option(`${year}年`, year)));
  if (years.includes(Number(prior))) $('year-select').value = prior;
  populateMazes();
}
function populateMazes() {
  const selected = $('maze-select').value;
  const year = $('year-select').value;
  const finals = $('round-select').value === 'final';
  const entries = state.catalog.mazes.filter((m) => m.family === state.family && (year === 'all' || m.year === Number(year)) && (!finals || m.is_final));
  $('maze-select').replaceChildren(...entries.map((maze) => {
    const option = new Option(`${maze.name}${maze.available === false ? ' · データ不完全' : ''}`, maze.id);
    option.disabled = maze.available === false;
    if (option.disabled) option.title = maze.error || '';
    return option;
  }));
  const usable = entries.filter((m) => m.available !== false);
  if (usable.some((m) => m.id === selected)) $('maze-select').value = selected;
  else if (usable.length) $('maze-select').value = usable[0].id;
  else { $('maze-select').replaceChildren(new Option('選択条件に利用可能な迷路がありません', '')); }
  $('run-button').disabled = state.busy || !usable.length;
  updateMazeMeta();
}
function updateMazeMeta() {
  const maze = state.catalog.mazes.find((m) => m.id === $('maze-select').value);
  $('run-button').disabled = state.busy || !maze || maze.available === false;
  if (!maze) { text('maze-meta', '年度またはラウンドを変更してください。'); return; }
  text('maze-meta', `${maze.width} × ${maze.height} 区画 · ゴール ${maze.goals?.length || 0} 区画 · ${maze.id}`);
  $('source-link').href = maze.source_url || 'https://www.kerislab.jp/micromouse-maze-data/';
}
function populateProfile() {
  state.extraProfileOverrides = {};
  const profile = state.catalog.profiles[state.machine];
  text('machine-name', state.machine === 'mini_r2_0' ? 'mini_r2' : 'classic_r1');
  $('primary-params').replaceChildren(); $('advanced-params').replaceChildren();
  PARAMS.forEach(([key, title, unit, minimum, increment, location]) => {
    if (!finite(profile[key])) return;
    const label = element('label', 'input-with-unit');
    label.append(element('span', '', title));
    const wrap = element('div');
    const input = element('input');
    input.id = `param-${key}`; input.type = 'number'; input.min = minimum;
    input.step = 'any'; input.required = true;
    input.value = String(Number(profile[key].toFixed(6)));
    input.dataset.initial = input.value;
    input.dataset.key = key;
    input.setAttribute('aria-label', `${title} (${unit})`);
    input.addEventListener('input', () => label.classList.toggle('modified', Number(input.value) !== Number(input.dataset.initial)));
    if (key === 'turn90_s') input.title = 'この値を変更した場合だけ、角速度からの自動計算を上書きします。';
    wrap.append(input, element('span', '', unit)); label.append(wrap);
    $(`${location}-params`).append(label);
  });
  const note = element('p', 'field-note', '90°旋回時間を変更すると自動計算を上書きします。未変更なら速度・角加速度から再計算します。');
  $('advanced-params').append(note);
}
function selectFamily(family) {
  state.family = family; state.machine = family === 'half' ? 'mini_r2_0' : 'classic_r1_0';
  document.querySelectorAll('[data-family]').forEach((button) => {
    const active = button.dataset.family === family;
    button.classList.toggle('active', active); button.setAttribute('aria-pressed', active);
  });
  $('shortest-case').replaceChildren(...Array.from({ length: family === 'half' ? 4 : 9 }, (_, index) => {
    const value = index + (family === 'half' ? 6 : 1); return new Option(`case ${value}`, value);
  }));
  $('shortest-case').value = family === 'half' ? '8' : '1';
  populateYears(); populateProfile();
}
function profileOverrides() {
  const overrides = { ...state.extraProfileOverrides };
  document.querySelectorAll('[data-key]').forEach((input) => {
    const value = Number(input.value);
    if (!Number.isFinite(value) || input.value.trim() === '') throw new Error('走行パラメータには有効な数値を指定してください。');
    if (value !== Number(input.dataset.initial)) overrides[input.dataset.key] = value;
  });
  return overrides;
}
async function runExperiment(event) {
  event.preventDefault();
  if (state.busy) return;
  pause();
  try {
    const request = { maze_id: $('maze-select').value, machine: state.machine, profile: profileOverrides(),
      shortest_mode: Number($('shortest-mode').value), shortest_case: Number($('shortest-case').value),
      epsilon: Number($('epsilon').value) / 100, return_home: $('return-home').checked, max_steps: state.maxSteps };
    state.busy = true; $('run-button').disabled = true;
    status('比較を準備中。最短走行モデルを初回にビルドします。', 'running');
    const job = await api('/api/simulate', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(request) });
    let completed = false;
    while (!completed) {
      await new Promise((resolve) => setTimeout(resolve, 850));
      const response = await api(`/api/jobs/${encodeURIComponent(job.job_id)}`);
      if (response.status === 'done') {
        installResult(response.result); completed = true;
        status('比較が完了しました。タイムラインを動かして探索順を確認できます。');
      } else {
        const progress = response.progress || {};
        const name = progress.algorithm === 'baseline' ? '従来版' : progress.algorithm === 'relevant' ? '改善版' : '探索';
        status(response.status === 'queued' ? '先行する実験の完了を待っています。' : `${name}を計算中${finite(progress.step) ? ` · ${progress.step.toLocaleString()} 歩` : '…'}${finite(progress.known_edges) ? ` · ${progress.known_edges.toLocaleString()} 境界を観測済み` : ''}`, 'running');
      }
    }
  } catch (error) { status(error.message || String(error), 'error'); }
  finally { state.busy = false; $('run-button').disabled = !$('maze-select').value; }
}

function restoreReplay(result) {
  if (!result?.maze || !result.profile || !Array.isArray(result.runs)) throw new Error('保存リプレイの形式が不正です。');
  const family = result.profile.family || result.maze.family || (result.profile.machine === 'classic_r1_0' ? 'classic' : 'half');
  selectFamily(family);
  $('year-select').value = 'all'; $('round-select').value = 'all'; populateMazes();
  const mazeId = (result.maze.id || '').replace(/\.maze$/, '');
  const inCatalog = state.catalog.mazes.some((maze) => maze.id === mazeId && maze.available !== false);
  if (inCatalog) { $('maze-select').value = mazeId; updateMazeMeta(); }
  else {
    $('maze-select').append(new Option(`${result.maze.name || mazeId} · 保存リプレイ`, mazeId));
    $('maze-select').value = mazeId; $('run-button').disabled = true;
    text('maze-meta', 'この迷路は現在のデータセットにありません。再実行する場合は別の迷路を選択してください。');
  }
  for (const [key] of PARAMS) {
    const input = $(`param-${key}`);
    if (!input || !finite(result.profile[key])) continue;
    input.value = String(Number(result.profile[key].toFixed(6)));
    input.closest('label').classList.toggle('modified', Number(input.value) !== Number(input.dataset.initial));
  }
  // Preserve CLI-only edits too when rerunning a saved comparison.
  const defaults = state.catalog.profiles[state.machine];
  for (const key of ['turn_rounding', 'turn_entry_mm', 'turn_exit_mm']) {
    if (finite(result.profile[key]) && result.profile[key] !== defaults[key]) state.extraProfileOverrides[key] = result.profile[key];
  }
  const options = result.options || {};
  state.maxSteps = options.max_steps || 12000;
  if (options.shortest_mode != null) $('shortest-mode').value = String(options.shortest_mode);
  if (options.shortest_case != null) $('shortest-case').value = String(options.shortest_case);
  $('epsilon').value = String(Number(((options.epsilon || 0) * 100).toFixed(6)));
  $('return-home').checked = Boolean(options.return_home);
  installResult(result);
  status('保存済みの設定と結果を復元しました。再生ボタンで探索順を確認できます。');
}

function installResult(result) {
  if (!result.maze || !Array.isArray(result.runs) || !result.runs.length) throw new Error('シミュレーション結果の形式が不正です。');
  state.result = result; state.cursor = 0; state.views = {};
  result.runs.forEach((run) => { state.views[run.algorithm] = createView(run); });
  $('empty-state').hidden = true; $('results').hidden = false;
  text('experiment-title', result.maze.name || result.maze.id || '探索比較');
  text('experiment-subtitle', `${result.maze.width} × ${result.maze.height} 区画 / ${result.profile.machine || state.machine} / 探索 mode 1 case 1 を基準${result.options?.return_home ? ' / 帰還を含む' : ' / 帰還を含まない'}`);
  const byName = Object.fromEntries(result.runs.map((run) => [run.algorithm, run]));
  for (const name of ['baseline', 'relevant']) {
    const summary = byName[name]?.summary;
    if (!summary) continue;
    const metric = $(`${name}-duration`); metric.replaceChildren(document.createTextNode(clock(summary.duration_s)), element('small', '', '推定'));
    text(`${name}-foot`, `${summary.steps.toLocaleString()} 歩 · ${summary.distance_m.toFixed(2)} m${summary.completed ? '' : ' · 未完了'}`);
  }
  const baseline = byName.baseline?.summary; const relevant = byName.relevant?.summary;
  if (baseline && relevant && baseline.completed && relevant.completed && baseline.certified && relevant.certified && baseline.duration_s > 0) {
    const saving = baseline.duration_s - relevant.duration_s;
    const percent = saving / baseline.duration_s * 100;
    $('saving-percent').replaceChildren(document.createTextNode(`${percent >= 0 ? '−' : '+'}${Math.abs(percent).toFixed(1)}`), element('small', '', '%'));
    text('saving-time', `${humanDuration(Math.abs(saving))} ${saving >= 0 ? '短縮' : '増加'} · 同一の速度条件`);
  } else { text('saving-percent', '—'); text('saving-time', '未完了・経路未確定のため算出しません'); }
  fillComparison(); fillProvenance(); configureTimeline(); render();
}
function createView(run) {
  const { width, height } = state.result.maze;
  const view = { run, index: -1, known: Array.from({ length: height }, () => Array(width).fill(0)), walls: Array.from({ length: height }, () => Array(width).fill(0)), visits: Array.from({ length: height }, () => Array(width).fill(0)) };
  for (let y = 0; y < height; y++) for (let x = 0; x < width; x++) {
    DXY.forEach(([dx, dy], d) => {
      if (x + dx < 0 || x + dx >= width || y + dy < 0 || y + dy >= height) { view.known[y][x] |= 1 << d; view.walls[y][x] |= 1 << d; }
    });
  }
  return view;
}
function eventIndex(events, value, field) {
  let low = 0, high = events.length - 1;
  while (low < high) { const mid = Math.ceil((low + high) / 2); if (events[mid][field] <= value) low = mid; else high = mid - 1; }
  return low;
}
function updateView(name) {
  let view = state.views[name];
  if (!view) return null;
  const index = eventIndex(view.run.events, state.cursor, state.mode === 'time' ? 't' : 'step');
  if (index < view.index) { view = createView(view.run); state.views[name] = view; }
  for (let i = view.index + 1; i <= index; i++) {
    const event = view.run.events[i];
    for (const [x, y, d, wall] of event.changes || []) {
      view.known[y][x] |= 1 << d;
      if (wall) view.walls[y][x] |= 1 << d;
      const nx = x + DXY[d][0], ny = y + DXY[d][1], opposite = (d + 2) % 4;
      if (ny >= 0 && ny < view.known.length && nx >= 0 && nx < view.known[ny].length) {
        view.known[ny][nx] |= 1 << opposite;
        if (wall) view.walls[ny][nx] |= 1 << opposite;
      }
    }
    view.visits[event.y][event.x]++;
  }
  view.index = index;
  return view;
}
function maxCursor() {
  if (!state.result) return 1;
  return Math.max(1, ...state.result.runs.map((run) => state.mode === 'time' ? run.summary.duration_s : run.summary.steps));
}
function configureTimeline() {
  // Native range controls snap to step increments. Round the outer limit up
  // so End/dragging all the way right can reach the final observation too.
  $('timeline').max = state.mode === 'time' ? Math.ceil(maxCursor() * 100) / 100 : maxCursor();
  $('timeline').step = state.mode === 'time' ? '0.01' : '1';
  $('timeline').value = state.cursor;
  text('timeline-end', state.mode === 'time' ? clock(maxCursor(), true) : `${maxCursor().toLocaleString()} STEPS`);
}
function setCursor(value) { state.cursor = Math.min(maxCursor(), Math.max(0, value)); $('timeline').value = state.cursor; render(); }
function switchMode(mode) {
  if (mode === state.mode) return;
  pause();
  const run = state.views.relevant?.run || state.result?.runs[0];
  const event = run ? run.events[eventIndex(run.events, state.cursor, state.mode === 'time' ? 't' : 'step')] : null;
  state.mode = mode;
  state.cursor = event ? (mode === 'time' ? event.t : event.step) : 0;
  document.querySelectorAll('[data-sync]').forEach((button) => {
    const active = button.dataset.sync === mode; button.classList.toggle('active', active); button.setAttribute('aria-pressed', active);
  });
  configureTimeline(); render();
}
function pause() { state.playing = false; text('play', '▶'); $('play').setAttribute('aria-label', '再生'); }
function play() {
  if (!state.result) return;
  if (state.playing) { pause(); return; }
  if (state.cursor >= maxCursor()) setCursor(0);
  state.playing = true; state.lastFrame = performance.now(); text('play', 'Ⅱ'); $('play').setAttribute('aria-label', '一時停止');
  requestAnimationFrame(tick);
}
function tick(now) {
  if (!state.playing) return;
  const dt = Math.min(0.15, (now - state.lastFrame) / 1000); state.lastFrame = now;
  setCursor(state.cursor + dt * Number($('speed').value) * (state.mode === 'step' ? 3 : 1));
  if (state.cursor >= maxCursor()) pause();
  else requestAnimationFrame(tick);
}
function stepBy(direction) {
  pause(); if (!state.result) return;
  if (state.mode === 'step') { setCursor(Math.floor(state.cursor) + direction); return; }
  const candidates = state.result.runs.flatMap((run) => {
    const index = eventIndex(run.events, state.cursor + (direction < 0 ? -0.0001 : 0), 't');
    const event = run.events[direction > 0 ? index + 1 : index];
    return event ? [event.t] : [];
  }).filter((value) => direction > 0 ? value > state.cursor + 0.00001 : value < state.cursor - 0.00001);
  setCursor(candidates.length ? (direction > 0 ? Math.min(...candidates) : Math.max(...candidates)) : (direction > 0 ? maxCursor() : 0));
}
function context(canvas) {
  const bounds = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  const width = Math.max(1, bounds.width), height = Math.max(1, bounds.height);
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) { canvas.width = Math.round(width * dpr); canvas.height = Math.round(height * dpr); }
  const ctx = canvas.getContext('2d'); ctx.setTransform(dpr, 0, 0, dpr, 0, 0); ctx.clearRect(0, 0, width, height);
  return { ctx, width, height };
}
function drawMaze(name, view) {
  const canvas = $(`${name}-maze`); const { ctx, width, height } = context(canvas);
  const maze = state.result.maze, color = COLORS[name];
  const pad = maze.width >= 24 ? 16 : 22;
  const cell = Math.min((width - 2 * pad) / maze.width, (height - 2 * pad) / maze.height);
  const ox = (width - cell * maze.width) / 2, oy = (height - cell * maze.height) / 2;
  const centre = (x, y) => [ox + (x + 0.5) * cell, oy + (maze.height - y - 0.5) * cell];
  const rectangle = (x, y) => [ox + x * cell, oy + (maze.height - y - 1) * cell, cell, cell];
  const edge = (x, y, d) => {
    const left = ox + x * cell, top = oy + (maze.height - y - 1) * cell;
    return [[left, top, left + cell, top], [left + cell, top, left + cell, top + cell], [left, top + cell, left + cell, top + cell], [left, top, left, top + cell]][d];
  };
  const drawEdge = (x, y, d) => { const [x1, y1, x2, y2] = edge(x, y, d); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); };
  const event = view.run.events[view.index];
  if ($('show-heat').checked) {
    for (let y = 0; y < maze.height; y++) for (let x = 0; x < maze.width; x++) {
      if (!view.visits[y][x]) continue;
      ctx.globalAlpha = Math.min(0.32, 0.055 + view.visits[y][x] * 0.045); ctx.fillStyle = color; ctx.fillRect(...rectangle(x, y));
    }
  }
  ctx.globalAlpha = 1;
  for (const [x, y] of maze.goals) { ctx.fillStyle = '#64c9b524'; ctx.fillRect(...rectangle(x, y)); }
  // Draw each physical edge exactly once. Unobserved edges carry no truth.
  ctx.strokeStyle = '#375064'; ctx.lineWidth = Math.max(0.65, cell / 30); ctx.setLineDash([1.3, 2.7]); ctx.beginPath();
  for (let y = 0; y < maze.height; y++) for (let x = 0; x < maze.width; x++) for (const d of [0, 1]) {
    if (!(view.known[y][x] & (1 << d))) drawEdge(x, y, d);
  }
  ctx.stroke(); ctx.setLineDash([]);
  if ($('show-truth').checked) {
    ctx.strokeStyle = '#d6dee02f'; ctx.lineWidth = Math.max(1, cell / 15); ctx.beginPath();
    for (let y = 0; y < maze.height; y++) for (let x = 0; x < maze.width; x++) for (const d of [0, 1]) {
      if (!(view.known[y][x] & (1 << d)) && maze.walls[y][x] & (1 << d)) drawEdge(x, y, d);
    }
    ctx.stroke();
  }
  if ($('show-trail').checked && view.index > 0) {
    ctx.strokeStyle = `${color}80`; ctx.lineWidth = Math.max(1.2, cell * 0.11); ctx.lineCap = 'round'; ctx.lineJoin = 'round'; ctx.beginPath();
    for (let i = 0; i <= view.index; i++) { const ev = view.run.events[i]; const point = centre(ev.x, ev.y); if (!i) ctx.moveTo(...point); else ctx.lineTo(...point); }
    ctx.stroke();
  }
  const finished = view.index === view.run.events.length - 1;
  if ($('show-route').checked && (finished || $('show-truth').checked) && view.run.known_route?.length) {
    ctx.strokeStyle = '#d7f3fa'; ctx.lineWidth = Math.max(1.8, cell * 0.16); ctx.setLineDash([4, 3]); ctx.beginPath();
    view.run.known_route.forEach((point, index) => {
      const x = Array.isArray(point) ? point[0] : point.x, y = Array.isArray(point) ? point[1] : point.y;
      if (!finite(x) || !finite(y)) return;
      if (index) ctx.lineTo(...centre(x, y)); else ctx.moveTo(...centre(x, y));
    }); ctx.stroke(); ctx.setLineDash([]);
  }
  ctx.lineCap = 'square'; ctx.strokeStyle = '#b9c9d8'; ctx.lineWidth = Math.max(1.15, cell / 13); ctx.beginPath();
  for (let y = 0; y < maze.height; y++) for (let x = 0; x < maze.width; x++) {
    for (const d of [0, 1, ...(y === 0 ? [2] : []), ...(x === 0 ? [3] : [])]) if (view.walls[y][x] & (1 << d)) drawEdge(x, y, d);
  }
  ctx.stroke();
  // Highlight current observations (including observed-open boundaries).
  ctx.strokeStyle = color; ctx.lineWidth = Math.max(1.9, cell / 9);
  for (const [x, y, d, wall] of event.changes || []) {
    ctx.globalAlpha = wall ? 1 : 0.65; ctx.setLineDash(wall ? [] : [2, 2]); ctx.beginPath(); drawEdge(x, y, d); ctx.stroke();
  }
  ctx.globalAlpha = 1; ctx.setLineDash([]);
  ctx.font = `${Math.max(6, Math.min(11, cell * 0.45))}px ui-monospace,monospace`; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
  for (const [x, y] of maze.goals) { ctx.fillStyle = '#91b9ae'; ctx.fillText('G', ...centre(x, y)); }
  ctx.fillStyle = '#a0b6c5'; ctx.fillText('S', ...centre(...maze.start));
  if (event.target && !finished) {
    const [tx, ty] = centre(...event.target); const radius = Math.max(4, cell * 0.27);
    ctx.strokeStyle = color; ctx.lineWidth = 1.1; ctx.beginPath(); ctx.moveTo(tx, ty - radius); ctx.lineTo(tx + radius, ty); ctx.lineTo(tx, ty + radius); ctx.lineTo(tx - radius, ty); ctx.closePath(); ctx.stroke();
  }
  let px = event.x, py = event.y, heading = event.heading;
  if (state.mode === 'time' && !finished && state.playing) {
    const next = view.run.events[view.index + 1]; const fraction = Math.min(1, Math.max(0, (state.cursor - event.t) / Math.max(0.0001, next.t - event.t)));
    px += (next.x - px) * fraction; py += (next.y - py) * fraction;
    let diff = ((next.heading - heading + 6) % 4) - 2; heading += diff * Math.min(1, fraction * 2);
  }
  const [rx, ry] = centre(px, py), radius = Math.max(4.2, cell * 0.35);
  ctx.save(); ctx.translate(rx, ry); ctx.rotate(heading * Math.PI / 2); ctx.shadowColor = color; ctx.shadowBlur = 8;
  ctx.fillStyle = color; ctx.strokeStyle = '#08141c'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(0, -radius); ctx.lineTo(radius * 0.72, radius * 0.75); ctx.lineTo(0, radius * 0.4); ctx.lineTo(-radius * 0.72, radius * 0.75); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
  ctx.font = '8px ui-monospace,monospace'; ctx.fillStyle = '#496478'; ctx.textAlign = 'center';
  const interval = maze.width > 16 ? 8 : 4;
  for (let x = 0; x < maze.width; x += interval) ctx.fillText(String(x), centre(x, 0)[0], height - oy / 2 + 1);
  ctx.textAlign = 'right';
  for (let y = 0; y < maze.height; y += interval) ctx.fillText(String(y), ox - 5, centre(0, y)[1]);
  canvas.setAttribute('aria-label', `${name === 'baseline' ? '従来' : '改善'}、${event.step} 歩、位置 (${event.x}, ${event.y})、${event.known_edges} 境界が既知`);
}
function renderLive(name, view) {
  const event = view.run.events[view.index], finished = view.index === view.run.events.length - 1;
  const live = $(`${name}-live`); live.replaceChildren();
  const values = [[`${event.step.toLocaleString()}`, '歩', '走行ステップ'], [clock(event.t, true), '', '経過時間'], [`${event.visited_cells.toLocaleString()}`, `/${state.result.maze.reachable_cells || state.result.maze.width * state.result.maze.height}`, '訪問区画']];
  values.forEach(([value, unit, label]) => { const item = element('span', '', label); const strong = element('strong', '', value); if (unit) strong.append(element('small', '', unit)); item.append(strong); live.append(item); });
  const phase = $(`${name}-phase`); phase.textContent = finished ? (view.run.summary.completed ? '探索終了' : '中断') : PHASES[event.phase] || event.phase;
  phase.classList.toggle('certified', event.certified);
  const bounds = $(`${name}-bounds`); bounds.replaceChildren(element('span', '', 'L'), element('strong', '', seconds(event.lower_s)), element('span', '', '→ U'), element('strong', '', seconds(event.upper_s)));
  const gap = finite(event.upper_s) && finite(event.lower_s) && event.lower_s > 0 ? `差 ${Math.max(0, (event.upper_s / event.lower_s - 1) * 100).toFixed(1)}%` : '既知経路なし';
  bounds.append(element('span', 'gap', event.certified ? '確定済み' : gap));
}
function drawChart() {
  const { ctx, width, height } = context($('bounds-chart'));
  const left = 40, right = width - 13, top = 16, bottom = height - 26;
  const runs = state.result.runs;
  const duration = Math.max(1, ...runs.map((run) => run.summary.duration_s));
  const costs = runs.flatMap((run) => run.events.flatMap((ev) => [ev.lower_s, ev.upper_s].filter(finite)));
  if (!costs.length) return;
  const maximum = Math.max(...costs) * 1.06 || 1; const minimum = 0;
  const px = (t) => left + (right - left) * t / duration, py = (cost) => bottom - (bottom - top) * (cost - minimum) / (maximum - minimum);
  state.chartBounds = { left, right, duration };
  ctx.font = '8px ui-monospace,monospace'; ctx.textAlign = 'right'; ctx.textBaseline = 'middle';
  for (let i = 0; i <= 4; i++) {
    const cost = maximum * i / 4, y = py(cost); ctx.strokeStyle = '#293949'; ctx.lineWidth = 0.6; ctx.beginPath(); ctx.moveTo(left, y); ctx.lineTo(right, y); ctx.stroke(); ctx.fillStyle = '#7c95a8'; ctx.fillText(`${cost.toFixed(1)}`, left - 7, y);
  }
  ctx.textAlign = 'center';
  for (let i = 0; i <= 4; i++) ctx.fillText(clock(duration * i / 4), px(duration * i / 4), bottom + 16);
  ctx.textAlign = 'left'; ctx.fillText('s', 6, top - 7);
  for (const run of runs) {
    for (const field of ['lower_s', 'upper_s']) {
      ctx.strokeStyle = COLORS[run.algorithm]; ctx.globalAlpha = field === 'lower_s' ? 0.65 : 0.95; ctx.lineWidth = field === 'lower_s' ? 1 : 1.7; ctx.setLineDash(field === 'lower_s' ? [4, 3] : []); ctx.beginPath();
      let started = false, previous = null;
      for (const event of run.events) {
        if (!finite(event[field])) { started = false; previous = null; continue; }
        const x = px(event.t), y = py(event[field]);
        if (!started) { ctx.moveTo(x, y); started = true; }
        else { ctx.lineTo(x, py(previous)); ctx.lineTo(x, y); }
        previous = event[field];
      }
      ctx.stroke();
    }
    const certificate = run.summary.certificate_s;
    if (finite(certificate) && finite(run.summary.upper_s)) {
      const certificateEvent = run.events.find((event) => event.certified);
      ctx.globalAlpha = 1; ctx.fillStyle = COLORS[run.algorithm]; ctx.beginPath(); ctx.arc(px(certificate), py(certificateEvent?.upper_s ?? run.summary.upper_s), 3, 0, Math.PI * 2); ctx.fill();
    }
  }
  ctx.globalAlpha = 1; ctx.setLineDash([]);
  for (const run of runs) {
    const event = run.events[eventIndex(run.events, state.cursor, state.mode === 'time' ? 't' : 'step')];
    const cursor = state.mode === 'time' ? Math.min(state.cursor, duration) : event.t;
    if (state.mode === 'time' && run !== runs[0]) continue;
    ctx.strokeStyle = state.mode === 'time' ? '#d3e6f08c' : `${COLORS[run.algorithm]}b0`; ctx.lineWidth = 1; ctx.setLineDash([2, 3]); ctx.beginPath(); ctx.moveTo(px(cursor), top); ctx.lineTo(px(cursor), bottom); ctx.stroke();
  }
  ctx.setLineDash([]);
}
function render() {
  if (!state.result || $('results').hidden) return;
  const bothFinished = state.result.runs.every((run) => state.cursor >= (state.mode === 'time' ? run.summary.duration_s : run.summary.steps));
  $('show-route').disabled = !(bothFinished || $('show-truth').checked);
  if ($('show-route').disabled) $('show-route').checked = false;
  for (const name of ['baseline', 'relevant']) { const view = updateView(name); if (view) { drawMaze(name, view); renderLive(name, view); } }
  text('cursor-label', state.mode === 'time' ? clock(state.cursor, true) : `${Math.floor(state.cursor).toLocaleString()} 歩`);
  drawChart();
}
function fillComparison() {
  const runs = ['baseline', 'relevant'].map((name) => state.result.runs.find((run) => run.algorithm === name));
  const rows = [
    ['探索終了', (s) => humanDuration(s.duration_s)], ['初回ゴール到達', (s) => finite(s.first_goal_s) ? humanDuration(s.first_goal_s) : '未到達'],
    ['経路の確定', (s) => finite(s.certificate_s) ? humanDuration(s.certificate_s) : '未確定'], ['帰還完了', (s) => finite(s.home_s) ? humanDuration(s.home_s) : '—'],
    ['走行距離', (s) => `${s.distance_m.toFixed(2)} m`], ['90°旋回 / Uターン', (s) => `${s.turns90} / ${s.uturns} 回`],
    ['訪問区画 / 観測境界', (s) => `${s.visited_cells} / ${s.known_edges}`], ['最短走行の下限 / 上限', (s) => `${seconds(s.lower_s)} / ${seconds(s.upper_s)}`],
    ['終了理由', (s) => REASONS[s.reason] || s.reason],
  ];
  if (runs.some((run) => finite(run?.summary.empty_transit_steps))) {
    rows.splice(5, 0, ['新たな観測のない移動', (summary) => finite(summary.empty_transit_steps) ? `${summary.empty_transit_steps} 歩` : '—']);
  }
  $('comparison-body').replaceChildren(...rows.map(([title, format]) => { const row = element('tr'); row.append(element('td', '', title), ...runs.map((run) => element('td', '', run ? format(run.summary) : '—'))); return row; }));
}
function fillProvenance() {
  const target = $('provenance'); target.replaceChildren();
  const { maze, profile, metadata, options } = state.result;
  target.append(element('h4', '', 'この比較の読み方'));
  const list = element('ul');
  [
    '機体は区画中心で前・左・右の壁を観測します。背後は通過済み通路として扱い、壁の先は見通しません。描画の「真の迷路」は表示専用です。',
    '探索時間は加減速・直進・小回り・Uターンに基づく運動学モデルです。壁合わせ、姿勢誤差、スリップ、制御周期、記録処理、操作者の待ち時間は実測と一致しません。',
    '既知区間の最高速度は変更可能な比較用上限です。実機での到達速度・加速条件は、実走ログで較正してください。',
    '最短走行の上下限はホスト時間プランナの同じ運動グラフから計算します。ゴール進入時間が目的関数であり、現実の機体の全ての軌道に対する最適性を保証するものではありません。',
    '両方式とも初回ゴールまでは足立法です。改善版はその後、最速になり得る経路の未知境界を観測する位置を移動時間で選びます。候補経路が観測した壁で成立しなくなったときに経路を再導出します。許容差 0% では上下限の一致が終了条件です。',
    '初回ゴールへの走行中は、計算済みの下限を保持することがあります。既知経路の上限は原則として新たに 32 境界を観測するごとに更新し、候補経路の必要境界がすべて既知になった場合は直ちに確定します。このためグラフの値は各区画での再計算値とは限りません。',
  ].forEach((line) => list.append(element('li', '', line)));
  if (profile.family === 'classic' || maze.family === 'classic') {
    list.append(element('li', '', 'クラシックの最短走行評価は直交経路の時間モデルを使用し、斜め走行は含みません。従来版の探索は GOAL → FULL を連続して切り替え、実機 F405 のゴール停止・後退・強制前進による再出発を省略しています。そのため、この部分は実機と探索順・所要時間が異なります。'));
  }
  target.append(list, element('h4', '', '探索速度の出典'));
  const sourceList = element('ul'); (profile.source_refs || []).forEach((ref) => { const item = element('li'); item.append(element('code', '', ref)); sourceList.append(item); }); target.append(sourceList);
  target.append(element('p', '', '90°旋回時間は角速度プロファイルと前後距離から算出。手入力した場合はその値を使用します。Uターン時間は初期推定であり、特に F413 の実測角度 PID 旋回は実走による補正が必要です。'));
  target.append(element('h4', '', '迷路データ'));
  const link = element('a', '', `${maze.id} · KeriLab / MIT`); link.href = maze.source_url || 'https://www.kerislab.jp/micromouse-maze-data/'; link.target = '_blank'; link.rel = 'noreferrer'; target.append(link);
  target.append(element('p', '', `ゴールは元データの G マーカー ${maze.goals.length} 区画を使用。座標は x が東、y が北、左下が (0, 0)。`));
  target.append(element('p', '', `データ revision: ${maze.source_revision || '不明'}`), element('p', '', `迷路 SHA-256: ${maze.source_sha256 || '不明'}`));
  target.append(element('h4', '', '今回の実行設定・ビルド情報'));
  target.append(element('pre', '', JSON.stringify({ profile, options, metadata }, null, 2)));
}
function download(content, type, extension) {
  const blob = new Blob([content], { type }); const url = URL.createObjectURL(blob);
  const anchor = element('a'); anchor.href = url; anchor.download = `nightfall_${state.result.maze.id}_${new Date().toISOString().replace(/[:.]/g, '-')}.${extension}`;
  document.body.append(anchor); anchor.click(); anchor.remove(); setTimeout(() => URL.revokeObjectURL(url), 5000);
}
function exportCSV() {
  if (!state.result) return;
  const columns = ['maze_id', 'source_sha256', 'source_revision', 'fw_git_sha', 'fw_git_dirty', 'machine', 'algorithm', 'step', 't', 'dt', 'x', 'y', 'heading', 'turn', 'known_straight', 'known_edges', 'visited_cells', 'lower_s', 'upper_s', 'certified', 'phase', 'target', 'changes', 'profile_json', 'options_json', 'metadata_json'];
  const quote = (value) => { if (value === null || value === undefined) return ''; const s = typeof value === 'object' ? JSON.stringify(value) : String(value); return `"${s.replaceAll('"', '""')}"`; };
  const lines = [columns.join(',')];
  for (const run of state.result.runs) for (const event of run.events) {
    const row = { ...event, maze_id: state.result.maze.id, source_sha256: state.result.maze.source_sha256, source_revision: state.result.maze.source_revision, fw_git_sha: state.result.metadata?.fw_git_sha, fw_git_dirty: state.result.metadata?.fw_git_dirty, machine: state.result.profile.machine, algorithm: run.algorithm,
      profile_json: state.result.profile, options_json: state.result.options, metadata_json: state.result.metadata };
    lines.push(columns.map((key) => quote(row[key])).join(','));
  }
  download('\ufeff' + lines.join('\r\n') + '\r\n', 'text/csv;charset=utf-8', 'csv');
}

$('setup-form').addEventListener('submit', runExperiment);
$('year-select').addEventListener('change', populateMazes); $('round-select').addEventListener('change', populateMazes); $('maze-select').addEventListener('change', updateMazeMeta);
$('reset-profile').addEventListener('click', populateProfile);
document.querySelectorAll('[data-family]').forEach((button) => button.addEventListener('click', () => state.catalog && selectFamily(button.dataset.family)));
document.querySelectorAll('[data-sync]').forEach((button) => button.addEventListener('click', () => switchMode(button.dataset.sync)));
$('timeline').addEventListener('input', (event) => { pause(); setCursor(Number(event.target.value)); });
$('play').addEventListener('click', play); $('rewind').addEventListener('click', () => { pause(); setCursor(0); }); $('previous').addEventListener('click', () => stepBy(-1)); $('next').addEventListener('click', () => stepBy(1));
['show-trail', 'show-heat', 'show-truth', 'show-route'].forEach((id) => $(id).addEventListener('change', render));
$('export-json').addEventListener('click', () => state.result && download(JSON.stringify(state.result, null, 2), 'application/json', 'json'));
$('export-csv').addEventListener('click', exportCSV);
$('bounds-chart').addEventListener('click', (event) => {
  if (!state.chartBounds) return;
  pause(); const { left, right, duration } = state.chartBounds;
  const t = Math.max(0, Math.min(1, (event.clientX - event.currentTarget.getBoundingClientRect().left - left) / (right - left))) * duration;
  if (state.mode !== 'time') switchMode('time'); setCursor(t);
});
window.addEventListener('resize', render);
document.addEventListener('keydown', (event) => {
  if (event.target.closest('input,select,textarea,button,summary,a') || !state.result) return;
  if (event.code === 'Space') { event.preventDefault(); play(); }
  if (event.code === 'ArrowRight') { event.preventDefault(); stepBy(1); }
  if (event.code === 'ArrowLeft') { event.preventDefault(); stepBy(-1); }
});
for (let value = 1; value <= 9; value++) $('shortest-case').append(new Option(`case ${value}`, value));
$('run-button').disabled = true;
api('/api/catalog').then(async (catalog) => {
  state.catalog = catalog; selectFamily('half'); status('迷路を選んで比較を実行してください。');
  try {
    const replay = await api('/api/replay');
    if (replay) restoreReplay(replay);
  } catch (error) { status(`保存リプレイを読み込めません: ${error.message}`, 'error'); }
}).catch((error) => status(`データを読み込めません: ${error.message}`, 'error'));
