// Hub Control Center page. Talks only to this server's /api/*; builds every
// element with createElement (no innerHTML on data), polls state at 5 Hz.
'use strict';

const SVGNS = 'http://www.w3.org/2000/svg';
const H = 8;                                  // map height (m): SVG y = H - y
const POLL_MS = 200;
const EXAMPLES = ['rush hour reactive seed 3', 'robot escort with 2 trials then S9',
  'test the e-stop', 'parking then congestion', 'real video without stabilisation',
  'unit tests for routing', 'what can you do'];

const $ = (sel) => document.querySelector(sel);
function el(tag, attrs = {}, ...kids) {
  const n = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (v === null || v === undefined || v === false) continue;
    if (k === 'class') n.className = v;
    else if (k === 'text') n.textContent = v;
    else if (k.startsWith('on')) n.addEventListener(k.slice(2), v);
    else n.setAttribute(k, v === true ? '' : v);
  }
  for (const c of kids.flat()) if (c !== null && c !== undefined) n.append(c);
  return n;
}
function svg(tag, attrs = {}, ...kids) {
  const n = document.createElementNS(SVGNS, tag);
  for (const [k, v] of Object.entries(attrs)) {
    if (k === 'text') n.textContent = v; else if (k.startsWith('on')) n.addEventListener(k.slice(2), v);
    else n.setAttribute(k, v);
  }
  for (const c of kids.flat()) if (c) n.append(c);
  return n;
}
async function api(path, body) {
  const opt = body === undefined ? {} : {
    method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(body)};
  const r = await fetch(path, opt);
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.error || r.statusText);
  return data;
}
const fmtTime = (s) => s >= 60 ? `${Math.floor(s / 60)}m ${Math.round(s % 60)}s` : `${s.toFixed(0)}s`;
const fmtOpts = (o) => Object.entries(o || {}).map(([k, v]) => `${k}=${v}`).join(' · ') || 'defaults';

let CATALOG = [];
let MAP = null;
let lastHistoryTop = null;
let selectedRun = null;

// ------------------------------------------------------------------ tabs
document.querySelectorAll('.tabs button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('.tabs button').forEach((x) => x.setAttribute('aria-selected', x === b));
  for (const t of ['run', 'history', 'explain']) $('#tab-' + t).hidden = t !== b.dataset.tab;
}));

// ----------------------------------------------------------------- tasks
function optionInput(o) {
  if (o.type === 'bool') {
    return el('label', {class: 'check', title: o.help},
      el('input', {type: 'checkbox', name: o.name, checked: o.default}), o.label);
  }
  const input = o.type === 'choice'
    ? el('select', {name: o.name}, o.choices.map((c) => el('option', {value: c, text: c, selected: c === o.default})))
    : el('input', {type: 'number', name: o.name, value: o.default, min: o.min, max: o.max, step: 1});
  return el('label', {title: o.help}, o.label, input);
}
function readOptions(form, task) {
  const out = {};
  for (const o of task.options) {
    const f = form.elements[o.name];
    out[o.name] = o.type === 'bool' ? f.checked : o.type === 'int' ? Number(f.value) : f.value;
  }
  return out;
}
function renderTasks() {
  const list = $('#task-list');
  const tpl = $('#tpl-task');
  let group = '';
  list.replaceChildren();
  for (const t of CATALOG) {
    if (t.group !== group) { group = t.group; list.append(el('div', {class: 'group', text: group})); }
    const node = tpl.content.firstElementChild.cloneNode(true);
    node.dataset.search = [t.id, t.title, t.owner, t.group, ...t.keywords].join(' ').toLowerCase();
    node.querySelector('.tid').textContent = t.id;
    node.querySelector('.ttl').textContent = t.title;
    node.querySelector('.who').textContent = t.owner;
    node.querySelector('.sum').textContent = t.summary;
    const form = node.querySelector('.opts');
    t.options.forEach((o) => form.append(optionInput(o)));
    node.querySelector('.mins').textContent = t.available
      ? `about ${t.minutes} min${t.live ? ' · live map' : ''}`
      : `needs ${t.missing.join(', ')}`;
    if (!t.available) node.classList.add('unavailable');
    node.querySelector('.run').addEventListener('click', async (e) => {
      e.preventDefault();
      try { await api('/api/run', {task: t.id, options: readOptions(form, t), source: 'form'}); }
      catch (err) { alert(err.message); }
    });
    list.append(node);
  }
}
$('#filter').addEventListener('input', (e) => {
  const q = e.target.value.trim().toLowerCase();
  document.querySelectorAll('#task-list .task').forEach((n) => { n.hidden = q && !n.dataset.search.includes(q); });
  document.querySelectorAll('#task-list .group').forEach((g) => { g.hidden = !!q; });
});

// ------------------------------------------------------------------- ask
EXAMPLES.forEach((x) => $('#examples').append(el('button', {type: 'button', text: x,
  onclick: () => { $('#ask').value = x; planRequest(); }})));
$('#ask-form').addEventListener('submit', (e) => { e.preventDefault(); planRequest(); });

async function planRequest() {
  const box = $('#plan');
  box.hidden = false;
  box.replaceChildren(el('p', {class: 'muted', text: 'thinking…'}));
  let res;
  try { res = await api('/api/plan', {text: $('#ask').value}); }
  catch (err) { box.replaceChildren(el('p', {text: err.message})); return; }
  const steps = res.plan.map((p, i) => el('div', {class: 'step' + (p.confidence === 'low' ? ' low' : '')},
    el('div', {}, el('b', {text: `${i + 1}. ${p.task}`}), p.title),
    el('small', {text: fmtOpts(p.options)}),
    el('small', {text: `${p.confidence} confidence · ${p.reason}`}),
    p.notes.map((n) => el('small', {text: '⚠ ' + n})),
    p.alternatives.length ? el('small', {}, 'or: ', p.alternatives.map((a) => el('button', {
      type: 'button', text: a, onclick: () => { $('#ask').value = a; planRequest(); }}))) : null));
  const run = res.plan.length ? el('button', {class: 'primary', text: `Run ${res.plan.length > 1 ? 'all ' + res.plan.length : ''}`,
    onclick: async () => {
      try { await api('/api/run', {plan: res.plan.map((p) => ({task: p.task, options: p.options})), source: 'ask'}); box.hidden = true; }
      catch (err) { alert(err.message); }
    }}) : null;
  box.replaceChildren(el('p', {class: 'msg', text: res.message}), ...steps, run);
}

// ------------------------------------------------------------------- map
function drawStaticMap() {
  const m = $('#map');
  const [x0, y0, x1, y1] = MAP.site;
  m.replaceChildren(svg('rect', {class: 'site', x: x0, y: H - y1, width: x1 - x0, height: y1 - y0}));
  const rect = (r, cls, id) => svg('rect', {class: cls, id, x: r[0], y: H - r[3], width: r[2] - r[0], height: r[3] - r[1]});
  m.append(rect(MAP.queue, 'zone-q'), svg('text', {x: MAP.queue[0] + 0.1, y: H - MAP.queue[3] + 0.35, text: 'Q queue'}));
  for (const [id, e] of Object.entries(MAP.edges)) {
    m.append(svg('line', {class: 'edge' + (e.road ? ' road' : ''), id: 'e-' + id,
      x1: e.a[0], y1: H - e.a[1], x2: e.b[0], y2: H - e.b[1]}));
  }
  m.append(rect(MAP.crossing, 'zone-c', 'zone-c'), svg('text', {class: 'label', x: MAP.crossing[0] + 0.32, y: H - MAP.crossing[1] - 0.15, text: 'C'}));
  const [sw, sh] = MAP.slot_size;
  for (const s of ['P1', 'P2', 'P3']) {
    const [x, y] = MAP.nodes[s];
    m.append(svg('rect', {class: 'slot', id: 'slot-' + s, x: x - sw / 2, y: H - y - sh / 2, width: sw, height: sh, rx: 0.08}));
  }
  for (const [n, [x, y]] of Object.entries(MAP.nodes)) {
    m.append(svg('text', {class: n.startsWith('P') ? 'label' : 'node', x: x + 0.12, y: H - y - 0.12, text: n}));
  }
  m.append(svg('g', {id: 'dyn'}));
}
function drawLive(live) {
  const veh = live.vehicles;
  const any = veh && !veh.stale;
  $('#map').classList.toggle('stale', !any);
  $('#live-note').textContent = any ? 'live' : veh ? `last data ${veh.age.toFixed(0)} s ago` : 'waiting for a run…';
  const edges = live.edges && !live.edges.stale ? live.edges.value : {};
  for (const id of Object.keys(MAP.edges)) {
    const e = edges[id];
    const line = document.getElementById('e-' + id);
    if (!line) continue;
    line.classList.toggle('blocked', !!(e && e.blocked));
    line.classList.toggle('dense', !!(e && !e.blocked && e.valid && e.density >= 0.7));
    line.classList.toggle('unknown', !!(e && !e.valid));
  }
  const c = live.crossing && !live.crossing.stale ? live.crossing.value : null;
  const zc = document.getElementById('zone-c');
  zc.classList.toggle('granted', !!(c && c.owner));
  zc.classList.toggle('ped', !!(c && c.pedestrian));
  const slots = live.parking && !live.parking.stale ? live.parking.value : {};
  for (const s of ['P1', 'P2', 'P3']) {
    document.getElementById('slot-' + s).setAttribute('class', 'slot ' + ((slots[s] || {}).state || ''));
  }
  const dyn = $('#dyn');
  const kids = [];
  for (const [agent, pts] of Object.entries(live.routes || {})) {
    if (pts.length > 1) kids.push(svg('polyline', {class: 'route ' + agent, points: pts.map(([x, y]) => `${x},${H - y}`).join(' ')}));
  }
  const r = MAP.radius;
  for (const p of (live.pedestrians && !live.pedestrians.stale ? live.pedestrians.value : [])) {
    kids.push(svg('circle', {class: 'agent pedestrian', cx: p.x, cy: H - p.y, r: 0.14}));
  }
  for (const v of (any ? veh.value : [])) {
    const kind = r[v.kind] ? v.kind : 'car';
    const rad = r[kind];
    const g = svg('g', {class: 'agent ' + kind + (v.valid ? '' : ' invalid')},
      svg('title', {text: `${v.id} · ${v.speed.toFixed(2)} m/s${v.edge ? ' · ' + v.edge : ''}`}),
      svg('circle', {cx: v.x, cy: H - v.y, r: rad}));
    if (kind !== 'obstacle') {
      g.append(svg('line', {class: 'heading', x1: v.x, y1: H - v.y,
        x2: v.x + Math.cos(v.theta) * rad, y2: H - v.y - Math.sin(v.theta) * rad}));
    }
    kids.push(g);
  }
  dyn.replaceChildren(...kids);
}

// ----------------------------------------------------------------- tiles
function tile(k, v, s, cls = '', wide = false) {
  return el('div', {class: `tile ${cls}${wide ? ' wide' : ''}`}, el('div', {class: 'k', text: k}),
    el('div', {class: 'v', text: v}), el('div', {class: 's', text: s}));
}
const val = (x) => (x && !x.stale ? x.value : null);
function drawTiles(live) {
  const s = val(live.safety);
  const estop = val(live.estop);
  const c = val(live.crossing);
  const q = val(live.queue);
  const d = val(live.demand);
  const slots = val(live.parking);
  const veh = val(live.vehicles) || [];
  const safety = !s ? tile('Safety', '—', 'no supervisor data')
    : s.stop_latched ? tile('Safety', 'LATCHED', s.reason || 'stop latched', 'bad', true)
      : !s.motion_permitted ? tile('Safety', 'HOLD', s.reason || 'motion held', 'warn', true)
        : tile('Safety', 'OK', 'motion permitted', 'ok', true);
  const speed = (kind) => { const v = veh.find((x) => x.kind === kind); return v ? `${v.speed.toFixed(2)} m/s` : '—'; };
  const free = slots ? Object.values(slots).filter((x) => x.state === 'FREE').length : null;
  $('#tiles').replaceChildren(
    safety,
    tile('E-stop', estop === null ? '—' : estop ? 'PRESSED' : 'released', '', estop ? 'bad' : estop === false ? 'ok' : ''),
    tile('Crossing C', c ? c.state || 'FREE' : '—', c ? (c.owner ? 'owner ' + c.owner : 'no owner') + (c.pedestrian ? ' · person in C' : '') : '',
      c ? (c.pedestrian ? 'warn' : 'ok') : ''),
    tile('Queue Q', q ? String(q.count) : '—', q ? `confidence ${q.confidence}${q.valid ? '' : ' · invalid'}` : '', q ? (q.valid ? 'ok' : 'warn') : ''),
    tile('Forecast +30 s', d && d.valid ? d.predicted.toFixed(1) : '—', d ? (d.valid ? `persistence ${d.baseline.toFixed(1)}` : 'warming up / invalid') : '',
      d ? (d.valid ? 'ok' : 'warn') : ''),
    tile('Parking', free === null ? '—' : `${free}/3 free`, slots ? Object.entries(slots).map(([k, x]) => `${k} ${x.state}`).join(' · ') : '', slots ? 'ok' : ''),
    tile('Bus', speed('bus'), '', veh.some((x) => x.kind === 'bus') ? 'ok' : ''),
    tile('Robot', speed('robot'), '', veh.some((x) => x.kind === 'robot') ? 'ok' : ''));
  const ms = Object.entries(live.missions || {});
  $('#missions').replaceChildren(...(ms.length ? ms.map(([k, m]) => el('div', {}, el('b', {text: k}), ` ${m.event}`,
    m.reason ? el('span', {class: 'muted', text: ' · ' + m.reason}) : null)) : [document.createTextNode('none yet')]));
}

// ------------------------------------------------------------------ runs
const badge = (s) => el('span', {class: 'badge ' + s, text: s});
function drawRuns(runs) {
  const a = runs.active;
  $('#stop').disabled = !a;
  if (a) {
    const t = CATALOG.find((x) => x.id === a.task);
    const est = (a.options.run_for || (t ? t.minutes * 60 : 60));
    const frac = Math.min(1, a.elapsed / est);
    $('#active').className = '';
    $('#active').replaceChildren(
      el('div', {class: 'row'}, el('span', {class: 'title', text: `${a.task} · ${a.title}`}), badge('running')),
      el('div', {class: 'muted', text: `${a.owner} · ${fmtOpts(a.options)}`}),
      el('div', {class: 'bar'}, el('i', {})),
      el('div', {class: 'muted mono', text: `${fmtTime(a.elapsed)} of about ${fmtTime(est)} · evidence in ${a.out_dir}`}));
    $('#active .bar i').style.transform = `scaleX(${frac})`;
    const log = $('#log');
    const atEnd = log.scrollTop + log.clientHeight >= log.scrollHeight - 20;
    log.textContent = a.log.join('\n');
    if (atEnd) log.scrollTop = log.scrollHeight;
  } else {
    $('#active').replaceChildren(document.createTextNode('Nothing running. Pick a task or ask.'));
    $('#active').className = 'muted';
  }
  $('#queue').replaceChildren(...runs.queue.map((q, i) => el('div', {class: 'queued'},
    `next ${i + 1}: ${q.task} (${fmtOpts(q.options)}) `,
    el('button', {type: 'button', text: 'remove', onclick: () => api('/api/stop', {id: q.id})}))));
  const last = runs.history[0];
  if (last && last.id !== lastHistoryTop) { lastHistoryTop = last.id; drawVerdict(last); drawHistory(runs.history); }
}
function drawVerdict(r) {
  $('#verdict').className = '';
  $('#verdict').replaceChildren(
    el('div', {class: 'row'}, el('b', {text: `${r.task} · ${r.title}`}), badge(r.status)),
    el('div', {class: 'muted', text: `${fmtOpts(r.options)} · ${fmtTime(r.elapsed)}`}),
    el('p', {text: r.summary}),
    el('ul', {class: 'checks'}, (r.checks || []).map((c) => el('li', {class: c.passed ? '' : 'no'},
      el('span', {text: c.check}), c.detail ? el('small', {text: c.detail}) : null))));
}
function drawHistory(hist) {
  $('#history tbody').replaceChildren(...hist.map((r) => el('tr', {},
    el('td', {class: 'mono', text: new Date((r.started_at || r.queued_at) * 1000).toLocaleString()}),
    el('td', {}, el('b', {text: r.task}), ' ' + r.title),
    el('td', {class: 'muted', text: fmtOpts(r.options)}),
    el('td', {}, badge(r.status), ' ', r.summary),
    el('td', {class: 'mono', text: fmtTime(r.elapsed || 0)}),
    el('td', {}, el('button', {type: 'button', text: 'log', onclick: () => showLog(r.id)}),
      ' ', el('button', {type: 'button', text: 'details', onclick: () => { drawVerdict(r); document.querySelector('[data-tab=run]').click(); }})))));
}
async function showLog(id) {
  const pre = $('#history-log');
  pre.hidden = false;
  pre.textContent = 'loading…';
  selectedRun = id;
  const r = await api('/api/log?id=' + encodeURIComponent(id));
  if (selectedRun === id) pre.textContent = r.log || '(no log)';
}
$('#stop').addEventListener('click', () => api('/api/stop', {}).catch((e) => alert(e.message)));

// ---------------------------------------------------------------- explain
const GRAPH = {
  scenario_runner: {x: 20, y: 220, owner: 'M5 Saidumar', lane: 'world',
    what: 'Deterministic 20 Hz simulator: moves bus, robot, cars and people, renders the synthetic camera, injects faults, serves /scenario/reset.',
    in: ['/robot/cmd_vel_applied', '/bus/cmd_vel_applied', '/scenario/inject'],
    out: ['/hub/vehicles', '/robot/odom', '/camera/image_raw', '/sim/truth (evaluation only)', '/hub/estop']},
  passenger_detector: {x: 220, y: 40, owner: 'M1-A Husan', lane: 'sense',
    what: 'Detects people, projects their feet onto the map with a homography, tracks them and counts queue Q; flags a person in crossing C.',
    in: ['/camera/image_raw'], out: ['/hub/queue (5 Hz)']},
  demand_predictor: {x: 220, y: 150, owner: 'M1-B Tuymurod', lane: 'sense',
    what: 'Causal ridge model: 30 s demand forecast from the last 60 s of queue counts, with a persistence baseline and a validity flag.',
    in: ['/hub/queue'], out: ['/hub/demand (1 Hz)']},
  traffic_monitor: {x: 220, y: 280, owner: 'M3 Carla', lane: 'sense',
    what: 'Labels every road FREE, DENSE, BLOCKED or UNKNOWN from vehicle speeds and density, with stale detection.',
    in: ['/hub/vehicles'], out: ['/traffic/edges']},
  parking_manager: {x: 220, y: 390, owner: 'M3 Carla', lane: 'decide',
    what: 'Atomic, idempotent slot reservations with tokens and expiry; sensed state after expiry.',
    in: ['/parking/observed', 'ReserveParking / ReleaseParking'], out: ['/parking/state']},
  hub_coordinator: {x: 420, y: 95, owner: 'M2-B Arkar', lane: 'decide',
    what: 'Dispatches the bus from the forecast (hysteresis, FALLBACK), owns crossing C (one owner, leases, release), triggers escorts.',
    in: ['/hub/demand', '/hub/queue', '/crossing/request'], out: ['/bus/dock (action goal)', '/robot/escort (action goal)', '/crossing/state']},
  route_planner: {x: 420, y: 280, owner: 'M4 Aidil', lane: 'decide',
    what: 'Dijkstra over valid edges only: removes blocked, stale, forbidden and obstacle edges; cost = length / speed × (1 + 2·density) + wait.',
    in: ['/traffic/edges', '/hub/vehicles', '/crossing/state'], out: ['/route/compute (service)', '/route/edge_costs']},
  bus_manager: {x: 620, y: 95, owner: 'M2-A Fazliddin', lane: 'act',
    what: 'DockBus action server: route, crossing acquire/renew/release, slow final approach, 1 s docked hold, door interlock.',
    in: ['/bus/dock', '/route/compute', '/crossing/request'], out: ['/bus/cmd_vel_candidate', '/bus/door_request']},
  robot_manager: {x: 620, y: 280, owner: 'M4 Aidil', lane: 'act',
    what: 'EscortPassenger action server: to pickup, wait for /robot/confirm_ready (20 s), escort, reroute on blocked roads.',
    in: ['/robot/escort', '/route/compute', '/robot/odom'], out: ['/robot/cmd_vel_candidate']},
  safety_supervisor: {x: 810, y: 190, owner: 'M5 Saidumar', lane: 'safety',
    what: 'The only node that sends motor commands. Every 50 ms: e-stop, freshness, clearance, door, crossing, site, speed caps. Latch or hold.',
    in: ['/robot/cmd_vel_candidate', '/bus/cmd_vel_candidate', '/hub/estop', '/hub/vehicles'], out: ['/robot/cmd_vel', '/bus/cmd_vel', '/hub/safety', '/safety/clear_stop (service)']},
  actuator_bridge: {x: 810, y: 390, owner: 'M5 Saidumar', lane: 'safety',
    what: 'Motor-side watchdog: zero after 0.5 s without a final command, its own speed caps and door interlock.',
    in: ['/robot/cmd_vel', '/bus/cmd_vel'], out: ['/robot/cmd_vel_applied', '/bus/cmd_vel_applied']},
  observers: {x: 20, y: 40, owner: 'M5 Saidumar', lane: 'observe', label: 'metrics + dashboards',
    what: 'metrics_logger, the RViz marker dashboard and this Control Center: they only listen, never command (the Control Center starts runs like a terminal would).',
    in: ['all state topics'], out: ['CSV/JSON evidence', '/hub/markers']},
};
const LINKS = [['scenario_runner', 'passenger_detector'], ['passenger_detector', 'demand_predictor'],
  ['scenario_runner', 'traffic_monitor'], ['demand_predictor', 'hub_coordinator'],
  ['traffic_monitor', 'route_planner'], ['hub_coordinator', 'bus_manager'], ['hub_coordinator', 'robot_manager'],
  ['route_planner', 'bus_manager'], ['route_planner', 'robot_manager'],
  ['bus_manager', 'safety_supervisor', 1], ['robot_manager', 'safety_supervisor', 1],
  ['safety_supervisor', 'actuator_bridge', 1], ['actuator_bridge', 'scenario_runner', 1],
  ['scenario_runner', 'observers'], ['parking_manager', 'hub_coordinator']];
function drawGraph() {
  const g = $('#graph');
  const W = 170;
  const NH = 52;
  const BOTTOM = 500;
  const mk = (id, color) => svg('marker', {id, viewBox: '0 0 10 10', refX: 9, refY: 5, markerWidth: 7, markerHeight: 7, orient: 'auto-start-reverse'},
    svg('path', {d: 'M0,0 L10,5 L0,10 z', fill: color}));
  g.replaceChildren(svg('defs', {}, mk('arrow', '#8b96a1'), mk('arrow-red', '#c42b2b')));
  for (const [x, label] of [[20, 'world'], [220, 'sense'], [420, 'decide'], [620, 'act'], [810, 'safety']]) {
    g.append(svg('text', {class: 'lane', x, y: 22, text: label}));
  }
  for (const [a, b, safety] of LINKS) {
    const A = GRAPH[a];
    const B = GRAPH[b];
    let d;
    if (A.x === B.x) {                       // same column: straight up or down
      const down = B.y > A.y;
      const x = A.x + W / 2;
      d = `M${x},${down ? A.y + NH : A.y} L${x},${down ? B.y : B.y + NH}`;
    } else if (B.x > A.x) {                   // forward: right side to left side
      const [x1, y1, x2, y2] = [A.x + W, A.y + NH / 2, B.x, B.y + NH / 2];
      const mx = (x1 + x2) / 2;
      d = `M${x1},${y1} C${mx},${y1} ${mx},${y2} ${x2},${y2}`;
    } else {                                  // feedback: along the bottom edge
      const [x1, x2] = [A.x + W / 2, B.x + W / 2];
      d = `M${x1},${A.y + NH} L${x1},${BOTTOM} L${x2},${BOTTOM} L${x2},${B.y + NH}`;
    }
    g.append(svg('path', {class: 'gedge' + (safety ? ' safety' : ''), d}));
  }
  for (const [id, p] of Object.entries(GRAPH)) {
    const node = svg('g', {class: 'gnode', tabindex: 0, role: 'button', 'aria-label': id,
      onclick: () => selectNode(id, node), onkeydown: (e) => { if (e.key === 'Enter') selectNode(id, node); }},
    svg('rect', {x: p.x, y: p.y, width: W, height: NH}),
    svg('text', {x: p.x + 12, y: p.y + 22, text: p.label || id}),
    svg('text', {class: 'own', x: p.x + 12, y: p.y + 40, text: p.owner}));
    g.append(node);
  }
  g.append(svg('text', {class: 'lane', x: 300, y: BOTTOM - 8, text: 'applied commands drive the simulator (third stop layer)'}));
}
function selectNode(id, node) {
  document.querySelectorAll('#graph .gnode').forEach((n) => n.classList.toggle('sel', n === node));
  const p = GRAPH[id];
  $('#node-info').className = 'node-info';
  $('#node-info').replaceChildren(el('h3', {text: p.label || id}), el('div', {class: 'muted', text: p.owner}),
    el('p', {text: p.what}), el('b', {text: 'Listens to'}), el('ul', {}, p.in.map((t) => el('li', {}, el('code', {text: t})))),
    el('b', {text: 'Says'}), el('ul', {}, p.out.map((t) => el('li', {}, el('code', {text: t})))));
}

// ------------------------------------------------------------------ loop
async function poll() {
  try {
    const s = await api('/api/state');
    $('#conn').textContent = 'connected';
    $('#conn').className = 'pill ok';
    $('#commit').textContent = 'commit ' + s.commit;
    drawLive(s.live);
    drawTiles(s.live);
    drawRuns(s.runs);
  } catch (err) {
    $('#conn').textContent = 'server offline';
    $('#conn').className = 'pill bad';
  }
  setTimeout(poll, POLL_MS);
}
(async function init() {
  const [cat, map] = await Promise.all([api('/api/catalog'), api('/api/map')]);
  CATALOG = cat.tasks;
  MAP = map;
  renderTasks();
  drawStaticMap();
  drawGraph();
  poll();
})();
