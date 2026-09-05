"use strict";

const state = {
  scenarioId: null,
  runId: null,
  sections: [],
  officialGrid: [],
  summary: {},
  track: [],
  mapData: { checkpoints: [], route: [] },
  bounds: null,
  cursor: 0,          // index into track currently displayed
  liveFollow: true,   // auto-jump to newest frame as data arrives
  livePolling: true,
  playing: false,
  playSpeed: 1,
  runIsLive: false,
  pollTimer: null,
  playTimer: null,
};

const el = (id) => document.getElementById(id);

async function getJSON(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url} -> ${res.status}`);
  return res.json();
}

function fmt(value, digits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) return "-";
  return Number(value).toFixed(digits);
}

/* ---------- scenario / run selection ---------- */

async function loadScenarios() {
  const scenarios = await getJSON("/api/scenarios");
  const select = el("scenarioSelect");
  select.innerHTML = "";
  for (const s of scenarios) {
    const opt = document.createElement("option");
    opt.value = s.scenario_id;
    opt.textContent = `${s.scenario_id} (${s.category}, 실행 ${s.run_count}건)`;
    select.appendChild(opt);
  }
  if (scenarios.length) {
    select.value = scenarios.find((s) => s.run_count > 0)?.scenario_id || scenarios[0].scenario_id;
    await onScenarioChange();
  }
}

async function loadRuns(scenarioId) {
  const runs = await getJSON(`/api/runs?scenario_id=${encodeURIComponent(scenarioId)}`);
  const select = el("runSelect");
  select.innerHTML = "";
  for (const r of runs) {
    const opt = document.createElement("option");
    opt.value = r.run_id;
    const when = new Date(r.mtime * 1000).toLocaleString("ko-KR");
    opt.textContent = `${r.run_id}${r.live ? " [LIVE]" : ""} - ${when}`;
    select.appendChild(opt);
  }
  return runs;
}

async function onScenarioChange() {
  state.scenarioId = el("scenarioSelect").value;
  const [runs, mapData] = await Promise.all([
    loadRuns(state.scenarioId),
    getJSON(`/api/map?scenario_id=${encodeURIComponent(state.scenarioId)}`),
  ]);
  state.mapData = mapData;
  el("scenarioMeta").textContent = mapData.meta
    ? `${mapData.meta.name || ""} · ${mapData.meta.map || ""}`
    : "";
  if (runs.length) {
    el("runSelect").value = runs[0].run_id;
    await onRunChange();
  } else {
    resetRunView();
  }
}

async function onRunChange() {
  state.runId = el("runSelect").value;
  state.track = [];
  state.cursor = 0;
  state.liveFollow = true;
  await refreshRun(true);
}

function resetRunView() {
  state.runId = null;
  state.track = [];
  state.officialGrid = [];
  state.sections = [];
  renderChecklist();
  renderStats(null);
  drawCanvas();
}

/* ---------- run data polling ---------- */

async function refreshRun(fullReload = false) {
  if (!state.scenarioId || !state.runId) return;
  const since = fullReload ? -1 : (state.track.length ? state.track[state.track.length - 1].frame : -1);
  let payload;
  try {
    payload = await getJSON(
      `/api/run?scenario_id=${encodeURIComponent(state.scenarioId)}&run_id=${encodeURIComponent(state.runId)}&since=${since}`
    );
  } catch (err) {
    console.error(err);
    return;
  }

  if (fullReload) {
    state.track = payload.track;
  } else if (payload.track.length) {
    state.track = state.track.concat(payload.track);
  }

  state.sections = payload.sections;
  state.officialGrid = payload.official_grid;
  state.summary = payload.summary;

  const runs = await getRunsCache();
  const runInfo = runs.find((r) => r.run_id === state.runId);
  state.runIsLive = !!(runInfo && runInfo.live);
  updateLiveBadge();

  if (state.liveFollow) {
    state.cursor = state.track.length - 1;
  }

  el("scrubber").max = Math.max(0, state.track.length - 1);
  if (state.cursor > state.track.length - 1) state.cursor = state.track.length - 1;
  el("scrubber").value = state.cursor;

  renderChecklist();
  renderStats(currentPoint());
  renderScoreBar();
  drawCanvas();
}

let runsCache = { scenarioId: null, runs: [] };
async function getRunsCache() {
  if (runsCache.scenarioId !== state.scenarioId) {
    runsCache = { scenarioId: state.scenarioId, runs: await loadRunsQuiet(state.scenarioId) };
  }
  return runsCache.runs;
}
async function loadRunsQuiet(scenarioId) {
  try {
    return await getJSON(`/api/runs?scenario_id=${encodeURIComponent(scenarioId)}`);
  } catch {
    return [];
  }
}

function updateLiveBadge() {
  const badge = el("liveBadge");
  if (!state.runId) {
    badge.textContent = "대기중";
    badge.className = "badge badge-idle";
  } else if (state.runIsLive) {
    badge.textContent = "실행 중";
    badge.className = "badge badge-live";
  } else {
    badge.textContent = "완료";
    badge.className = "badge badge-done";
  }
}

function currentPoint() {
  if (!state.track.length) return null;
  const idx = Math.min(Math.max(state.cursor, 0), state.track.length - 1);
  return state.track[idx];
}

/* ---------- stats row ---------- */

function renderStats(point) {
  if (!point) {
    el("statSection").textContent = "-";
    el("statElapsed").innerHTML = '0.0<span class="unit">초</span>';
    el("statRisk").textContent = "0";
    el("statSpeed").innerHTML = '0.0<span class="unit">km/h</span>';
    el("statLongDist").innerHTML = '0.0<span class="unit">m</span>';
    el("statTtc").innerHTML = '-<span class="unit">초</span>';
    el("statScoredAt").textContent = "-";
    el("statSignal").textContent = "-";
    return;
  }
  el("statSection").textContent = point.section === "unsegmented" ? "전체" : (point.section || "-");
  el("statElapsed").innerHTML = `${fmt(point.t, 1)}<span class="unit">초</span>`;

  const collisions = state.track.slice(0, state.cursor + 1).filter((p) => p.collision).length;
  const riskEl = el("statRisk");
  riskEl.textContent = String(collisions);
  riskEl.classList.toggle("risk-active", collisions > 0);

  el("statSpeed").innerHTML = `${fmt(point.speed !== null ? point.speed * 3.6 : null, 1)}<span class="unit">km/h</span>`;
  el("statLongDist").innerHTML = `${fmt(point.obj_long, 1)}<span class="unit">m</span>`;
  el("statTtc").innerHTML = `${fmt(point.ttc, 2)}<span class="unit">초</span>`;
  el("statScoredAt").textContent = new Date().toLocaleTimeString("ko-KR");
  el("statSignal").textContent = point.signal || "없음";

  el("frameReadout").textContent = `frame ${point.frame} / ${state.track.length ? state.track[state.track.length - 1].frame : 0}`;
  el("overlayTime").textContent = formatClock(point.t);
  el("overlaySignal").textContent = `신호 ${point.signal || "-"}`;
}

function formatClock(seconds) {
  if (seconds === null || seconds === undefined) return "00:00.0";
  const m = Math.floor(seconds / 60);
  const s = seconds - m * 60;
  return `${String(m).padStart(2, "0")}:${s.toFixed(1).padStart(4, "0")}`;
}

/* ---------- checklist ---------- */

const STATUS_CHIP = {
  PASS: ["PASS", "chip-pass"],
  MINOR: ["MINOR", "chip-minor"],
  MAJOR: ["MAJOR", "chip-major"],
  UNAVAILABLE: ["-", "chip-unavailable"],
  NOT_APPLICABLE: ["N/A", "chip-na"],
};

function chipFor(status) {
  const [label, cls] = STATUS_CHIP[status] || ["-", "chip-unavailable"];
  return `<span class="chip ${cls}">${label}</span>`;
}

function renderChecklist() {
  const head = el("checklistHead");
  head.innerHTML = '<th class="col-item">항목</th>';
  const sections = state.sections.length ? state.sections : [];
  for (const s of sections) {
    const th = document.createElement("th");
    th.textContent = s === "unsegmented" ? "전체" : s;
    head.appendChild(th);
  }

  const body = el("checklistBody");
  body.innerHTML = "";
  for (const item of state.officialGrid) {
    const tr = document.createElement("tr");
    const nameTd = document.createElement("td");
    nameTd.className = "col-item";
    nameTd.textContent = `${item.number}. ${item.label}`;
    tr.appendChild(nameTd);

    for (const s of sections) {
      const td = document.createElement("td");
      let cellStatus;
      if (item.status === "UNAVAILABLE" || item.status === "NOT_APPLICABLE") {
        cellStatus = item.status;
      } else {
        const sev = item.by_section[s];
        cellStatus = sev === 2 ? "MAJOR" : sev === 1 ? "MINOR" : "PASS";
      }
      td.innerHTML = chipFor(cellStatus);
      tr.appendChild(td);
    }
    body.appendChild(tr);
  }
}

/* ---------- score bar ---------- */

function renderScoreBar() {
  const s = state.summary || {};
  el("totalScore").textContent = s.official_score || "-";
  const violations = (s.official_violations || "").split(";").filter(Boolean);
  const unavailable = (s.official_unavailable_items || "").split(";").filter(Boolean);
  el("scoreViolations").textContent = `위반 ${violations.length}건`;
  el("scoreUnavailable").textContent = `미평가 ${unavailable.length}건`;
}

el("reportBtn").addEventListener("click", () => {
  el("reportContent").textContent = JSON.stringify({ summary: state.summary, official_grid: state.officialGrid }, null, 2);
  el("reportModal").classList.remove("hidden");
});
el("closeReport").addEventListener("click", () => el("reportModal").classList.add("hidden"));

/* ---------- map canvas ---------- */

function computeBounds() {
  const xs = [];
  const ys = [];
  for (const p of state.mapData.route || []) { xs.push(p.x); ys.push(p.y); }
  for (const p of state.mapData.checkpoints || []) { xs.push(p.x); ys.push(p.y); }
  for (const p of state.track) { if (p.x !== null) { xs.push(p.x); ys.push(p.y); } }
  if (!xs.length) return null;
  const minX = Math.min(...xs), maxX = Math.max(...xs);
  const minY = Math.min(...ys), maxY = Math.max(...ys);
  const pad = Math.max(5, (maxX - minX) * 0.08, (maxY - minY) * 0.08);
  return { minX: minX - pad, maxX: maxX + pad, minY: minY - pad, maxY: maxY + pad };
}

function toScreen(canvas, bounds, x, y) {
  const w = canvas.width, h = canvas.height;
  const rangeX = bounds.maxX - bounds.minX || 1;
  const rangeY = bounds.maxY - bounds.minY || 1;
  const scale = Math.min(w / rangeX, h / rangeY);
  const offX = (w - rangeX * scale) / 2;
  const offY = (h - rangeY * scale) / 2;
  return {
    sx: offX + (x - bounds.minX) * scale,
    sy: h - (offY + (y - bounds.minY) * scale),
  };
}

function drawCanvas() {
  const canvas = el("mapCanvas");
  const wrap = canvas.parentElement;
  canvas.width = wrap.clientWidth;
  canvas.height = wrap.clientHeight;
  const ctx = canvas.getContext("2d");
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  const bounds = computeBounds();
  if (!bounds) return;

  // route path
  const route = state.mapData.route || [];
  if (route.length) {
    ctx.strokeStyle = "#2b3247";
    ctx.lineWidth = 3;
    ctx.beginPath();
    route.forEach((p, i) => {
      const { sx, sy } = toScreen(canvas, bounds, p.x, p.y);
      i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }

  // checkpoints
  ctx.fillStyle = "#4b5266";
  ctx.font = "11px sans-serif";
  for (const cp of state.mapData.checkpoints || []) {
    const { sx, sy } = toScreen(canvas, bounds, cp.x, cp.y);
    ctx.beginPath();
    ctx.arc(sx, sy, 4, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillText(String(cp.seq), sx + 6, sy - 6);
  }

  // traveled trail
  const upto = state.track.slice(0, state.cursor + 1).filter((p) => p.x !== null);
  if (upto.length > 1) {
    ctx.strokeStyle = "#22d3ee";
    ctx.lineWidth = 2.5;
    ctx.beginPath();
    upto.forEach((p, i) => {
      const { sx, sy } = toScreen(canvas, bounds, p.x, p.y);
      i === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }

  const point = currentPoint();
  if (point && point.x !== null) {
    const { sx, sy } = toScreen(canvas, bounds, point.x, point.y);

    // nearest object
    if (point.obj_x !== null && point.obj_y !== null) {
      const obj = toScreen(canvas, bounds, point.obj_x, point.obj_y);
      ctx.strokeStyle = "rgba(239,68,68,0.5)";
      ctx.lineWidth = 1;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(obj.sx, obj.sy);
      ctx.stroke();
      ctx.fillStyle = "#ef4444";
      ctx.beginPath();
      ctx.arc(obj.sx, obj.sy, 5, 0, Math.PI * 2);
      ctx.fill();
    }

    // ego marker with heading
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-(point.heading || 0));
    ctx.fillStyle = point.collision ? "#ef4444" : "#22d3ee";
    ctx.beginPath();
    ctx.moveTo(10, 0);
    ctx.lineTo(-6, 6);
    ctx.lineTo(-6, -6);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

window.addEventListener("resize", drawCanvas);

/* ---------- playback ---------- */

function stopPlayback() {
  state.playing = false;
  el("playBtn").textContent = "▶ 재생";
  if (state.playTimer) { clearInterval(state.playTimer); state.playTimer = null; }
}

function startPlayback() {
  if (!state.track.length) return;
  if (state.cursor >= state.track.length - 1) state.cursor = 0;
  state.playing = true;
  state.liveFollow = false;
  el("playBtn").textContent = "⏸ 정지";
  state.playTimer = setInterval(() => {
    if (state.cursor >= state.track.length - 1) { stopPlayback(); return; }
    state.cursor += state.playSpeed;
    if (state.cursor > state.track.length - 1) state.cursor = state.track.length - 1;
    el("scrubber").value = state.cursor;
    renderStats(currentPoint());
    drawCanvas();
  }, 100);
}

el("playBtn").addEventListener("click", () => (state.playing ? stopPlayback() : startPlayback()));

el("speedSelect").addEventListener("change", (e) => {
  state.playSpeed = Number(e.target.value);
});

el("scrubber").addEventListener("input", (e) => {
  state.liveFollow = false;
  stopPlayback();
  state.cursor = Number(e.target.value);
  renderStats(currentPoint());
  drawCanvas();
});

el("jumpLiveBtn").addEventListener("click", () => {
  state.liveFollow = true;
  stopPlayback();
  state.cursor = state.track.length - 1;
  el("scrubber").value = state.cursor;
  renderStats(currentPoint());
  drawCanvas();
});

/* ---------- top controls ---------- */

el("scenarioSelect").addEventListener("change", onScenarioChange);
el("runSelect").addEventListener("change", onRunChange);

el("liveToggle").addEventListener("click", () => {
  state.livePolling = !state.livePolling;
  el("liveToggle").textContent = `실시간 갱신: ${state.livePolling ? "ON" : "OFF"}`;
  el("liveToggle").classList.toggle("off", !state.livePolling);
});

/* ---------- polling loop ---------- */

async function pollLoop() {
  if (state.livePolling && state.runId) {
    await refreshRun(false);
  }
  setTimeout(pollLoop, 1500);
}

loadScenarios().then(pollLoop);
