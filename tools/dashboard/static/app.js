"use strict";

const COMPETITION_LIMIT_SECONDS = 15 * 60;

const OFFICIAL_ITEM_LABELS = [
  "제한속도 준수",
  "보호구역 속도 준수",
  "중앙선 침범 금지",
  "보도 침범 금지",
  "리스폰 (구간 첫 회 허용)",
  "차로변경 방향지시등 사용",
  "실선 차로변경 금지",
  "적색신호 정지",
  "녹색신호 통과",
  "적색점멸신호 일시정지",
  "횡단보도 정차 금지",
  "정적 장애물 충돌",
  "보행자 대응",
  "도로이용자 충돌",
];

const BACKEND_ITEM_ORDER = [1, 2, 4, 5, 15, 13, 6, 7, 8, 9, 12, 11, 10, 14];
const DISPLAY_ITEM_NUMBERS = [1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];

const state = {
  scenarioId: null,
  runId: null,
  sections: [],
  officialGrid: [],
  summary: {},
  track: [],
  mapData: { checkpoints: [], route: [] },
  cursor: 0,
  liveFollow: true,
  livePolling: true,
  playing: false,
  playSpeed: 1,
  runIsLive: false,
  playTimer: null,
  viewTab: "mainrs",
  mainrsRetryTimer: null,
  source: "live",
  liveStatus: "WAITING",
  livePayload: null,
};

const el = (id) => document.getElementById(id);

async function getJSON(url) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${url} -> ${response.status}`);
  return response.json();
}

function fmt(value, digits = 1) {
  if (value === null || value === undefined || Number.isNaN(value)) return "-";
  return Number(value).toFixed(digits);
}

function sectionLabel(section, index) {
  if (section === "unsegmented") return "전체";
  const numeric = String(section).match(/(?:^|[^0-9])(\d+)$/);
  return numeric ? `${Number(numeric[1])}구간` : (section || `${index + 1}구간`);
}

/* ---------- scenario / run selection ---------- */

let runsCache = { scenarioId: null, runs: [], loadedAt: 0 };

async function loadScenarios() {
  const scenarios = await getJSON("/api/scenarios");
  const select = el("scenarioSelect");
  select.innerHTML = "";

  for (const scenario of scenarios) {
    const option = document.createElement("option");
    option.value = scenario.scenario_id;
    option.textContent = `${scenario.scenario_id} · ${scenario.category || "scenario"}`;
    select.appendChild(option);
  }

  if (!scenarios.length) {
    const option = document.createElement("option");
    option.textContent = "등록된 시나리오 없음";
    select.appendChild(option);
    resetRunView();
    return;
  }

  select.value = scenarios.find((scenario) => scenario.run_count > 0)?.scenario_id || scenarios[0].scenario_id;
  await onScenarioChange();
}

async function loadRuns(scenarioId) {
  const runs = await getJSON(`/api/runs?scenario_id=${encodeURIComponent(scenarioId)}`);
  runsCache = { scenarioId, runs, loadedAt: Date.now() };

  const select = el("runSelect");
  select.innerHTML = "";

  for (const run of runs) {
    const option = document.createElement("option");
    option.value = run.run_id;
    const when = new Date(run.mtime * 1000).toLocaleTimeString("ko-KR", {
      hour: "2-digit",
      minute: "2-digit",
      second: "2-digit",
    });
    option.textContent = `${run.run_id}${run.live ? " · LIVE" : ""} · ${when}`;
    select.appendChild(option);
  }

  select.disabled = runs.length === 0;
  if (!runs.length) {
    const option = document.createElement("option");
    option.textContent = "실행 기록 없음";
    select.appendChild(option);
  }
  return runs;
}

async function loadRunsQuiet(scenarioId) {
  try {
    return await getJSON(`/api/runs?scenario_id=${encodeURIComponent(scenarioId)}`);
  } catch {
    return [];
  }
}

async function getRunsCache(force = false) {
  const stale = Date.now() - runsCache.loadedAt > 3000;
  if (force || stale || runsCache.scenarioId !== state.scenarioId) {
    runsCache = {
      scenarioId: state.scenarioId,
      runs: await loadRunsQuiet(state.scenarioId),
      loadedAt: Date.now(),
    };
  }
  return runsCache.runs;
}

async function onScenarioChange() {
  if (state.source === "live") return;
  state.scenarioId = el("scenarioSelect").value;
  const [runs, mapData] = await Promise.all([
    loadRuns(state.scenarioId),
    getJSON(`/api/map?scenario_id=${encodeURIComponent(state.scenarioId)}`),
  ]);

  state.mapData = mapData;
  const title = mapData.meta?.name || state.scenarioId;
  const map = mapData.meta?.map ? ` · ${mapData.meta.map}` : "";
  el("scenarioMeta").textContent = `${title}${map}`;

  if (runs.length) {
    el("runSelect").value = runs[0].run_id;
    await onRunChange();
  } else {
    resetRunView();
  }
}

async function onRunChange() {
  state.source = "history";
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
  state.summary = {};
  state.runIsLive = false;
  updateLiveBadge();
  renderChecklist();
  renderStats(null);
  renderScoreBar();
  drawCanvas();
}

/* ---------- run data polling ---------- */

async function refreshRun(fullReload = false) {
  if (!state.scenarioId || !state.runId) return;

  const since = fullReload || !state.track.length ? -1 : state.track[state.track.length - 1].frame;
  let payload;

  try {
    payload = await getJSON(
      `/api/run?scenario_id=${encodeURIComponent(state.scenarioId)}&run_id=${encodeURIComponent(state.runId)}&since=${since}`
    );
  } catch (error) {
    console.error(error);
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
  const runInfo = runs.find((run) => run.run_id === state.runId);
  state.runIsLive = Boolean(runInfo?.live);
  updateLiveBadge();

  if (state.liveFollow) state.cursor = Math.max(0, state.track.length - 1);
  state.cursor = Math.min(Math.max(state.cursor, 0), Math.max(0, state.track.length - 1));

  el("scrubber").max = Math.max(0, state.track.length - 1);
  el("scrubber").value = state.cursor;

  renderChecklist();
  renderStats(currentPoint());
  renderScoreBar();
  drawCanvas();
}

function updateLiveBadge() {
  const badge = el("liveBadge");

  if (!state.runId) {
    badge.innerHTML = "<i></i> READY";
    badge.className = "run-state badge-idle";
  } else if (state.runIsLive) {
    badge.innerHTML = "<i></i> RUNNING";
    badge.className = "run-state badge-live";
  } else {
    badge.innerHTML = "<i></i> COMPLETED";
    badge.className = "run-state badge-done";
  }
}

function currentPoint() {
  if (!state.track.length) return null;
  return state.track[Math.min(Math.max(state.cursor, 0), state.track.length - 1)];
}

/* ---------- summary cards ---------- */

function cumulativeDistance(endIndex) {
  let distance = 0;
  const last = Math.min(endIndex, state.track.length - 1);

  for (let index = 1; index <= last; index += 1) {
    const previous = state.track[index - 1];
    const current = state.track[index];
    if (![previous.x, previous.y, current.x, current.y].every(Number.isFinite)) continue;

    const step = Math.hypot(current.x - previous.x, current.y - previous.y);
    if (step <= 50) distance += step;
  }
  return distance;
}

function updateSignal(signal) {
  const normalized = String(signal || "").toUpperCase();
  const red = normalized.includes("RED");
  const yellow = normalized.includes("YELLOW") || normalized.includes("AMBER");
  const green = normalized.includes("GREEN");
  const other = Boolean(normalized) && !red && !yellow && !green;

  el("signalRed").classList.toggle("active", red);
  el("signalYellow").classList.toggle("active", yellow);
  el("signalGreen").classList.toggle("active", green);
  el("signalOther").classList.toggle("active", other);

  const label = signal || "신호 없음";
  el("statSignal").textContent = label;
  el("overlaySignal").textContent = label;
}

function renderStats(point) {
  const finalFrame = state.track.length ? state.track[state.track.length - 1].frame : 0;

  if (!point) {
    el("statSection").textContent = "-";
    el("statElapsed").innerHTML = "0<small>s</small>";
    el("statSpeed").innerHTML = "0.0<small>km/h</small>";
    el("statDistance").innerHTML = "0.0<small>m</small>";
    el("statLimit").innerHTML = `${COMPETITION_LIMIT_SECONDS}<small>s</small>`;
    el("stationFrame").textContent = "#0";
    el("frameReadout").textContent = "0 frames";
    el("overlayFrame").textContent = "0 frames";
    el("overlayTime").textContent = "00:00.0";
    updateSignal("");
    return;
  }

  const elapsed = Math.max(0, point.t || 0);
  const remaining = Math.max(0, COMPETITION_LIMIT_SECONDS - elapsed);
  const currentSection = sectionLabel(point.section, 0);

  el("statSection").textContent = currentSection.replace("구간", "");
  el("statElapsed").innerHTML = `${Math.round(elapsed)}<small>s</small>`;
  el("statSpeed").innerHTML = `${fmt(Number.isFinite(point.speed) ? point.speed * 3.6 : null, 1)}<small>km/h</small>`;
  el("statDistance").innerHTML = `${fmt(cumulativeDistance(state.cursor), 1)}<small>m</small>`;
  el("statLimit").innerHTML = `${Math.ceil(remaining)}<small>s</small>`;
  el("stationFrame").textContent = `#${point.frame}`;
  el("frameReadout").textContent = `${point.frame} / ${finalFrame} frames`;
  el("overlayFrame").textContent = `${point.frame} frames`;
  el("overlayTime").textContent = formatClock(elapsed);
  updateSignal(point.signal);
}

function formatClock(seconds) {
  if (seconds === null || seconds === undefined) return "00:00.0";
  const minutes = Math.floor(seconds / 60);
  const remainder = seconds - minutes * 60;
  return `${String(minutes).padStart(2, "0")}:${remainder.toFixed(1).padStart(4, "0")}`;
}

/* ---------- official 14-item table (original number 3 is excluded) ---------- */

function visibleSections() {
  return state.sections.length ? state.sections : ["unsegmented"];
}

function deductionFor(item, section) {
  if (!item) return null;
  if (item.status === "UNAVAILABLE" || item.status === "NOT_APPLICABLE") return null;
  if (state.source === "live" && section !== "unsegmented" && !state.livePayload?.reached_sections?.includes(section)) return null;
  if (item.observed_sections && !item.observed_sections.includes(section)) return null;
  if (item.penalty_by_section) return Number(item.penalty_by_section[section] || 0);
  const severity = Number(item.by_section?.[section] || 0);
  return severity === 2 ? 6 : severity === 1 ? 3 : 0;
}

function sectionScore(section) {
  let score = 100;
  for (const item of state.officialGrid) {
    score -= deductionFor(item, section) || 0;
  }
  return score;
}

function renderChecklist() {
  const sections = visibleSections();
  const byNumber = new Map(state.officialGrid.map((item) => [Number(item.number), item]));
  const pending = state.officialGrid.length === 0;

  const head = el("checklistHead");
  head.innerHTML = '<th class="col-item">평가 항목</th>';
  sections.forEach((section, index) => {
    const th = document.createElement("th");
    th.textContent = sectionLabel(section, index);
    head.appendChild(th);
  });

  const body = el("checklistBody");
  body.innerHTML = "";

  OFFICIAL_ITEM_LABELS.forEach((label, index) => {
    const item = byNumber.get(BACKEND_ITEM_ORDER[index]);
    const row = document.createElement("tr");
    const name = document.createElement("td");
    name.className = "col-item";
    name.innerHTML = `<span class="item-number">${String(DISPLAY_ITEM_NUMBERS[index]).padStart(2, "0")}</span>${label}`;
    row.appendChild(name);

    sections.forEach((section) => {
      const cell = document.createElement("td");
      const deduction = pending ? null : deductionFor(item, section);
      cell.className = "penalty-cell";

      if (deduction === null) {
        cell.textContent = "–";
        cell.classList.add("pending");
      } else {
        cell.textContent = deduction === 0 && item.partial_sections?.includes(section) ? "확인" : String(deduction);
        if (deduction === 3) cell.classList.add("minor");
        if (deduction >= 6) cell.classList.add("major");
      }
      cell.title = item?.detail || "판정 데이터 또는 기준 대기";
      row.appendChild(cell);
    });
    body.appendChild(row);
  });

  const foot = el("checklistFoot");
  const total = state.summary?.official_score ?? "–";
  foot.innerHTML = "";
  const row = document.createElement("tr");
  const label = document.createElement("td");
  label.className = "col-item";
  label.innerHTML = `구간별 점수 <span class="total-score-inline">종합점수: ${total}</span>`;
  row.appendChild(label);

  sections.forEach((section) => {
    const cell = document.createElement("td");
    cell.textContent = pending || (state.source === "live" && !state.livePayload?.reached_sections?.includes(section)) ? "–" : String(sectionScore(section));
    row.appendChild(cell);
  });
  foot.appendChild(row);
}

/* ---------- score ---------- */

function renderScoreBar() {
  const summary = state.summary || {};
  const score = summary.official_score ?? "–";
  const majorCount = summary.major_count ?? state.officialGrid.reduce(
    (count, item) => count + Object.values(item.by_section || {}).filter((severity) => Number(severity) === 2).length,
    0
  );

  el("totalScore").textContent = score;
  el("statMajor").textContent = String(majorCount);

  if (!state.runId) {
    el("scoreStatus").textContent = "평가 대기";
  } else if (summary.official_evaluation === "PARTIAL") {
    el("scoreStatus").textContent = "일부 항목 미평가";
  } else {
    el("scoreStatus").textContent = state.runIsLive ? "실시간 채점 중" : "평가 완료";
  }

  el("majorCaption").textContent = majorCount ? "확인 필요" : "중대 위반 없음";
}

el("reportBtn").addEventListener("click", () => {
  el("reportContent").textContent = JSON.stringify(
    state.source === "live" ? state.livePayload : { summary: state.summary, official_grid: state.officialGrid },
    null,
    2
  );
  el("reportModal").classList.remove("hidden");
});

el("closeReport").addEventListener("click", () => el("reportModal").classList.add("hidden"));
el("reportModal").addEventListener("click", (event) => {
  if (event.target === el("reportModal")) el("reportModal").classList.add("hidden");
});
document.addEventListener("keydown", (event) => {
  if (event.key === "Escape") el("reportModal").classList.add("hidden");
});

/* ---------- map canvas ---------- */

function computeBounds() {
  const xs = [];
  const ys = [];

  for (const point of state.mapData.route || []) {
    xs.push(point.x);
    ys.push(point.y);
  }
  for (const point of state.mapData.checkpoints || []) {
    xs.push(point.x);
    ys.push(point.y);
  }
  for (const point of state.track) {
    if (point.x !== null) {
      xs.push(point.x);
      ys.push(point.y);
    }
  }

  if (!xs.length) return null;

  const minX = Math.min(...xs);
  const maxX = Math.max(...xs);
  const minY = Math.min(...ys);
  const maxY = Math.max(...ys);
  const pad = Math.max(5, (maxX - minX) * 0.08, (maxY - minY) * 0.08);
  return { minX: minX - pad, maxX: maxX + pad, minY: minY - pad, maxY: maxY + pad };
}

function toScreen(canvas, bounds, x, y) {
  const rangeX = bounds.maxX - bounds.minX || 1;
  const rangeY = bounds.maxY - bounds.minY || 1;
  const scale = Math.min(canvas.width / rangeX, canvas.height / rangeY);
  const offsetX = (canvas.width - rangeX * scale) / 2;
  const offsetY = (canvas.height - rangeY * scale) / 2;

  return {
    sx: offsetX + (x - bounds.minX) * scale,
    sy: canvas.height - (offsetY + (y - bounds.minY) * scale),
  };
}

function drawCanvas() {
  const canvas = el("mapCanvas");
  const wrap = canvas.parentElement;
  canvas.width = wrap.clientWidth;
  canvas.height = wrap.clientHeight;

  const context = canvas.getContext("2d");
  context.clearRect(0, 0, canvas.width, canvas.height);
  const bounds = computeBounds();
  if (!bounds) return;

  const route = state.mapData.route || [];
  if (route.length) {
    context.strokeStyle = "#33424a";
    context.lineWidth = 3;
    context.beginPath();
    route.forEach((point, index) => {
      const screen = toScreen(canvas, bounds, point.x, point.y);
      if (index === 0) context.moveTo(screen.sx, screen.sy);
      else context.lineTo(screen.sx, screen.sy);
    });
    context.stroke();
  }

  context.fillStyle = "#64737a";
  context.font = "11px sans-serif";
  for (const checkpoint of state.mapData.checkpoints || []) {
    const screen = toScreen(canvas, bounds, checkpoint.x, checkpoint.y);
    context.beginPath();
    context.arc(screen.sx, screen.sy, 4, 0, Math.PI * 2);
    context.fill();
    context.fillText(String(checkpoint.seq), screen.sx + 6, screen.sy - 6);
  }

  const traveled = state.track.slice(0, state.cursor + 1).filter((point) => point.x !== null);
  if (traveled.length > 1) {
    context.strokeStyle = "#49d7e7";
    context.lineWidth = 2.5;
    context.beginPath();
    traveled.forEach((point, index) => {
      const screen = toScreen(canvas, bounds, point.x, point.y);
      if (index === 0) context.moveTo(screen.sx, screen.sy);
      else context.lineTo(screen.sx, screen.sy);
    });
    context.stroke();
  }

  const point = currentPoint();
  if (!point || point.x === null) return;
  const screen = toScreen(canvas, bounds, point.x, point.y);

  if (point.obj_x !== null && point.obj_y !== null) {
    const object = toScreen(canvas, bounds, point.obj_x, point.obj_y);
    context.strokeStyle = "rgba(255, 72, 101, 0.5)";
    context.lineWidth = 1;
    context.beginPath();
    context.moveTo(screen.sx, screen.sy);
    context.lineTo(object.sx, object.sy);
    context.stroke();
    context.fillStyle = "#ff4865";
    context.beginPath();
    context.arc(object.sx, object.sy, 5, 0, Math.PI * 2);
    context.fill();
  }

  context.save();
  context.translate(screen.sx, screen.sy);
  context.rotate(-(point.heading || 0));
  context.fillStyle = point.collision ? "#ff4865" : "#49d7e7";
  context.beginPath();
  context.moveTo(11, 0);
  context.lineTo(-7, 7);
  context.lineTo(-7, -7);
  context.closePath();
  context.fill();
  context.restore();
}

window.addEventListener("resize", drawCanvas);

/* ---------- mainRS live view ---------- */

function setStreamState(connected) {
  const badge = el("streamBadge");
  const wrap = document.querySelector(".viewer-canvas-wrap");
  badge.classList.toggle("waiting", !connected);
  badge.innerHTML = connected ? "<i></i> LIVE" : "<i></i> STANDBY";
  wrap.classList.toggle("stream-connected", connected);
}

function setViewTab(tab) {
  state.viewTab = tab;
  el("tabMainrs").classList.toggle("active", tab === "mainrs");
  el("tabMap").classList.toggle("active", tab === "map");
  el("mainrsImg").classList.add("hidden");
  el("mainrsFallback").classList.toggle(
    "hidden",
    tab !== "mainrs" || !el("mainrsFallback").dataset.show
  );
  el("mapCanvas").classList.toggle("hidden", tab !== "map");
  el("playbackControls").classList.toggle("hidden", tab !== "map");

  if (tab === "mainrs") {
    connectMainrsStream();
  } else {
    disconnectMainrsStream();
    setStreamState(false);
    drawCanvas();
  }
}

function connectMainrsStream() {
  if (state.mainrsRetryTimer) {
    clearTimeout(state.mainrsRetryTimer);
    state.mainrsRetryTimer = null;
  }

  const image = el("mainrsImg");
  el("mainrsFallback").dataset.show = "";
  el("mainrsFallback").classList.add("hidden");
  setStreamState(false);
  image.src = `/api/mainrs_stream?_=${Date.now()}`;
}

function disconnectMainrsStream() {
  if (state.mainrsRetryTimer) {
    clearTimeout(state.mainrsRetryTimer);
    state.mainrsRetryTimer = null;
  }
  el("mainrsImg").src = "";
}

el("mainrsImg").addEventListener("error", () => {
  if (state.viewTab !== "mainrs") return;

  el("mainrsImg").classList.add("hidden");
  el("mainrsFallback").dataset.show = "1";
  el("mainrsFallback").classList.remove("hidden");
  setStreamState(false);

  if (!state.mainrsRetryTimer) {
    state.mainrsRetryTimer = setTimeout(() => {
      state.mainrsRetryTimer = null;
      if (state.viewTab === "mainrs") connectMainrsStream();
    }, 3000);
  }
});

el("mainrsImg").addEventListener("load", () => {
  if (state.viewTab !== "mainrs") return;
  el("mainrsImg").classList.remove("hidden");
  el("mainrsFallback").dataset.show = "";
  el("mainrsFallback").classList.add("hidden");
  setStreamState(true);
});

el("tabMainrs").addEventListener("click", () => setViewTab("mainrs"));
el("tabMap").addEventListener("click", () => setViewTab("map"));

/* ---------- map playback ---------- */

function stopPlayback() {
  state.playing = false;
  el("playBtn").textContent = "▶ 재생";
  if (state.playTimer) {
    clearInterval(state.playTimer);
    state.playTimer = null;
  }
}

function startPlayback() {
  if (!state.track.length) return;
  if (state.cursor >= state.track.length - 1) state.cursor = 0;
  state.playing = true;
  state.liveFollow = false;
  el("playBtn").textContent = "Ⅱ 정지";

  state.playTimer = setInterval(() => {
    if (state.cursor >= state.track.length - 1) {
      stopPlayback();
      return;
    }
    state.cursor = Math.min(state.cursor + state.playSpeed, state.track.length - 1);
    el("scrubber").value = state.cursor;
    renderStats(currentPoint());
    drawCanvas();
  }, 100);
}

el("playBtn").addEventListener("click", () => (state.playing ? stopPlayback() : startPlayback()));
el("speedSelect").addEventListener("change", (event) => {
  state.playSpeed = Number(event.target.value);
});
el("scrubber").addEventListener("input", (event) => {
  state.liveFollow = false;
  stopPlayback();
  state.cursor = Number(event.target.value);
  renderStats(currentPoint());
  drawCanvas();
});
el("jumpLiveBtn").addEventListener("click", () => {
  state.liveFollow = true;
  stopPlayback();
  state.cursor = Math.max(0, state.track.length - 1);
  el("scrubber").value = state.cursor;
  renderStats(currentPoint());
  drawCanvas();
});

/* ---------- controls + polling ---------- */

el("scenarioSelect").addEventListener("change", () => { state.source = "history"; onScenarioChange(); });
el("liveSourceBtn").addEventListener("click", () => { state.source = "live"; refreshLive(); });
el("runSelect").addEventListener("change", onRunChange);
el("reloadRun").addEventListener("click", async () => {
  runsCache.loadedAt = 0;
  if (state.runId) await refreshRun(true);
  else await onScenarioChange();
});

el("liveToggle").addEventListener("click", () => {
  state.livePolling = !state.livePolling;
  const button = el("liveToggle");
  button.innerHTML = state.livePolling
    ? '<span aria-hidden="true">Ⅱ</span> 실시간 갱신'
    : '<span aria-hidden="true">▶</span> 갱신 재개';
  button.classList.toggle("off", !state.livePolling);
});

async function pollLoop() {
  try {
    const video = await getJSON("/api/mainrs_status").catch(() => null);
    if (state.viewTab === "mainrs" && video && !video.found) {
      setStreamState(false);
      el("mainrsImg").classList.add("hidden");
      el("mainrsFallback").classList.remove("hidden");
      el("mainrsFallback").title = video.error || "VTD 영상 대기";
      if (!state.mainrsRetryTimer) state.mainrsRetryTimer = setTimeout(connectMainrsStream, 3000);
    }
    if (state.livePolling && state.source === "live") await refreshLive();
    else if (state.livePolling && state.runId) await refreshRun(false);
    else if (state.livePolling && state.scenarioId) await onScenarioChange();
  } finally {
    setTimeout(pollLoop, 500);
  }
}

async function refreshLive() {
  let payload;
  try { payload = await getJSON("/api/live"); }
  catch (error) {
    el("liveBadge").textContent = "연결 끊김";
    el("scoreStatus").textContent = "상태 수신 중단";
    return;
  }
  if (!payload.enabled) {
    state.source = "history";
    await onScenarioChange();
    return;
  }
  if (state.source !== "live") return;
  if (state.runId !== payload.session_id) {
    state.track = [];
    state.cursor = 0;
  }
  state.runId = payload.session_id;
  state.livePayload = payload;
  state.liveStatus = payload.status;
  state.runIsLive = payload.status === "RUNNING" && payload.receiver.connected;
  state.summary = payload.summary;
  state.officialGrid = payload.official_grid;
  state.sections = payload.sections.length ? payload.sections : ["unsegmented"];
  if (payload.unsegmented_frames && !state.sections.includes("unsegmented")) state.sections = [...state.sections, "unsegmented"];
  const r = payload.latest;
  if (r && r.frame !== undefined) {
    const point = { frame:Number(r.frame), t:Number(r.elapsed_wall_time_s), x:Number(r.ego_x), y:Number(r.ego_y),
      heading:Number(r.ego_heading), speed:Number(r.ego_speed), signal:r.traffic_light_state || "",
      section:r.evaluation_section_id || "미설정", distance:Number(r.distance_m) };
    if (state.track.at(-1)?.frame !== point.frame) state.track.push(point);
    // Live map display is bounded; full evidence remains in the session JSONL.
    if (state.track.length > 10000) state.track.splice(0, state.track.length-10000);
    state.cursor = state.track.length-1;
    renderStats(point);
    el("statDistance").innerHTML = `${fmt(point.distance,1)}<small>m</small>`;
    el("statLimit").innerHTML = `${Math.ceil(Math.max(0,payload.time_limit_s-point.t))}<small>s</small>`;
  } else renderStats(null);
  renderChecklist();
  renderScoreBar();
  const names = {WAITING:"제어값 수신 대기", RUNNING:"채점 중", COMPLETED:"완주", TIMEOUT:"시간 종료", NO_DATA:"수신 대기", RESET:"초기화", ABORTED:"중단"};
  const badge = el("liveBadge");
  badge.textContent = payload.receiver.connected ? names[payload.status] || payload.status : "RDB 연결 대기";
  badge.className = `run-state ${state.runIsLive ? "badge-live" : "badge-idle"}`;
  el("scenarioMeta").textContent = `${r?.scenario_id || "VTD"} · ${payload.missing_metadata.length ? "평가 경계 설정 필요" : "실시간 상태 수신"}`;
  el("scenarioMeta").title = payload.receiver.error || payload.missing_metadata.join(", ");
  const checks = [
    payload.receiver.connected ? "VTD 상태 연결됨" : `VTD 상태 연결 대기: ${payload.receiver.error || "RDB 출력 확인"}`,
    payload.control_receiver?.connected ? "제어 수신 감지기 연결됨 · 첫 제어값부터 채점" : "제어 수신 감지기 미연결 · 서버 터미널에 출력된 감지기 실행 명령을 확인하세요",
    payload.completion?.mode === "final_checkpoint"
      ? `종료: 마지막 체크포인트 도달 · 허용거리 ${payload.completion.tolerance_m}m · 정지선 지정 불필요`
      : payload.finish_line ? `종료선 설정됨: ${payload.finish_line.id || "사용자 지정"} · 후륜축 정방향 통과` : "종료 정지선 미확정 (이전 종료선 방식)",
    ...(payload.warnings || []),
    "시나리오 선택 메뉴는 기록 조회용입니다. 실제 평가 경로는 서버의 설정 파일을 사용합니다.",
  ];
  el("healthSummary").textContent = `연결·평가 상태: ${payload.summary.official_evaluation === "COMPLETE" ? "입력 충족 (현장 검증 별도)" : "확인 필요"} · 횡단보도 ${payload.feature_counts?.crosswalks || 0}개`;
  el("healthDetails").replaceChildren(...checks.map(text => { const li = document.createElement("li"); li.textContent = text; return li; }));
  if (state.summary.official_score === null && state.runId) el("scoreStatus").textContent = "구간 미설정 · 감점만 집계";
  drawCanvas();
}

renderChecklist();
renderStats(null);
renderScoreBar();
setViewTab("mainrs");
loadScenarios()
  .catch((error) => {
    console.error(error);
    el("scenarioMeta").textContent = "대시보드 데이터를 불러오지 못했습니다";
  })
  .finally(pollLoop);
