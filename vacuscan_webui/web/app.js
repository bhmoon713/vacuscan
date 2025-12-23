(() => {
  // ------------------- Optional wire-length geometry -------------------
  const corner = 0.34;
  const a = 0.152;
  const C = [
    [ +corner, +corner ],
    [ -corner, +corner ],
    [ -corner, -corner ],
    [ +corner, -corner ]
  ];
  const A = [
    [ +a, +a ],
    [ -a, +a ],
    [ -a, -a ],
    [ +a, -a ]
  ];

  const MAX_LOG_LINES = 10;
  let mainLogLines = [];
  let actionLogLines = [];

  // ------------------- UI references -------------------
  const wsUrl = document.getElementById("wsUrl");
  const btnConnect = document.getElementById("btnConnect");
  const btnDisconnect = document.getElementById("btnDisconnect");
  const connState = document.getElementById("connState");

  const gotoX = document.getElementById("gotoX");
  const gotoY = document.getElementById("gotoY");
  const btnGoto = document.getElementById("btnGoto");

  const jogStep = document.getElementById("jogStep");
  const limMin = document.getElementById("limMin");
  const limMax = document.getElementById("limMax");
  const jogXm = document.getElementById("jogXm");
  const jogXp = document.getElementById("jogXp");
  const jogYm = document.getElementById("jogYm");
  const jogYp = document.getElementById("jogYp");
  const btnHome = document.getElementById("btnHome");

  const kpiX   = document.getElementById("kpiX");
  const kpiY   = document.getElementById("kpiY");
  const kpiXum = document.getElementById("kpiXum");
  const kpiYum = document.getElementById("kpiYum");

  const logEl = document.getElementById("log");

  // Waypoint controls
  const wpRoute   = document.getElementById("wpRoute");
  const wpTol     = document.getElementById("wpTol");
  const wpTimeout = document.getElementById("wpTimeout");
  const wpPause   = document.getElementById("wpPause");
  const wpLoop    = document.getElementById("wpLoop");
  const btnRunRoute  = document.getElementById("btnRunRoute");
  const btnStopRoute = document.getElementById("btnStopRoute");

  // Progress UI
  const progressBar = document.getElementById("route-progress-bar");
  let totalWaypoints = 0;
  let currentWaypointIndex = 0;

  function updateProgressBar(pct) {
    progressBar.style.width = pct + "%";
    progressBar.textContent = Math.round(pct) + "%";
  }

  function resetProgress() {
    totalWaypoints = 0;
    currentWaypointIndex = 0;
    updateProgressBar(0);
  }


// ================= Current Waypoint (row/col/idx) ==================
// These elements exist only in your "Current Waypoint" card. If the card is not present, this block is a no-op.
const wpRowEl = document.getElementById("wpRow") || document.getElementById("wpRowVal");
const wpColEl = document.getElementById("wpCol") || document.getElementById("wpColVal");
const wpIdxEl = document.getElementById("wpIdx") || document.getElementById("wpIdxVal");

function setWpInfo(row, col, idx) {
  if (wpRowEl) wpRowEl.textContent = (row ?? "—");
  if (wpColEl) wpColEl.textContent = (col ?? "—");
  if (wpIdxEl) wpIdxEl.textContent = (idx ?? "—");
}

// Holds the selected route's points from waypoints.yaml (if available)
let wpPoints = null;

// A tiny parser for YOUR waypoints.yaml format (no external libs).
// Supports:
// routes:
//   <route_name>:
//     points:
//     - x_mm: ...
//       y_mm: ...
//       row: ...
//       col: ...
//       idx: ...
function parseWaypointsYaml(text, routeName) {
  const lines = text.split(/\r?\n/);
  let inRoutes = false;
  let inRoute = false;
  let inPoints = false;

  const points = [];
  let cur = null;

  function commit() {
    if (cur) points.push(cur);
    cur = null;
  }

  for (let raw of lines) {
    const line = raw.replace(/\t/g, "    ");
    const trimmed = line.trim();

    if (!trimmed || trimmed.startsWith("#")) continue;

    if (trimmed === "routes:") {
      inRoutes = true;
      inRoute = false;
      inPoints = false;
      continue;
    }

    // Route header like: "  scan_from_grid:"
    const routeMatch = inRoutes ? line.match(/^\s{2}([A-Za-z0-9_\-]+):\s*$/) : null;
    if (routeMatch) {
      const name = routeMatch[1];
      inRoute = (name === routeName);
      inPoints = false;
      commit();
      continue;
    }

    if (!inRoute) continue;

    if (trimmed === "points:" || trimmed.startsWith("points:")) {
      inPoints = true;
      continue;
    }

    if (!inPoints) continue;

    // New point starts with "- "
    const pointStart = line.match(/^\s*-\s*(.*)$/);
    if (pointStart) {
      commit();
      cur = {};
      const rest = pointStart[1].trim();
      if (rest) {
        // handle "- x_mm: 1.0" style
        const kv = rest.match(/^([A-Za-z0-9_]+):\s*(.*)$/);
        if (kv) cur[kv[1]] = kv[2];
      }
      continue;
    }

    // key: value lines under a point
    if (cur) {
      const kv = trimmed.match(/^([A-Za-z0-9_]+):\s*(.*)$/);
      if (kv) cur[kv[1]] = kv[2];
    }
  }
  commit();

  // normalize types
  for (const p of points) {
    for (const k of ["x_mm","y_mm","row","col","idx"]) {
      if (p[k] === undefined) continue;
      const n = Number(p[k]);
      if (!Number.isNaN(n)) p[k] = n;
    }
  }
  return points;
}

async function loadWaypointsForSelectedRoute() {
  wpPoints = null;
  setWpInfo("—", "—", "—");

  // If the Current Waypoint card isn't present, don't do anything.
  if (!wpRowEl && !wpColEl && !wpIdxEl) return;

  const route = wpRoute ? wpRoute.value : null;
  if (!route) return;

  try {
    const resp = await fetch("./waypoints.yaml", { cache: "no-store" });
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const text = await resp.text();
    const pts = parseWaypointsYaml(text, route);
    wpPoints = pts;

    // Immediately update with current index if we already have progress
    updateCurrentWaypointInfo(currentWaypointIndex);
    log(`[WAYPOINTS] Loaded ${pts.length} points for route "${route}" from waypoints.yaml`);
  } catch (e) {
    // Most common cause: waypoints.yaml is not being served next to index.html
    log(`[WAYPOINTS] Could not load ./waypoints.yaml (${e}). Put waypoints.yaml next to index.html or update fetch path.`);
  }
}

function updateCurrentWaypointInfo(progressIndex) {
  // If we haven't loaded yaml yet, show the ROS progress index at least.
  if (!wpPoints || progressIndex == null) {
    setWpInfo("—", "—", (progressIndex != null ? progressIndex : "—"));
    return;
  }

  const i = Math.max(0, Math.min(wpPoints.length - 1, progressIndex));
  const p = wpPoints[i] || {};
  setWpInfo(
    (p.row !== undefined ? p.row : "—"),
    (p.col !== undefined ? p.col : "—"),
    (p.idx !== undefined ? p.idx : i)
  );
}




// ================= Timers (Run / Current Waypoint) ==================
const runTimerEl = document.getElementById("runTimer");
const wpTimerEl2 = document.getElementById("wpTimer");

let runActive = false;
let runStartMs = 0;
let wpStartMs = 0;
let timerHandle = null;
let lastWpIndexForTimer = null;

function fmtMMSS(ms) {
  const s = Math.max(0, Math.floor(ms / 1000));
  const mm = String(Math.floor(s / 60)).padStart(2, "0");
  const ss = String(s % 60).padStart(2, "0");
  return `${mm}:${ss}`;
}

function renderTimers() {
  if (!runTimerEl && !wpTimerEl2) return;
  const now = Date.now();
  if (runTimerEl) runTimerEl.textContent = fmtMMSS(runActive ? (now - runStartMs) : 0);
  if (wpTimerEl2) wpTimerEl2.textContent = fmtMMSS(runActive ? (now - wpStartMs) : 0);
}

function startTimers() {
  runActive = true;
  runStartMs = Date.now();
  wpStartMs = runStartMs;
  lastWpIndexForTimer = null;
  renderTimers();
  if (!timerHandle) timerHandle = setInterval(renderTimers, 250);
}

function resetTimers() {
  runActive = false;
  runStartMs = 0;
  wpStartMs = 0;
  lastWpIndexForTimer = null;
  if (timerHandle) { clearInterval(timerHandle); timerHandle = null; }
  renderTimers(); // shows 00:00
}

function onWaypointIndexChangedForTimer(idx) {
  if (!runActive) return;
  if (idx == null) return;
  if (lastWpIndexForTimer === null) {
    lastWpIndexForTimer = idx;
    wpStartMs = Date.now();
    return;
  }
  if (idx !== lastWpIndexForTimer) {
    lastWpIndexForTimer = idx;
    wpStartMs = Date.now();
  }
}


  // ------------- Wire-length chart canvas ------------------
  const bars = document.getElementById("bars");
  const ctx = bars ? bars.getContext("2d") : null;
  let L = [0,0,0,0];

  function redrawBars() {
    if (!bars || !ctx) return;
    const dpr = window.devicePixelRatio || 1;
    const W = bars.clientWidth * dpr;
    const H = bars.clientHeight * dpr;
    if (bars.width !== W || bars.height !== H) {
      bars.width = W;
      bars.height = H;
    }
    ctx.clearRect(0,0,W,H);

    const pad = 20*dpr;
    const w = W-pad*2;
    const h = H-pad*2;
    const maxMM = 500;

    ctx.strokeStyle = "#30363d";
    ctx.lineWidth = 1*dpr;
    ctx.beginPath();
    ctx.moveTo(pad, H-pad);
    ctx.lineTo(W-pad, H-pad);
    ctx.stroke();

    const labels = ["L1","L2","L3","L4"];
    const gap = 14*dpr;
    const barW = (w - gap*3)/4;
    const scale = h / maxMM;

    for (let i=0;i<4;i++){
      const val = Math.min(maxMM, Math.max(0,L[i]));
      const bh = val * scale;
      const x = pad + i*(barW+gap);
      const y = H-pad - bh;

      ctx.fillStyle = "#58a6ff";
      ctx.fillRect(x,y,barW,bh);

      ctx.fillStyle = "#9da7b3";
      ctx.textAlign="center";
      ctx.font = `${12*dpr}px ui-sans-serif`;
      ctx.fillText(labels[i], x+barW/2, H-pad+14*dpr);

      ctx.fillStyle = "#e6edf3";
      ctx.fillText(val.toFixed(1), x+barW/2, y-6*dpr);
    }
  }

  function computeLengthsFromPose() {
    if (!bars || !ctx) return;
    const wx = pose.x; 
    const wy = pose.y;
    const out = [];
    for (let i=0;i<4;i++){
      const start = C[i];
      const end   = [ wx + A[i][0], wy + A[i][1] ];
      const dx = end[0]-start[0], dy=end[1]-start[1];
      out.push(Math.hypot(dx,dy)*1000.0);
    }
    L = out;
    redrawBars();
  }

  // ------------------ Logging --------------------
  function log(msg) {
    const t = new Date().toLocaleTimeString();
    const line = `[${t}] ${msg}`;
    mainLogLines.push(line);
    if (mainLogLines.length > MAX_LOG_LINES)
      mainLogLines = mainLogLines.slice(-MAX_LOG_LINES);
    logEl.textContent = mainLogLines.join("\n");
  }

  function addActionLog(text) {
    const ts = new Date().toLocaleTimeString();
    const line = `[${ts}] ${text}`;
    actionLogLines.push(line);
    if (actionLogLines.length > MAX_LOG_LINES)
      actionLogLines = actionLogLines.slice(-MAX_LOG_LINES);
    document.getElementById("action_log").textContent = actionLogLines.join("\n");
  }

  // ------------------ ROS Connection --------------------
  let ros = null;
  let connected = false;
  let pose = {x:0,y:0};

  let poseTopic, gotoSrv, runWpSrv, stopWpSrv;

  function setConnected(state) {
    connected = state;
    connState.textContent = state ? "connected" : "disconnected";
    btnDisconnect.disabled = !state;
    btnGoto.disabled = !state;
    [jogXm,jogXp,jogYm,jogYp,btnHome].forEach(b=> b.disabled=!state);
    btnRunRoute.disabled = !state;
    btnStopRoute.disabled = !state;
  }

  function connect() {
    if (ros) { try{ ros.close(); }catch(e){} }

    ros = new ROSLIB.Ros({ url: wsUrl.value.trim() });
    connState.textContent = "connecting...";
    btnConnect.disabled = true;

    ros.on("connection", () => {
      setConnected(true);
      btnConnect.disabled = false;
      log("Connected.");

      // ----- Pose topic -----
      poseTopic = new ROSLIB.Topic({
        ros,
        name: "/wafer/pose",
        messageType: "geometry_msgs/msg/Pose2D",
      });
      poseTopic.subscribe(msg => {
        pose.x = msg.x;
        pose.y = msg.y;
        computeLengthsFromPose();
        kpiX.textContent   = (msg.x*1000).toFixed(3);
        kpiY.textContent   = (msg.y*1000).toFixed(3);
        kpiXum.textContent = Math.round(msg.x*1e6);
        kpiYum.textContent = Math.round(msg.y*1e6);
      });

      // ----- Action status topic -----
      const actionStatusTopic = new ROSLIB.Topic({
        ros,
        name: "/wafer/action_status",
        messageType: "std_msgs/msg/String"
      });
      actionStatusTopic.subscribe(msg => {
        addActionLog(msg.data);
      });

      // ----- Progress topics -----
      const progTotal = new ROSLIB.Topic({
        ros,
        name: "/wafer/route_total",
        messageType: "std_msgs/msg/Int32"
      });
      progTotal.subscribe(m => { totalWaypoints = m.data; });

      const progIndex = new ROSLIB.Topic({
        ros,
        name: "/wafer/route_progress",
        messageType: "std_msgs/msg/Int32"
      });
      progIndex.subscribe(m => {
          currentWaypointIndex = m.data;
          onWaypointIndexChangedForTimer(currentWaypointIndex);
          if (totalWaypoints > 0) {
            updateProgressBar( ((currentWaypointIndex+1)/totalWaypoints)*100 );
          }
          updateCurrentWaypointInfo(currentWaypointIndex);
        });

      loadWaypointsForSelectedRoute();

      // ----- Services -----
      gotoSrv = new ROSLIB.Service({
        ros,
        name: "/wafer/goto",
        serviceType: "wafer_stage_interfaces/srv/WaferGoto"
      });

      runWpSrv = new ROSLIB.Service({
        ros,
        name: "/wafer/run_waypoints",
        serviceType: "wafer_stage_interfaces/srv/RunWaypoints"
      });

      stopWpSrv = new ROSLIB.Service({
        ros,
        name: "/wafer/abort_waypoints",
        serviceType: "std_srvs/srv/Trigger"
      });
    });

    ros.on("error", e => {
      setConnected(false);
      btnConnect.disabled = false;
      log("Connection error: " + e);
      resetTimers();
    });

    ros.on("close", () => {
      setConnected(false);
      btnConnect.disabled = false;
      log("Disconnected.");
      resetTimers();
    });
  }

  function disconnect() {
    if (ros) ros.close();
  }

  // ================= Movement ==================
  function callGoto(x_mm, y_mm) {
    if (!gotoSrv) return log("Goto service not ready.");
    const req = new ROSLIB.ServiceRequest({
      x: x_mm/1000.0,
      y: y_mm/1000.0,
    });
    btnGoto.disabled = true;
    gotoSrv.callService(req, res => {
      btnGoto.disabled = false;
      log(`[GOTO] success=${res.success} msg="${res.message}"`);
    }, err => {
      btnGoto.disabled = false;
      log("[GOTO] error: " + err);
    });
  }

  function jog(dx, dy) {
    const x_mm = pose.x*1000 + dx;
    const y_mm = pose.y*1000 + dy;
    callGoto(x_mm, y_mm);
    gotoX.value = x_mm.toFixed(3);
    gotoY.value = y_mm.toFixed(3);
  }

  // ================= Waypoints ==================
  function callRunRoute() {
    if (!runWpSrv) return log("RunWaypoints not ready.");

    resetProgress();

    // Ensure we have the latest route points loaded
    loadWaypointsForSelectedRoute();

    const req = new ROSLIB.ServiceRequest({
      route: wpRoute.value,
      tol_mm: parseFloat(wpTol.value),
      timeout_s: parseFloat(wpTimeout.value),
      pause_s: parseFloat(wpPause.value),
      loop: (wpLoop.value === "true")
    });

    btnRunRoute.disabled = true;
    runWpSrv.callService(
      req,
      res => {
        btnRunRoute.disabled = false;
        log(`[RUN] accepted=${res.accepted} msg="${res.message}"`);
      },
      err => {
        btnRunRoute.disabled = false;
        log("[RUN] error: " + err);
      }
    );
  }

  function callStopRoute() {
    if (!stopWpSrv) return log("Abort service not ready.");
    stopWpSrv.callService({}, res => {
      log(`[STOP] success=${res.success} msg="${res.message}"`);
    });
  }

  // ---------------- UI Events --------------------
  btnConnect.addEventListener("click", connect);
  btnDisconnect.addEventListener("click", disconnect);
  btnGoto.addEventListener("click",
    () => callGoto(parseFloat(gotoX.value), parseFloat(gotoY.value))
  );
  btnHome.addEventListener("click", () => callGoto(0,0));

  jogXm.addEventListener("click", () => jog(-parseFloat(jogStep.value),0));
  jogXp.addEventListener("click", () => jog(+parseFloat(jogStep.value),0));
  jogYm.addEventListener("click", () => jog(0,-parseFloat(jogStep.value)));
  jogYp.addEventListener("click", () => jog(0,+parseFloat(jogStep.value)));

  if (wpRoute) wpRoute.addEventListener("change", () => loadWaypointsForSelectedRoute());
  btnRunRoute.addEventListener("click", () => { startTimers(); callRunRoute(); });
  btnStopRoute.addEventListener("click", () => { resetTimers(); callStopRoute(); });

  redrawBars();
  log("Vacuscan UI ready.");
})();