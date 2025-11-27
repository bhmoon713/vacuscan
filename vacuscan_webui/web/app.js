(() => {
  // ---- UI refs ----
  const wsUrl = document.getElementById('wsUrl');
  const btnConnect = document.getElementById('btnConnect');
  const btnDisconnect = document.getElementById('btnDisconnect');
  const connState = document.getElementById('connState');
  const gotoX = document.getElementById('gotoX');
  const gotoY = document.getElementById('gotoY');
  const btnGoto = document.getElementById('btnGoto');
  const jogStep = document.getElementById('jogStep');
  const limMin = document.getElementById('limMin');
  const limMax = document.getElementById('limMax');
  const jogXm = document.getElementById('jogXm');
  const jogXp = document.getElementById('jogXp');
  const jogYm = document.getElementById('jogYm');
  const jogYp = document.getElementById('jogYp');
  const btnHome = document.getElementById('btnHome');
  const kpiX = document.getElementById('kpiX');
  const kpiY = document.getElementById('kpiY');
  const kpiXum = document.getElementById('kpiXum');
  const kpiYum = document.getElementById('kpiYum');
  const logEl = document.getElementById('log');

  // Waypoints UI
  const wpRoute = document.getElementById('wpRoute');
  const wpTol = document.getElementById('wpTol');
  const wpTimeout = document.getElementById('wpTimeout');
  const wpPause = document.getElementById('wpPause');
  const wpLoop = document.getElementById('wpLoop');
  const btnRunRoute = document.getElementById('btnRunRoute');
  const btnStopRoute = document.getElementById('btnStopRoute');

  // XY canvas
  const xyCanvas = document.getElementById('xyCanvas');
  const xyCtx = xyCanvas.getContext('2d');

  const MAX_LOG_LINES = 15;
  let mainLogLines = [];
  let actionLogLines = [];

  let ros = null;
  let connected = false;
  let pose = { x: 0, y: 0 };  // meters
  let poseTopic = null;
  let gotoSrv = null;
  let runWpSrv = null, stopWpSrv = null, routesTopic = null;

  // ---------------------------
  // Route Progress Handling
  // ---------------------------
  let totalWaypoints = 0;
  let currentWaypointIndex = 0;

  function resetProgress() {
    totalWaypoints = 0;
    currentWaypointIndex = 0;
    updateProgressBar(0);
  }

  function updateProgressBar(percent) {
    const bar = document.getElementById('route-progress-bar');
    if (!bar) return;
    bar.style.width = percent + "%";
    bar.textContent = Math.round(percent) + "%";
  }

  function um(x) { return Math.round(x * 1e6); }

  function log(msg) {
    const t = new Date().toLocaleTimeString();
    const line = `[${t}] ${msg}`;

    // newest first
    mainLogLines.unshift(line);
    if (mainLogLines.length > MAX_LOG_LINES) {
      mainLogLines = mainLogLines.slice(0, MAX_LOG_LINES);
    }

    logEl.innerText = mainLogLines.join('\n');
    logEl.scrollTop = 0;
  }

  function setConnected(state) {
    connected = state;
    connState.textContent = state ? 'connected' : 'disconnected';
    connState.classList.toggle('ok', state);
    connState.style.borderColor = state ? 'transparent' : '#30363d';
    connState.style.background = state ? 'var(--ok)' : 'var(--card)';
    btnDisconnect.disabled = !state;
    btnGoto.disabled = !state;
    [jogXm, jogXp, jogYm, jogYp, btnHome].forEach(b => b.disabled = !state);
    btnRunRoute.disabled = !state;
    btnStopRoute.disabled = !state;
  }

  function inLimits(x_mm, y_mm) {
    const min = parseFloat(limMin.value), max = parseFloat(limMax.value);
    return (x_mm >= min && x_mm <= max && y_mm >= min && y_mm <= max);
  }

  // ---------------------------
  // XY Stage View drawing
  // ---------------------------
  function resizeXYCanvas() {
    const dpr = window.devicePixelRatio || 1;
    const W = xyCanvas.clientWidth * dpr;
    const H = xyCanvas.clientHeight * dpr;
    if (xyCanvas.width !== W || xyCanvas.height !== H) {
      xyCanvas.width = W;
      xyCanvas.height = H;
    }
    drawXY();
  }

  function drawXY() {
    const ctx = xyCtx;
    const W = xyCanvas.width;
    const H = xyCanvas.height;
    if (W === 0 || H === 0) return;

    // clear
    ctx.clearRect(0, 0, W, H);

    const dpr = window.devicePixelRatio || 1;
    const pad = 20 * dpr;
    const min = parseFloat(limMin.value);
    const max = parseFloat(limMax.value);
    const range = max - min || 1.0;

    // background
    ctx.fillStyle = '#05070b';
    ctx.fillRect(0, 0, W, H);

    // axes at center
    const centerX = W / 2;
    const centerY = H / 2;
    ctx.strokeStyle = '#30363d';
    ctx.lineWidth = 1 * dpr;
    ctx.beginPath();
    // X axis
    ctx.moveTo(pad, centerY);
    ctx.lineTo(W - pad, centerY);
    // Y axis
    ctx.moveTo(centerX, pad);
    ctx.lineTo(centerX, H - pad);
    ctx.stroke();

    // soft-limit square (mapped from [min,max] x [min,max])
    const size = Math.min(W - 2 * pad, H - 2 * pad);
    const squareLeft = centerX - size / 2;
    const squareTop = centerY - size / 2;

    ctx.strokeStyle = '#58a6ff';
    ctx.lineWidth = 1.5 * dpr;
    ctx.strokeRect(squareLeft, squareTop, size, size);

    // label min/max
    ctx.fillStyle = '#9da7b3';
    ctx.font = `${10 * dpr}px ui-sans-serif`;
    ctx.textAlign = 'left';
    ctx.fillText(`${min} mm`, squareLeft + 4 * dpr, squareTop - 4 * dpr);
    ctx.textAlign = 'right';
    ctx.fillText(`${max} mm`, squareLeft + size - 4 * dpr, squareTop - 4 * dpr);

    // draw current pose
    const x_mm = pose.x * 1000.0;
    const y_mm = pose.y * 1000.0;

    // normalize to [0,1]
    const nx = (x_mm - min) / range;
    const ny = (y_mm - min) / range;

    const px = squareLeft + nx * size;
    const py = squareTop + (1 - ny) * size; // invert y so +Y is up

    // crosshair
    ctx.strokeStyle = '#3fb950';
    ctx.lineWidth = 1 * dpr;
    ctx.beginPath();
    ctx.moveTo(px, squareTop);
    ctx.lineTo(px, squareTop + size);
    ctx.moveTo(squareLeft, py);
    ctx.lineTo(squareLeft + size, py);
    ctx.stroke();

    // dot
    ctx.fillStyle = '#3fb950';
    ctx.beginPath();
    ctx.arc(px, py, 4 * dpr, 0, 2 * Math.PI);
    ctx.fill();

    // text near dot
    ctx.fillStyle = '#e6edf3';
    ctx.textAlign = 'left';
    ctx.fillText(
      `(${x_mm.toFixed(2)}, ${y_mm.toFixed(2)}) mm`,
      px + 6 * dpr,
      py - 6 * dpr
    );
  }

  function updatePoseKpis() {
    kpiX.textContent = (pose.x * 1000).toFixed(3);
    kpiY.textContent = (pose.y * 1000).toFixed(3);
    kpiXum.textContent = um(pose.x);
    kpiYum.textContent = um(pose.y);
    drawXY();
  }

  // ---------------------------
  // ROS connection
  // ---------------------------
  function connect() {
    if (ros) try { ros.close(); } catch (e) { }
    ros = new ROSLIB.Ros({ url: wsUrl.value.trim() });
    connState.textContent = 'connecting...';
    connState.style.background = '#8b949e';
    btnConnect.disabled = true;

    ros.on('connection', () => {
      setConnected(true);
      btnConnect.disabled = false;
      log(`Connected to ${wsUrl.value}`);

      // Pose (pure XY joint control)
      poseTopic = new ROSLIB.Topic({
        ros,
        name: '/vacuscan/pose',
        messageType: 'geometry_msgs/msg/Pose2D',
        throttle_rate: 50
      });
      poseTopic.subscribe(msg => {
        pose.x = msg.x;
        pose.y = msg.y;
        updatePoseKpis();
      });

      // Action status
      const actionStatusListener = new ROSLIB.Topic({
        ros: ros,
        name: '/vacuscan/action_status',
        messageType: 'std_msgs/msg/String'
      });
      actionStatusListener.subscribe((msg) => {
        addActionLog(msg.data);
      });

      // Goto service (reuse WaferGoto srv definition)
      gotoSrv = new ROSLIB.Service({
        ros,
        name: '/vacuscan/goto',
        serviceType: 'wafer_stage_interfaces/srv/WaferGoto'
      });

      // Progress topics
      const progressTotalSub = new ROSLIB.Topic({
        ros: ros,
        name: "/vacuscan/route_total",
        messageType: "std_msgs/msg/Int32"
      });
      progressTotalSub.subscribe(msg => {
        totalWaypoints = msg.data;
        if (totalWaypoints === 0) {
          updateProgressBar(0);
        }
      });

      const progressSub = new ROSLIB.Topic({
        ros: ros,
        name: "/vacuscan/route_progress",
        messageType: "std_msgs/msg/Int32"
      });
      progressSub.subscribe(msg => {
        currentWaypointIndex = msg.data;
        if (totalWaypoints > 0) {
          const percent = ((currentWaypointIndex + 1) / totalWaypoints) * 100;
          updateProgressBar(percent);
        }
      });

      // Waypoints services
      runWpSrv = new ROSLIB.Service({
        ros,
        name: '/vacuscan/run_waypoints',
        serviceType: 'wafer_stage_interfaces/srv/RunWaypoints'
      });
      stopWpSrv = new ROSLIB.Service({
        ros,
        name: '/vacuscan/abort_waypoints',
        serviceType: 'std_srvs/srv/Trigger'
      });

      // (Optional) waypoint routes from YAML or JSON string
      routesTopic = new ROSLIB.Topic({
        ros,
        name: '/vacuscan/waypoint_routes',
        messageType: 'std_msgs/msg/String',
        throttle_rate: 1000
      });
      routesTopic.subscribe(msg => {
        try {
          const dataStr = msg.data;
          let parsed;
          if (window.jsyaml) {
            parsed = window.jsyaml.load(dataStr);
          } else {
            parsed = JSON.parse(dataStr);
          }
          const keys = Object.keys(parsed || {});
          if (keys.length) {
            wpRoute.innerHTML =
              keys.map(k => `<option value="${k}">${k}</option>`).join('');
            log(`Routes updated: ${keys.join(', ')}`);
          }
        } catch (err) {
          log(`Failed to parse waypoint_routes: ${err}`);
        }
      });
    });

    ros.on('error', (e) => {
      setConnected(false);
      btnConnect.disabled = false;
      log('Connection error: ' + (e?.message || e));
    });

    ros.on('close', () => {
      setConnected(false);
      btnConnect.disabled = false;
      log('Disconnected');
      try { poseTopic && poseTopic.unsubscribe(); } catch (e) { }
      try { routesTopic && routesTopic.unsubscribe(); } catch (e) { }
      poseTopic = routesTopic = null;
      gotoSrv = runWpSrv = stopWpSrv = null;
      resetProgress();
    });
  }

  function disconnect() {
    if (ros) { ros.close(); }
  }

  function addActionLog(text) {
    const el = document.getElementById("action_log");
    const ts = new Date().toLocaleTimeString();
    const line = `[${ts}] ${text}`;

    // newest first
    actionLogLines.unshift(line);
    if (actionLogLines.length > MAX_LOG_LINES) {
      actionLogLines = actionLogLines.slice(0, MAX_LOG_LINES);
    }

    el.textContent = actionLogLines.join('\n');
    el.scrollTop = 0;
  }

  // ---------------------------
  // Service calls
  // ---------------------------
  function callGoto(x_mm, y_mm) {
    if (!gotoSrv) return log('Goto service not ready.');
    if (!inLimits(x_mm, y_mm)) {
      log(`Blocked by soft limits: (${x_mm}, ${y_mm}) mm`);
      return;
    }
    const req = new ROSLIB.ServiceRequest({
      x: x_mm / 1000.0,
      y: y_mm / 1000.0
    });
    btnGoto.disabled = true;
    gotoSrv.callService(req, (res) => {
      btnGoto.disabled = false;
      log(`[GOTO] success=${res.success} msg="${res.message || ''}"`);
    }, (err) => {
      btnGoto.disabled = false;
      log(`[GOTO] error: ${err}`);
    });
  }

  function callRunRoute() {
    if (!runWpSrv) return log('RunWaypoints service not ready.');
    const req = new ROSLIB.ServiceRequest({
      route: wpRoute.value,
      tol_mm: parseFloat(wpTol.value),
      timeout_s: parseFloat(wpTimeout.value),
      pause_s: parseFloat(wpPause.value),
      loop: (wpLoop.value === 'true')
    });
    btnRunRoute.disabled = true;
    resetProgress();
    runWpSrv.callService(req, (res) => {
      btnRunRoute.disabled = false;
      log(`[RUN] accepted=${res.accepted} msg="${res.message || ''}"`);
    }, (err) => {
      btnRunRoute.disabled = false;
      log(`[RUN] error: ${err}`);
    });
  }

  function callStopRoute() {
    if (!stopWpSrv) return log('Stop service not ready.');
    btnStopRoute.disabled = true;
    stopWpSrv.callService(new ROSLIB.ServiceRequest({}), (res) => {
      btnStopRoute.disabled = false;
      log(`[STOP] success=${res.success} msg="${res.message || ''}"`);
    }, (err) => {
      btnStopRoute.disabled = false;
      log(`[STOP] error: ${err}`);
    });
  }

  // ---------------------------
  // UI wiring
  // ---------------------------
  btnConnect.addEventListener('click', connect);
  btnDisconnect.addEventListener('click', disconnect);

  btnGoto.addEventListener('click', () => {
    const x = parseFloat(gotoX.value);
    const y = parseFloat(gotoY.value);
    callGoto(x, y);
  });

  btnHome.addEventListener('click', () => callGoto(0, 0));

  function jog(dx_mm, dy_mm) {
    const x = (pose.x * 1000) + dx_mm;
    const y = (pose.y * 1000) + dy_mm;
    callGoto(x, y);
    gotoX.value = x.toFixed(3);
    gotoY.value = y.toFixed(3);
  }

  jogXm.addEventListener('click', () => jog(-parseFloat(jogStep.value), 0));
  jogXp.addEventListener('click', () => jog(+parseFloat(jogStep.value), 0));
  jogYm.addEventListener('click', () => jog(0, -parseFloat(jogStep.value)));
  jogYp.addEventListener('click', () => jog(0, +parseFloat(jogStep.value)));

  btnRunRoute.addEventListener('click', callRunRoute);
  btnStopRoute.addEventListener('click', callStopRoute);

  // Keep XY view in sync when limits change
  limMin.addEventListener('change', drawXY);
  limMax.addEventListener('change', drawXY);
  window.addEventListener('resize', resizeXYCanvas);

  // First paint
  resizeXYCanvas();
  log('Ready. Set your rosbridge URL and click Connect (Vacuscan XY).');
})();
