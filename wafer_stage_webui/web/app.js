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

  // ------------- Wire-length chart canvas ------------------
  const bars = document.getElementById("bars");
  const ctx = bars.getContext("2d");
  let L = [0,0,0,0];

  function redrawBars() {
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
        if (totalWaypoints > 0) {
          updateProgressBar( ((currentWaypointIndex+1)/totalWaypoints)*100 );
        }
      });

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
    });

    ros.on("close", () => {
      setConnected(false);
      btnConnect.disabled = false;
      log("Disconnected.");
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

  btnRunRoute.addEventListener("click", callRunRoute);
  btnStopRoute.addEventListener("click", callStopRoute);

  redrawBars();
  log("Vacuscan UI ready.");
})();
