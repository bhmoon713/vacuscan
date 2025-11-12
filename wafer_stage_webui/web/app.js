(() => {
  // ---- Geometry (must match URDF / RViz script) ----
  const corner = 0.34; // base half-size (m)
  const a = 0.152;     // wafer frame anchor offset (m)
  // Corners CCW from TOP-RIGHT: 1..4
  const C = [
    [ +corner, +corner ],  // corner_1
    [ -corner, +corner ],  // corner_2
    [ -corner, -corner ],  // corner_3
    [ +corner, -corner ],  // corner_4
  ];
  // Wafer anchors CCW from TOP-RIGHT: 1..4
  const A = [
    [ +a, +a ],  // anchor_1
    [ -a, +a ],  // anchor_2
    [ -a, -a ],  // anchor_3
    [ +a, -a ],  // anchor_4
  ];

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

  // ---- Bars canvas ----
  const bars = document.getElementById('bars');
  const ctx = bars.getContext('2d');
  const maxMM = 500; // chart scale
  let L = [0,0,0,0]; // lengths in mm

  function um(x){ return Math.round(x*1e6); }

  let ros = null;
  let connected = false;
  let pose = {x:0, y:0};  // meters
  let poseTopic = null, lengthsTopic = null, gotoSrv = null;

  function log(msg) {
    const t = new Date().toLocaleTimeString();
    logEl.innerText = `[${t}] ${msg}\n` + logEl.innerText;
  }

  function setConnected(state) {
    connected = state;
    connState.textContent = state ? 'connected' : 'disconnected';
    connState.style.borderColor = state ? 'transparent' : '#30363d';
    connState.style.background = state ? 'var(--ok)' : 'var(--card)';
    btnDisconnect.disabled = !state;
    btnGoto.disabled = !state;
    [jogXm,jogXp,jogYm,jogYp,btnHome].forEach(b => b.disabled = !state);
  }

  function inLimits(x_mm, y_mm) {
    const min = parseFloat(limMin.value), max = parseFloat(limMax.value);
    return (x_mm >= min && x_mm <= max && y_mm >= min && y_mm <= max);
  }

  function redrawBars() {
    const dpr = window.devicePixelRatio || 1;
    const W = bars.clientWidth * dpr, H = bars.clientHeight * dpr;
    if (bars.width !== W || bars.height !== H) { bars.width = W; bars.height = H; }
    ctx.clearRect(0,0,W,H);

    const pad = 20*dpr;
    const w = (W - pad*2);
    const h = (H - pad*2);

    // axis
    ctx.strokeStyle = '#30363d';
    ctx.lineWidth = 1*dpr;
    ctx.beginPath(); ctx.moveTo(pad, H-pad); ctx.lineTo(W-pad, H-pad); ctx.stroke();

    const labels = ['L1','L2','L3','L4'];
    const n = 4;
    const gap = 14*dpr;
    const barW = (w - gap*(n-1)) / n;
    const scale = h / maxMM;

    for (let i=0;i<n;i++){
      const val = Math.max(0, Math.min(maxMM, L[i]));
      const x = pad + i*(barW+gap);
      const bh = val * scale;
      const y = H - pad - bh;

      ctx.fillStyle = '#58a6ff';
      ctx.fillRect(x, y, barW, bh);

      ctx.fillStyle = '#9da7b3';
      ctx.font = `${12*dpr}px ui-sans-serif`;
      ctx.textAlign = 'center';
      ctx.fillText(labels[i], x+barW/2, H - pad + 14*dpr);

      ctx.fillStyle = '#e6edf3';
      ctx.font = `${12*dpr}px ui-sans-serif`;
      ctx.fillText(val.toFixed(1), x+barW/2, y-6*dpr);
    }

    ctx.fillStyle = '#9da7b3';
    ctx.textAlign = 'left';
    ctx.fillText(`${maxMM} mm`, pad+4*dpr, pad+10*dpr);
  }

  function computeLengthsFromPose() {
    const wa = [pose.x, pose.y]; // m
    const out = [];
    for(let i=0;i<4;i++){
      const start = C[i];
      const end = [ wa[0] + A[i][0], wa[1] + A[i][1] ];
      const dx = end[0]-start[0], dy = end[1]-start[1];
      const len_m = Math.hypot(dx,dy);
      out.push(len_m*1000.0); // to mm
    }
    L = out;
    redrawBars();
  }

  function updatePoseKpis() {
    kpiX.textContent = (pose.x*1000).toFixed(3);
    kpiY.textContent = (pose.y*1000).toFixed(3);
    kpiXum.textContent = um(pose.x);
    kpiYum.textContent = um(pose.y);
  }

  function connect() {
    if (ros) try { ros.close(); } catch(e){}
    ros = new ROSLIB.Ros({ url: wsUrl.value.trim() });
    connState.textContent = 'connecting...'; connState.style.background = '#8b949e';
    btnConnect.disabled = true;

    ros.on('connection', () => {
      setConnected(true);
      btnConnect.disabled = false;
      log(`Connected to ${wsUrl.value}`);

      // Pose
      poseTopic = new ROSLIB.Topic({
        ros, name: '/wafer/pose', messageType: 'geometry_msgs/msg/Pose2D', throttle_rate: 50
      });
      poseTopic.subscribe(msg => {
        pose.x = msg.x; pose.y = msg.y; // meters
        updatePoseKpis();
        if (!lengthsTopic) computeLengthsFromPose();
      });

      // Wire lengths (meters)
      lengthsTopic = new ROSLIB.Topic({
        ros, name: '/wafer/wire_lengths', messageType: 'std_msgs/msg/Float32MultiArray'
      });
      lengthsTopic.subscribe(msg => {
        if (msg.data && msg.data.length >= 4) {
          L = msg.data.slice(0,4).map(v => v*1000.0); // m -> mm
          redrawBars();
        }
      });

      // Goto service
      gotoSrv = new ROSLIB.Service({
        ros, name: '/wafer/goto', serviceType: 'wafer_stage_interfaces/srv/WaferGoto'
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
      try { poseTopic && poseTopic.unsubscribe(); } catch(e){}
      try { lengthsTopic && lengthsTopic.unsubscribe(); } catch(e){}
      poseTopic = lengthsTopic = null;
      gotoSrv = null;
    });
  }

  function disconnect() {
    if (ros) { ros.close(); }
  }

  function callGoto(x_mm, y_mm) {
    if (!gotoSrv) return log('Goto service not ready.');
    if (!inLimits(x_mm, y_mm)) {
      log(`Blocked by soft limits: (${x_mm}, ${y_mm}) mm`);
      return;
    }
    const req = new ROSLIB.ServiceRequest({ x: x_mm/1000.0, y: y_mm/1000.0 });
    btnGoto.disabled = true;
    gotoSrv.callService(req, (res) => {
      btnGoto.disabled = false;
      log(`[GOTO] success=${res.success} msg="${res.message||''}"`);
    }, (err) => {
      btnGoto.disabled = false;
      log(`[GOTO] error: ${err}`);
    });
  }

  // ---- Wire up UI ----
  document.getElementById('btnConnect').addEventListener('click', connect);
  document.getElementById('btnDisconnect').addEventListener('click', disconnect);
  document.getElementById('btnGoto').addEventListener('click', () => {
    const x = parseFloat(gotoX.value), y = parseFloat(gotoY.value);
    callGoto(x,y);
  });
  document.getElementById('btnHome').addEventListener('click', () => callGoto(0,0));

  function jog(dx_mm, dy_mm) {
    const x = (pose.x*1000) + dx_mm;
    const y = (pose.y*1000) + dy_mm;
    callGoto(x,y);
    gotoX.value = x.toFixed(3);
    gotoY.value = y.toFixed(3);
  }
  document.getElementById('jogXm').addEventListener('click', () => jog(-parseFloat(jogStep.value), 0));
  document.getElementById('jogXp').addEventListener('click', () => jog(+parseFloat(jogStep.value), 0));
  document.getElementById('jogYm').addEventListener('click', () => jog(0, -parseFloat(jogStep.value)));
  document.getElementById('jogYp').addEventListener('click', () => jog(0, +parseFloat(jogStep.value)));

  // First paint
  redrawBars();
  log('Ready. Set your rosbridge URL and click Connect.');
})();
