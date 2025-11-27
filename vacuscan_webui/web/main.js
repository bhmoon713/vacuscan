(() => {
  // Geometry — must match URDF
  const corner = 0.34;     // base half-size (m)
  const a = 0.152;         // wafer frame anchor offset (m)
  const C = [ [ +corner, +corner ], [ -corner, +corner ], [ -corner, -corner ], [ +corner, -corner ] ];
  const A = [ [ +a, +a ], [ -a, +a ], [ -a, -a ], [ +a, -a ] ];

  // UI
  const wsEl = document.getElementById('ws');
  const btnConnect = document.getElementById('btnConnect');
  const btnDisconnect = document.getElementById('btnDisconnect');
  const connTag = document.getElementById('connTag');
  const gotoX = document.getElementById('gotoX');
  const gotoY = document.getElementById('gotoY');
  const jogStep = document.getElementById('jogStep');
  const softLim = document.getElementById('softLim');
  const btnGoto = document.getElementById('btnGoto');
  const btnHome = document.getElementById('btnHome');
  const jogXm = document.getElementById('jogXm');
  const jogXp = document.getElementById('jogXp');
  const jogYm = document.getElementById('jogYm');
  const jogYp = document.getElementById('jogYp');
  const jogC  = document.getElementById('jogC');
  const kX = document.getElementById('kX');
  const kY = document.getElementById('kY');
  const kXum = document.getElementById('kXum');
  const kYum = document.getElementById('kYum');
  const Lbars = ['L1_bar','L2_bar','L3_bar','L4_bar'].map(id=>document.getElementById(id));
  const Lvals = ['L1_val','L2_val','L3_val','L4_val'].map(id=>document.getElementById(id));
  const logs = document.getElementById('logs');

  function log(s){ const t=new Date().toLocaleTimeString(); logs.textContent = `[${t}] ${s}\n` + logs.textContent; }

  // State
  let ros=null, pose={x:0,y:0}, gotoSrv=null, poseTopic=null, lengthsTopic=null;
  let receivedLengths=false;   // ← key: compute from pose until actual /wafer/wire_lengths arrives

  function setConnected(on){
    [btnGoto, btnHome, jogXm, jogXp, jogYm, jogYp, jogC, btnDisconnect].forEach(b=>b.disabled=!on);
    connTag.textContent = on ? 'connected' : 'disconnected';
    connTag.style.background = on ? 'var(--ok)' : 'transparent';
  }

  function inLimits(xmm, ymm){
    const lim = parseFloat(softLim.value)||200;
    return (xmm>=-lim && xmm<=lim && ymm>=-lim && ymm<=lim);
  }

  function updatePoseUI(){
    const xmm = pose.x*1000, ymm = pose.y*1000;
    kX.textContent = xmm.toFixed(3);
    kY.textContent = ymm.toFixed(3);
    kXum.textContent = Math.round(pose.x*1e6);
    kYum.textContent = Math.round(pose.y*1e6);
  }

  function computeLengthsFromPose(){
    const wa = [pose.x, pose.y];
    const out = [];
    for (let i=0;i<4;i++){
      const start=C[i];
      const end=[ wa[0]+A[i][0], wa[1]+A[i][1] ];
      out.push(Math.hypot(end[0]-start[0], end[1]-start[1]) * 1000.0); // mm
    }
    applyLengths(out);
  }

  function applyLengths(Lmm){
    const max = Math.max(1, ...Lmm);
    for (let i=0;i<4;i++){
      const pct = Math.max(0, Math.min(100, (Lmm[i]/max)*100));
      Lbars[i].style.width = pct.toFixed(1)+'%';
      Lvals[i].textContent = Lmm[i].toFixed(1);
    }
  }

  function callGoto(xmm, ymm){
    if (!gotoSrv){ log('Goto service not ready'); return; }
    if (!inLimits(xmm, ymm)){ log(`Blocked by soft limits: (${xmm}, ${ymm}) mm`); return; }
    const req = new ROSLIB.ServiceRequest({ x:xmm/1000.0, y:ymm/1000.0 }); // meters
    btnGoto.disabled = true;
    gotoSrv.callService(req, (res)=>{
      btnGoto.disabled=false;
      log(`[GOTO] success=${res.success} msg="${res.message||''}"`);
    }, (err)=>{
      btnGoto.disabled=false;
      log(`[GOTO] error: ${err}`);
    });
  }

  // Connect / Disconnect
  btnConnect.addEventListener('click', ()=>{
    if (ros) try{ ros.close(); }catch(e){}
    ros = new ROSLIB.Ros({ url: wsEl.value.trim() });
    connTag.textContent='connecting...';

    ros.on('connection', ()=>{
      setConnected(true);
      log(`Connected to ${wsEl.value}`);
      // pose
      poseTopic = new ROSLIB.Topic({ ros, name:'/wafer/pose', messageType:'geometry_msgs/msg/Pose2D' });
      poseTopic.subscribe(msg=>{
        pose.x=msg.x; pose.y=msg.y;
        updatePoseUI();
        if (!receivedLengths) computeLengthsFromPose();  // ← update bars from pose until topic arrives
      });
      // optional lengths topic
      lengthsTopic = new ROSLIB.Topic({ ros, name:'/wafer/wire_lengths', messageType:'std_msgs/msg/Float32MultiArray' });
      lengthsTopic.subscribe(msg=>{
        if (msg?.data?.length >= 4){
          receivedLengths = true;
          applyLengths(msg.data.slice(0,4).map(m=>m*1000.0)); // meters→mm
        }
      });
      // service
      gotoSrv = new ROSLIB.Service({ ros, name:'/wafer/goto', serviceType:'wafer_stage_interfaces/srv/WaferGoto' });
    });
    ros.on('close', ()=>{ setConnected(false); log('Disconnected'); receivedLengths=false; });
    ros.on('error', e=>{ setConnected(false); log('Connection error: '+(e?.message||e)); });
  });

  btnDisconnect.addEventListener('click', ()=>{ if (ros) ros.close(); });

  // Motion
  btnGoto.addEventListener('click', ()=> {
    const xmm=parseFloat(gotoX.value)||0, ymm=parseFloat(gotoY.value)||0;
    callGoto(xmm, ymm);
  });
  btnHome.addEventListener('click', ()=>callGoto(0,0));
  function jog(dx,dy){
    const step=parseFloat(jogStep.value)||1;
    const xmm = pose.x*1000 + dx*step, ymm = pose.y*1000 + dy*step;
    gotoX.value=xmm.toFixed(3); gotoY.value=ymm.toFixed(3);
    callGoto(xmm, ymm);
  }
  jogXm.addEventListener('click', ()=>jog(-1,0));
  jogXp.addEventListener('click', ()=>jog(+1,0));
  jogYm.addEventListener('click', ()=>jog(0,-1));
  jogYp.addEventListener('click', ()=>jog(0,+1));
  jogC .addEventListener('click', ()=>callGoto(0,0));

  log('Ready. Set your rosbridge URL and click Connect.');
})();

