import React, { useRef, useEffect, useState, useMemo } from 'react';
import { 
  Activity, ShieldCheck, Boxes, Compass, Navigation, 
  Cpu, Zap, AlertTriangle, CheckCircle2, Thermometer,
  RotateCw, Gauge, Server, RefreshCw, Download, Play, 
  Check, Info, Wrench, Shield, ChevronRight, Search,
  Radio, Layers, Terminal
} from 'lucide-react';
import { batteryView } from '../../utils/battery';
import { ROBOT_SPEC, ROBOT_NODES, wheelRpm } from '../../config/robotSpec';
import rosConnection from '../../ros/rosConnection';

const SPEC = ROBOT_SPEC;
const BODY_RING_M = Number(SPEC.body.footprintCornerM.toFixed(2));
const INFLATION_M = SPEC.navigation.inflationRadiusM;
const fmtDist = (d) => (d == null ? '--' : d.toFixed(2));

export default function DiagnosticsTemplate({
  laserScan,
  robotPose,
  velocity,
  battery,
  systemHealth,
  systemState,
  connected
}) {
  const lidarCanvasRef = useRef(null);
  const [viewZoom, setViewZoom] = useState(4.0); // max meters displayed (2m, 4m, 8m)
  const [nodeSearch, setNodeSearch] = useState('');
  const [isSelfTesting, setIsSelfTesting] = useState(false);
  const [selfTestStep, setSelfTestStep] = useState(0);
  const [selfTestComplete, setSelfTestComplete] = useState(false);
  const [diagnosticNotice, setDiagnosticNotice] = useState('');
  const [selfTestFailed, setSelfTestFailed] = useState(false);

  // Audio beep feedback for industrial hardware test
  const playDiagnosticBeep = (freq = 880, type = 'sine', duration = 0.08) => {
    try {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (!AudioCtx) return;
      const ctx = new AudioCtx();
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.type = type;
      osc.frequency.setValueAtTime(freq, ctx.currentTime);
      gain.gain.setValueAtTime(0.08, ctx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + duration);
      osc.connect(gain);
      gain.connect(ctx.destination);
      osc.start();
      osc.stop(ctx.currentTime + duration);
    } catch (_) {}
  };

  // 1. Compute 4-Sector Real-Time Obstacle Clearances from laserScan
  const sectorData = useMemo(() => {
    let front = 999;
    let right = 999;
    let rear = 999;
    let left = 999;
    let closestOverall = { dist: 999, angleDeg: 0 };
    let totalRays = 0;
    let activeRays = 0;

    if (laserScan?.ranges?.length > 0) { // CBOR delivers ranges as a Float32Array, not an Array
      const ranges = laserScan.ranges;
      totalRays = ranges.length;
      const minAngle = laserScan.angle_min != null ? laserScan.angle_min : -Math.PI;
      const inc = laserScan.angle_increment != null ? laserScan.angle_increment : (Math.PI / 180);
      const minRange = laserScan.range_min || 0.15;
      const maxRange = laserScan.range_max || 12.0;

      for (let i = 0; i < ranges.length; i++) {
        const r = ranges[i];
        if (r >= minRange && r <= maxRange) {
          activeRays++;
          let ang = minAngle + i * inc;
          ang = Math.atan2(Math.sin(ang), Math.cos(ang));
          const deg = ang * (180 / Math.PI);

          if (r < closestOverall.dist) {
            closestOverall = { dist: r, angleDeg: Math.round(deg) };
          }

          // Sector breakdown (relative to Optimus heading):
          // Front: 0° ± 30°
          if (deg >= -30 && deg <= 30) {
            if (r < front) front = r;
          }
          // Left Flank: +90° ± 30° (ROS angles are counter-clockwise, REP-103)
          else if (deg >= 60 && deg <= 120) {
            if (r < left) left = r;
          }
          // Rear: 180° ± 30° (>= 150° or <= -150°)
          else if (deg >= 150 || deg <= -150) {
            if (r < rear) rear = r;
          }
          // Right Flank: -90° ± 30°
          else if (deg >= -120 && deg <= -60) {
            if (r < right) right = r;
          }
        }
      }
    }

    const hasScan = totalRays > 0;
    // Distances are from the LiDAR. The thresholds are the robot's real
    // geometry, not stop/slow zones (the robot has no distance-based stop).
    const getStatus = (dist) => {
      if (!hasScan) return { label: 'NO SCAN', color: 'text-slate-400', bg: 'bg-slate-800/40 border-slate-700', dot: 'bg-slate-400' };
      if (dist < BODY_RING_M) return { label: 'AT BODY EDGE', color: 'text-red-400', bg: 'bg-red-500/10 border-red-500/40', dot: 'bg-red-500' };
      if (dist < INFLATION_M) return { label: 'IN NAV2 INFLATION', color: 'text-amber-400', bg: 'bg-amber-500/10 border-amber-500/40', dot: 'bg-amber-500' };
      return { label: 'CLEAR', color: 'text-emerald-400', bg: 'bg-emerald-500/10 border-emerald-500/40', dot: 'bg-emerald-500' };
    };
    const maxRange = SPEC.lidar.rangeM;
    const sector = (d) => ({ dist: !hasScan ? null : d === 999 ? maxRange : d, status: getStatus(d) });

    return {
      front: sector(front),
      right: sector(right),
      rear: sector(rear),
      left: sector(left),
      closest: !hasScan ? { dist: null, angleDeg: 0 } : closestOverall.dist === 999 ? { dist: maxRange, angleDeg: 0 } : closestOverall,
      totalRays,
      activeRays
    };
  }, [laserScan]);

  // 2. Draw 360° LiDAR Safety Envelope & Obstacle Field (Technical Canvas, No Fake Radar)
  useEffect(() => {
    const canvas = lidarCanvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext('2d');
    const size = canvas.width = canvas.height = 360;
    const center = size / 2;
    const radius = center - 18;

    // Technical Blueprint Grid Background
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, size, size);

    // Subtle Grid pattern
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    const step = 30;
    for (let x = step; x < size; x += step) {
      ctx.beginPath();
      ctx.moveTo(x, 0);
      ctx.lineTo(x, size);
      ctx.stroke();
    }
    for (let y = step; y < size; y += step) {
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(size, y);
      ctx.stroke();
    }

    // Outer Sensor FOV Boundary
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(center, center, radius, 0, Math.PI * 2);
    ctx.stroke();

    // Concentric Range Rings (e.g. 1m, 2m, 3m, 4m based on viewZoom)
    // Reference rings: robot body corner, Nav2 inflation radius, then 1 m steps.
    const ringSteps = [BODY_RING_M, INFLATION_M, ...(viewZoom === 2 ? [1, 2] : viewZoom === 8 ? [2, 4, 6, 8] : [1, 2, 3, 4])];
    ringSteps.forEach(dist => {
      const ringPx = (dist / viewZoom) * radius;
      if (ringPx > radius) return;
      const special = dist === BODY_RING_M ? { color: 'rgba(239, 68, 68, 0.75)', label: `${dist}m BODY` }
        : dist === INFLATION_M ? { color: 'rgba(245, 158, 11, 0.7)', label: `${dist}m INFLATION` } : null;
      ctx.strokeStyle = special ? special.color : '#334155';
      ctx.lineWidth = special ? 1.5 : 1;
      ctx.setLineDash(special ? [4, 3] : []);
      ctx.beginPath();
      ctx.arc(center, center, ringPx, 0, Math.PI * 2);
      ctx.stroke();
      ctx.setLineDash([]);
      // Label the small body/inflation rings only when zoomed in enough to read them.
      if (special && ringPx < 28) return;
      ctx.fillStyle = special ? special.color : '#64748b';
      ctx.font = special ? 'bold 7.5px JetBrains Mono' : '8px JetBrains Mono';
      if (special) {
        ctx.fillText(special.label, center + ringPx * 0.71 + 3, center + (dist === INFLATION_M ? 1 : -1) * ringPx * 0.71);
      } else {
        ctx.fillText(`${dist}m`, center + ringPx + 3, center - 3);
      }
    });

    // Sector Division Lines (45° diagonals & 90° axes)
    ctx.strokeStyle = '#1e293b';
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(center - radius, center);
    ctx.lineTo(center + radius, center);
    ctx.moveTo(center, center - radius);
    ctx.lineTo(center, center + radius);
    ctx.stroke();

    // Sector Cone Guides (±30° from Front 0° heading)
    const coneAngle = 30 * (Math.PI / 180);
    ctx.strokeStyle = 'rgba(249, 115, 22, 0.25)';
    ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath();
    ctx.moveTo(center, center);
    ctx.lineTo(center + radius * Math.sin(coneAngle), center - radius * Math.cos(coneAngle));
    ctx.moveTo(center, center);
    ctx.lineTo(center - radius * Math.sin(coneAngle), center - radius * Math.cos(coneAngle));
    ctx.stroke();
    ctx.setLineDash([]);

    // Cardinal Heading Labels
    ctx.fillStyle = '#94a3b8';
    ctx.font = 'bold 9px JetBrains Mono';
    ctx.fillText('FRONT (0°)', center - 24, center - radius + 14);
    ctx.fillText('REAR (180°)', center - 26, center + radius - 6);
    ctx.fillText('R (-90°)', center + radius - 42, center + 14);
    ctx.fillText('L (+90°)', center - radius + 6, center + 14);

    // Plot Laser Reflection Points from /scan
    if (laserScan?.ranges?.length > 0) { // CBOR delivers ranges as a Float32Array, not an Array
      const ranges = laserScan.ranges;
      const minAngle = laserScan.angle_min != null ? laserScan.angle_min : -Math.PI;
      const inc = laserScan.angle_increment != null ? laserScan.angle_increment : (Math.PI / 180);

      ctx.fillStyle = '#ef4444';
      ctx.shadowColor = 'rgba(239, 68, 68, 0.6)';
      ctx.shadowBlur = 4;

      for (let i = 0; i < ranges.length; i += 2) {
        const r = ranges[i];
        if (r >= (laserScan.range_min || 0.15) && r <= viewZoom) {
          // 0° (robot front) points up; positive angles (robot's left) go left.
          const ang = minAngle + i * inc;
          const px = center - (r / viewZoom) * radius * Math.sin(ang);
          const py = center - (r / viewZoom) * radius * Math.cos(ang);

          ctx.beginPath();
          ctx.arc(px, py, 2.2, 0, Math.PI * 2);
          ctx.fill();
        }
      }
      ctx.shadowBlur = 0;
    } else {
      // No /scan yet: say so instead of drawing placeholder reflections.
      ctx.fillStyle = '#64748b';
      ctx.font = 'bold 9px JetBrains Mono';
      ctx.fillText('NO /scan DATA', center - 34, center + 46);
    }

    // Draw Optimus AMR Technical Silhouette at Center
    // Heading is UP (0° Front)
    const chassisTop = center - 8;
    const axleY = center + 8;

    // Chassis Body (Safety Orange)
    ctx.fillStyle = '#ea580c';
    ctx.strokeStyle = '#090d16';
    ctx.lineWidth = 1.8;
    ctx.beginPath();
    ctx.roundRect(center - 11, chassisTop, 22, 28, 4);
    ctx.fill();
    ctx.stroke();

    // Bumblebee Matte Black Racing Stripes
    ctx.fillStyle = '#090d16';
    ctx.fillRect(center - 5, chassisTop + 2, 3, 24);
    ctx.fillRect(center + 2, chassisTop + 2, 3, 24);

    // Differential Drive Wheels on Axle
    ctx.fillStyle = '#020617';
    ctx.strokeStyle = '#f97316';
    ctx.lineWidth = 1;
    ctx.fillRect(center - 15, axleY - 4, 3, 9);
    ctx.strokeRect(center - 15, axleY - 4, 3, 9);
    ctx.fillRect(center + 12, axleY - 4, 3, 9);
    ctx.strokeRect(center + 12, axleY - 4, 3, 9);

    // Fixed Forward LiDAR Mount (forward of base_link, see ROBOT_SPEC.lidar)
    // Displayed at front of chassis
    const lidarY = chassisTop + 2;
    ctx.fillStyle = '#090d16';
    ctx.strokeStyle = '#38bdf8';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(center, lidarY, 4.5, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    // Laser Diode Center
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.arc(center, lidarY, 1.8, 0, Math.PI * 2);
    ctx.fill();

    // URDF Offset Annotation
    ctx.fillStyle = '#38bdf8';
    ctx.font = 'bold 7.5px JetBrains Mono';
    ctx.fillText(`+${SPEC.lidar.mountForwardM.toFixed(2)}m OFFSET`, center - 26, lidarY - 7);
  }, [laserScan, viewZoom, sectorData]);

  // Wheel speeds from the measured body velocity (/odom) and the real
  // drivetrain geometry. The ZLAC8015D does not report current or
  // temperature to the robot software, so those are not shown.
  const { left: rpmLeft, right: rpmRight } = wheelRpm(velocity?.linear, velocity?.angular);
  const bat = batteryView(battery);
  const batVolt = bat.voltText;
  const batPct = bat.pctText;

  // Live ROS graph: which of the robot's nodes are actually running (rosapi).
  const [runningNodes, setRunningNodes] = useState(null);
  useEffect(() => {
    if (!connected) {
      setRunningNodes(null);
      return undefined;
    }
    let cancelled = false;
    const poll = async () => {
      const names = await rosConnection.getNodeNames();
      if (!cancelled) setRunningNodes(names);
    };
    poll();
    const timer = setInterval(poll, 5000);
    return () => {
      cancelled = true;
      clearInterval(timer);
    };
  }, [connected]);

  const rosNodes = ROBOT_NODES.map((n) => ({
    ...n,
    running: runningNodes === null ? null : runningNodes.includes(n.name),
  }));
  const coreNodes = rosNodes.filter((n) => n.group === 'CORE');
  const coreRunning = coreNodes.filter((n) => n.running).length;

  const filteredNodes = rosNodes.filter(n => 
    n.name.toLowerCase().includes(nodeSearch.toLowerCase()) ||
    n.role.toLowerCase().includes(nodeSearch.toLowerCase()) ||
    n.group.toLowerCase().includes(nodeSearch.toLowerCase()) ||
    n.topic.toLowerCase().includes(nodeSearch.toLowerCase())
  );

  // Self-test: checks the live robot health and ROS graph (no simulated passes).
  const SELF_TEST_STEPS = 5;
  const runChecks = () => {
    const comps = systemHealth?.components || {};
    const ok = (key) => comps[key]?.status === 'OK';
    return [
      { name: 'Dashboard connected to rosbridge', pass: !!connected },
      { name: `LiDAR /scan (${SPEC.lidar.model})`, pass: ok('lidar') && sectorData.totalRays > 0 },
      { name: `Drive /joint_states (${SPEC.driver.model})`, pass: ok('controller') },
      { name: 'Wheel odometry /odom', pass: ok('odometry') },
      { name: 'TF odom → base_link → lidar_link', pass: ok('tf') },
      { name: `Core ROS nodes running (${coreRunning}/${coreNodes.length})`, pass: runningNodes !== null && coreRunning === coreNodes.length },
    ];
  };

  const handleRunSelfTest = () => {
    if (isSelfTesting) return;
    setIsSelfTesting(true);
    setSelfTestStep(1);
    setSelfTestComplete(false);
    playDiagnosticBeep(520, 'sine', 0.1);

    for (let step = 2; step <= SELF_TEST_STEPS; step += 1) {
      setTimeout(() => setSelfTestStep(step), (step - 1) * 500);
    }

    setTimeout(() => {
      const checks = runChecks();
      const failed = checks.filter((c) => !c.pass);
      setIsSelfTesting(false);
      setSelfTestComplete(true);
      setSelfTestFailed(failed.length > 0);
      playDiagnosticBeep(failed.length ? 300 : 1200, failed.length ? 'square' : 'triangle', 0.2);
      setDiagnosticNotice(failed.length
        ? `Self-test FAILED (${checks.length - failed.length}/${checks.length}): ${failed.map((c) => c.name).join(' · ')}`
        : `Self-test PASSED: all ${checks.length} checks OK`);
      setTimeout(() => setDiagnosticNotice(''), 8000);
    }, SELF_TEST_STEPS * 500);
  };

  // Export JSON Diagnostic Snapshot Bundle
  const handleExportDiagnostics = () => {
    const comps = systemHealth?.components || {};
    const report = {
      robot: SPEC.name,
      timestamp: new Date().toISOString(),
      connected: !!connected,
      overall_health: systemHealth?.overall ?? null,
      system_state: systemState?.state ?? null,
      specification: SPEC,
      live: {
        lidar: {
          status: comps.lidar ?? null,
          rays_in_last_scan: sectorData.totalRays,
          valid_returns: sectorData.activeRays,
          sector_clearance_m: {
            front: sectorData.front.dist,
            left: sectorData.left.dist,
            rear: sectorData.rear.dist,
            right: sectorData.right.dist
          },
          closest_obstacle: { distance_m: sectorData.closest.dist, heading_deg: sectorData.closest.angleDeg }
        },
        drive: {
          status: comps.controller ?? null,
          body_velocity: { linear_mps: Number(velocity?.linear || 0), angular_radps: Number(velocity?.angular || 0) },
          wheel_rpm_from_odometry: { left: rpmLeft, right: rpmRight },
          motor_current_temperature: 'not reported by the ZLAC8015D driver integration'
        },
        odometry: comps.odometry ?? null,
        tf: comps.tf ?? null,
        battery: { telemetry: SPEC.power.batteryTelemetry, voltage_v: bat.volt, percentage: bat.pct },
        ros_nodes: rosNodes.map(({ name, group, running }) => ({ name, group, running }))
      }
    };

    const blob = new Blob([JSON.stringify(report, null, 2)], { type: 'application/json' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = `optimus-diagnostics-${Date.now()}.json`;
    a.click();
    URL.revokeObjectURL(url);
  };

  return (
    <div className="flex flex-col gap-3 flex-1 font-mono">
      {/* 1. Top Diagnostic Header & Quick Actions */}
      <div className="clay-card p-4 flex flex-col md:flex-row items-start md:items-center justify-between gap-3">
        <div>
          <div className="flex items-center gap-2">
            <Activity className="w-5 h-5 text-orange-500 animate-pulse" />
            <h2 className="text-sm font-black text-slate-50 tracking-tight">
              OPTIMUS AMR — HARDWARE & SENSORS DIAGNOSTICS
            </h2>
            <span className={`text-[10px] px-2 py-0.5 rounded-full font-extrabold border ${
              !connected || !systemHealth?.overall || systemHealth.overall === 'UNKNOWN' ? 'bg-slate-800/70 text-slate-300 border-slate-700'
                : systemHealth?.overall === 'OK' ? 'bg-emerald-500/15 text-emerald-200 border-emerald-500/40'
                  : systemHealth?.overall === 'WARNING' ? 'bg-amber-500/15 text-amber-200 border-amber-500/40'
                    : 'bg-red-500/15 text-red-200 border-red-500/40'
            }`}>
              {!connected ? 'OFFLINE' : `HEALTH: ${systemHealth?.overall || 'UNKNOWN'}`}
            </span>
          </div>
          <p className="text-xs text-slate-400 mt-0.5">
            {SPEC.lidar.model} • {SPEC.driver.model} {SPEC.driver.bus} • {SPEC.software.ros}
          </p>
        </div>

        {/* Action Buttons: Self-Test + Export */}
        <div className="flex items-center gap-2 w-full md:w-auto">
          <button
            onClick={handleRunSelfTest}
            disabled={isSelfTesting}
            className={`btn-clay-orange text-xs px-3.5 py-2 flex items-center justify-center gap-1.5 flex-1 md:flex-none font-extrabold shadow-xs cursor-pointer active:scale-95 ${
              isSelfTesting ? 'opacity-75 cursor-wait' : ''
            }`}
            title="Check live robot health: connection, LiDAR, drive, odometry, TF and core ROS nodes"
          >
            <Play className={`w-3.5 h-3.5 ${isSelfTesting ? 'animate-spin' : ''}`} />
            <span>{isSelfTesting ? `TESTING (${selfTestStep}/${SELF_TEST_STEPS})...` : 'RUN SELF-TEST'}</span>
          </button>

          <button
            onClick={handleExportDiagnostics}
            className="btn-clay-white text-xs px-3 py-2 flex items-center justify-center gap-1.5 text-slate-200 hover:text-slate-50 font-bold shadow-xs cursor-pointer active:scale-95 shrink-0"
            title="Export the robot specification and live status as JSON"
          >
            <Download className="w-3.5 h-3.5 text-orange-500" />
            <span>EXPORT REPORT</span>
          </button>
        </div>
      </div>

      {/* Toast Alert Notice */}
      {diagnosticNotice && (
        <div className={`px-4 py-2 rounded-xl border text-xs font-bold flex items-center gap-2 shadow-sm animate-in fade-in ${
          selfTestFailed ? 'bg-red-500/10 border-red-500/40 text-red-200' : 'bg-emerald-500/10 border-emerald-500/40 text-emerald-200'
        }`}>
          {selfTestFailed
            ? <AlertTriangle className="w-4 h-4 text-red-400 shrink-0" />
            : <CheckCircle2 className="w-4 h-4 text-emerald-400 shrink-0" />}
          <span>{diagnosticNotice}</span>
        </div>
      )}

      {/* 2. Top Banner: 4 Core Hardware Metric Tiles */}
      <div className="grid grid-cols-2 lg:grid-cols-4 gap-3">
        {/* LiDAR Card */}
        <div className="clay-card p-3.5 flex items-center justify-between hover:border-slate-700 transition-all">
          <div>
            <div className="text-[10px] text-slate-400 font-bold uppercase tracking-wider">{SPEC.lidar.model}</div>
            <div className="text-xl font-black text-slate-50">
              {sectorData.totalRays || '--'} <span className="text-xs text-slate-400 font-normal">rays • {SPEC.lidar.fovDeg}°</span>
            </div>
            <div className={`text-[10px] font-bold flex items-center gap-1 ${systemHealth?.components?.lidar?.status === 'OK' ? 'text-emerald-400' : 'text-amber-400'}`}>
              <span className={`h-1.5 w-1.5 rounded-full ${systemHealth?.components?.lidar?.status === 'OK' ? 'bg-emerald-500' : 'bg-amber-500'}`}></span>
              {connected ? (systemHealth?.components?.lidar?.message || 'Waiting for /scan') : `~${SPEC.lidar.nominalRateHz} Hz nominal • offline`}
            </div>
          </div>
          <div className="p-2.5 rounded-2xl bg-orange-500/10 border border-orange-500/30 text-orange-400 shadow-2xs">
            <Radio className="w-5 h-5" />
          </div>
        </div>

        {/* RS485 Bus Card */}
        <div className="clay-card p-3.5 flex items-center justify-between hover:border-slate-700 transition-all">
          <div>
            <div className="text-[10px] text-slate-400 font-bold uppercase tracking-wider">{SPEC.driver.bus}</div>
            <div className="text-xl font-black text-slate-50">
              {SPEC.driver.baud.toLocaleString()} <span className="text-xs text-slate-400 font-normal">baud</span>
            </div>
            <div className={`text-[10px] font-bold ${systemHealth?.components?.controller?.status === 'OK' ? 'text-emerald-400' : 'text-amber-400'}`}>
              ● SLAVE ID {SPEC.driver.slaveId} • {connected ? (systemHealth?.components?.controller?.message || 'Waiting for joint states') : 'offline'}
            </div>
          </div>
          <div className="p-2.5 rounded-2xl bg-sky-500/10 border border-sky-500/30 text-sky-400 shadow-2xs">
            <Zap className="w-5 h-5" />
          </div>
        </div>

        {/* ZLAC8015D Drivers Card */}
        <div className="clay-card p-3.5 flex items-center justify-between hover:border-slate-700 transition-all">
          <div>
            <div className="text-[10px] text-slate-400 font-bold uppercase tracking-wider">{SPEC.driver.model} L / R WHEEL</div>
            <div className="text-xl font-black text-slate-50">
              {rpmLeft} / {rpmRight} <span className="text-xs text-slate-400 font-normal">RPM</span>
            </div>
            <div className="text-[10px] text-slate-300 font-bold">FROM ODOMETRY • RATED {SPEC.motor.ratedRpm} RPM</div>
          </div>
          <div className="p-2.5 rounded-2xl bg-emerald-500/10 border border-emerald-500/30 text-emerald-400 shadow-2xs">
            <Boxes className="w-5 h-5" />
          </div>
        </div>

        {/* Power Rail Card */}
        <div className="clay-card p-3.5 flex items-center justify-between hover:border-slate-700 transition-all">
          <div>
            <div className="text-[10px] text-slate-400 font-bold uppercase tracking-wider">{SPEC.power.nominalV} V SUPPLY • BATTERY SIMULATED</div>
            <div className="text-xl font-black text-orange-400">
              {batVolt} <span className="text-xs text-slate-400 font-normal">V</span>
            </div>
            <div className="text-[10px] text-slate-300 font-bold">
              {batPct}% • E-STOP: {systemState?.is_emergency_stop ? 'TRIPPED' : 'ARMED'}
            </div>
          </div>
          <div className="p-2.5 rounded-2xl bg-orange-500/10 border border-orange-500/30 text-orange-400 shadow-2xs">
            <Gauge className="w-5 h-5" />
          </div>
        </div>
      </div>

      {/* 3. Main Diagnostics Grid: LiDAR Clearance vs Motor/Node Telemetry */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-3 items-stretch flex-1">
        {/* Left: 360° LiDAR Safety Envelope & Clearance Analyzer (5 cols) */}
        <div className="lg:col-span-5 flex flex-col clay-card overflow-hidden">
          <div className="panel-header py-2.5 px-4 bg-slate-950 border-b border-slate-800 text-white flex items-center justify-between">
            <div className="flex items-center gap-2">
              <Radio className="w-4 h-4 text-orange-500" />
              <span className="font-bold text-xs text-white tracking-wider">
                360° LIDAR SAFETY ENVELOPE
              </span>
            </div>

            {/* Range Zoom selector */}
            <div className="flex items-center gap-1 text-[10px]">
              <span className="text-slate-400 font-semibold">FOV:</span>
              {[2, 4, 8].map(z => (
                <button
                  key={z}
                  onClick={() => setViewZoom(z)}
                  className={`px-2 py-0.5 rounded-lg cursor-pointer transition-all ${
                    viewZoom === z
                      ? 'bg-gradient-to-r from-orange-500 to-orange-600 text-white font-extrabold shadow-2xs'
                      : 'bg-slate-800 border border-slate-700 text-slate-300 hover:text-white hover:bg-slate-700'
                  }`}
                >
                  {z}m
                </button>
              ))}
            </div>
          </div>

          {/* Technical LiDAR Canvas */}
          <div className="p-3.5 flex flex-col items-center justify-center bg-slate-950">
            <canvas
              ref={lidarCanvasRef}
              className="rounded-2xl shadow-inner border border-slate-800 max-w-[310px] max-h-[310px] w-full h-auto aspect-square bg-[#0f172a]"
            />
          </div>

          {/* 4-Sector Clearance Grid Matrix */}
          <div className="p-3 bg-slate-900 border-t border-slate-800 grid grid-cols-2 sm:grid-cols-4 gap-2 text-center text-xs">
            {/* Front */}
            <div className={`p-2 rounded-xl border ${sectorData.front.status.bg}`}>
              <div className="text-[9px] font-bold text-slate-400 uppercase">FRONT (0°)</div>
              <div className="text-sm font-black text-slate-50">
                {fmtDist(sectorData.front.dist)}m
              </div>
              <div className={`text-[8.5px] font-extrabold ${sectorData.front.status.color}`}>
                {sectorData.front.status.label}
              </div>
            </div>

            {/* Right */}
            <div className={`p-2 rounded-xl border ${sectorData.right.status.bg}`}>
              <div className="text-[9px] font-bold text-slate-400 uppercase">RIGHT (-90°)</div>
              <div className="text-sm font-black text-slate-50">
                {fmtDist(sectorData.right.dist)}m
              </div>
              <div className={`text-[8.5px] font-extrabold ${sectorData.right.status.color}`}>
                {sectorData.right.status.label}
              </div>
            </div>

            {/* Rear */}
            <div className={`p-2 rounded-xl border ${sectorData.rear.status.bg}`}>
              <div className="text-[9px] font-bold text-slate-400 uppercase">REAR (180°)</div>
              <div className="text-sm font-black text-slate-50">
                {fmtDist(sectorData.rear.dist)}m
              </div>
              <div className={`text-[8.5px] font-extrabold ${sectorData.rear.status.color}`}>
                {sectorData.rear.status.label}
              </div>
            </div>

            {/* Left */}
            <div className={`p-2 rounded-xl border ${sectorData.left.status.bg}`}>
              <div className="text-[9px] font-bold text-slate-400 uppercase">LEFT (+90°)</div>
              <div className="text-sm font-black text-slate-50">
                {fmtDist(sectorData.left.dist)}m
              </div>
              <div className={`text-[8.5px] font-extrabold ${sectorData.left.status.color}`}>
                {sectorData.left.status.label}
              </div>
            </div>
          </div>

          {/* Technical Specs Footer */}
          <div className="p-3 border-t border-slate-800 bg-slate-800/40 text-[10px] text-slate-300 flex items-center justify-between">
            <span>{SPEC.lidar.model} • {sectorData.totalRays || 0} rays</span>
            <span className="font-bold text-slate-100">Closest: {fmtDist(sectorData.closest.dist)}m @ {sectorData.closest.angleDeg}°</span>
            <span className="text-orange-400 font-bold">Offset: +{SPEC.lidar.mountForwardM.toFixed(2)}m</span>
          </div>
        </div>

        {/* Right: Motors + Kinematics + ROS 2 Node Graph (7 cols) */}
        <div className="lg:col-span-7 flex flex-col gap-3">
          {/* Dual Motor Driver Real-Time Telemetry */}
          <div className="clay-card p-4 space-y-3">
            <div className="flex items-center justify-between border-b border-slate-800 pb-2">
              <div className="flex items-center gap-2">
                <Boxes className="w-4 h-4 text-orange-500" />
                <span className="text-xs font-bold text-slate-50 uppercase tracking-wide">
                  {SPEC.driver.model} • 2 × {SPEC.motor.model.toUpperCase()}
                </span>
              </div>
              <span className="text-[10px] px-2 py-0.5 rounded-full bg-emerald-500/15 text-emerald-200 border border-emerald-500/40 font-bold">
                RS485: {SPEC.driver.baud} BAUD • ID {SPEC.driver.slaveId}
              </span>
            </div>

            <div className="grid grid-cols-1 sm:grid-cols-2 gap-3">
              {/* Left Motor Card */}
              <div className="clay-inset p-3.5 rounded-2xl border border-slate-800/90 space-y-2">
                <div className="flex items-center justify-between text-xs">
                  <span className="font-bold text-slate-50 flex items-center gap-1.5">
                    <span className="h-2 w-2 rounded-full bg-emerald-500"></span>
                    LEFT CHANNEL
                  </span>
                  <span className="text-orange-400 font-black">{rpmLeft} RPM</span>
                </div>
                <div className="grid grid-cols-3 gap-1.5 text-[10px] pt-1">
                  <div className="bg-slate-900 border border-slate-800 p-2 rounded-xl text-center shadow-2xs">
                    <span className="text-slate-400 block text-[9px]">RATED / PEAK</span>
                    <span className="text-slate-100 font-bold">{SPEC.motor.ratedRpm} / {SPEC.motor.peakRpm} RPM</span>
                  </div>
                  <div className="bg-slate-900 border border-slate-800 p-2 rounded-xl text-center shadow-2xs">
                    <span className="text-slate-400 block text-[9px]">TORQUE</span>
                    <span className="text-slate-100 font-bold">{SPEC.motor.ratedTorqueNm} / {SPEC.motor.peakTorqueNm} Nm</span>
                  </div>
                  <div className="bg-slate-900 border border-slate-800 p-2 rounded-xl text-center shadow-2xs">
                    <span className="text-slate-400 block text-[9px]">CURRENT</span>
                    <span className="text-slate-100 font-bold">{SPEC.motor.ratedCurrentA} / {SPEC.motor.peakCurrentA} A</span>
                  </div>
                </div>
              </div>

              {/* Right Motor Card */}
              <div className="clay-inset p-3.5 rounded-2xl border border-slate-800/90 space-y-2">
                <div className="flex items-center justify-between text-xs">
                  <span className="font-bold text-slate-50 flex items-center gap-1.5">
                    <span className="h-2 w-2 rounded-full bg-emerald-500"></span>
                    RIGHT CHANNEL
                  </span>
                  <span className="text-orange-400 font-black">{rpmRight} RPM</span>
                </div>
                <div className="grid grid-cols-3 gap-1.5 text-[10px] pt-1">
                  <div className="bg-slate-900 border border-slate-800 p-2 rounded-xl text-center shadow-2xs">
                    <span className="text-slate-400 block text-[9px]">RATED / PEAK</span>
                    <span className="text-slate-100 font-bold">{SPEC.motor.ratedRpm} / {SPEC.motor.peakRpm} RPM</span>
                  </div>
                  <div className="bg-slate-900 border border-slate-800 p-2 rounded-xl text-center shadow-2xs">
                    <span className="text-slate-400 block text-[9px]">TORQUE</span>
                    <span className="text-slate-100 font-bold">{SPEC.motor.ratedTorqueNm} / {SPEC.motor.peakTorqueNm} Nm</span>
                  </div>
                  <div className="bg-slate-900 border border-slate-800 p-2 rounded-xl text-center shadow-2xs">
                    <span className="text-slate-400 block text-[9px]">CURRENT</span>
                    <span className="text-slate-100 font-bold">{SPEC.motor.ratedCurrentA} / {SPEC.motor.peakCurrentA} A</span>
                  </div>
                </div>
              </div>
            </div>

            {/* Drivetrain specification strip */}
            <div className="p-2.5 rounded-xl bg-slate-800/40 border border-slate-800/90 grid grid-cols-2 sm:grid-cols-4 gap-2 text-[10px] text-slate-300">
              <div>
                <span className="text-slate-400 block">WHEEL SEPARATION</span>
                <span className="font-bold text-slate-100">{Math.round(SPEC.drivetrain.wheelSeparationM * 1000)} mm</span>
              </div>
              <div>
                <span className="text-slate-400 block">WHEEL DIAMETER</span>
                <span className="font-bold text-slate-100">{SPEC.drivetrain.wheelDiameterMm} mm</span>
              </div>
              <div>
                <span className="text-slate-400 block">SPEED LIMIT</span>
                <span className="font-bold text-slate-100">{SPEC.drivetrain.maxLinearMps} m/s • {SPEC.drivetrain.maxAngularRadps} rad/s</span>
              </div>
              <div>
                <span className="text-slate-400 block">ENCODER</span>
                <span className="font-bold text-slate-100">{SPEC.motor.encoderLines} lines</span>
              </div>
            </div>
          </div>

          {/* ROS 2 Humble Node & Topic Graph Monitor */}
          <div className="clay-card flex-1 flex flex-col overflow-hidden">
            <div className="panel-header py-2.5 px-4 bg-slate-950 border-b border-slate-800 text-white flex items-center justify-between">
              <div className="flex items-center gap-2">
                <Server className="w-4 h-4 text-orange-500" />
                <span className="text-xs font-bold text-white tracking-wide">
                  ROS GRAPH • {runningNodes === null ? 'UNAVAILABLE' : `${coreRunning}/${coreNodes.length} CORE NODES RUNNING`}
                </span>
              </div>

              {/* Node Search Bar */}
              <div className="relative w-44 sm:w-56">
                <Search className="w-3 h-3 absolute left-2.5 top-1/2 -translate-y-1/2 text-slate-400" />
                <input
                  type="text"
                  value={nodeSearch}
                  onChange={(e) => setNodeSearch(e.target.value)}
                  placeholder="Filter nodes..."
                  className="w-full bg-slate-900 border border-slate-700 rounded-lg pl-7 pr-2 py-1 text-[10px] text-white placeholder-slate-400 focus:outline-none focus:border-orange-500 font-mono"
                />
              </div>
            </div>

            <div className="p-3 flex-1 overflow-y-auto space-y-1.5 max-h-[260px] smooth-scroll">
              <div className="grid grid-cols-1 sm:grid-cols-2 gap-2">
                {filteredNodes.map(node => (
                  <div
                    key={node.name}
                    className="p-2.5 rounded-xl clay-inset bg-slate-800/40 border border-slate-800 text-xs flex items-center justify-between hover:border-slate-700 transition-all shadow-2xs"
                  >
                    <div className="min-w-0 pr-2">
                      <div className="text-slate-50 font-bold truncate text-[11px] flex items-center gap-1.5">
                        {node.running
                          ? <CheckCircle2 className="w-3.5 h-3.5 text-emerald-400 shrink-0" />
                          : <AlertTriangle className={`w-3.5 h-3.5 shrink-0 ${node.group === 'CORE' && node.running === false ? 'text-red-500' : 'text-slate-400'}`} />}
                        <span className="truncate">{node.name}</span>
                      </div>
                      <div className="text-[10px] text-slate-400 truncate">
                        {node.role} • {node.topic}
                      </div>
                    </div>
                    <span className={`px-2 py-0.5 rounded-md font-bold text-[10px] border shrink-0 ${
                      node.running ? 'bg-emerald-500/10 text-emerald-300 border-emerald-500/30'
                        : node.running === false && node.group === 'CORE' ? 'bg-red-500/10 text-red-300 border-red-500/30'
                          : 'bg-slate-800/40 text-slate-400 border-slate-800'
                    }`}>
                      {node.running === null ? 'UNKNOWN' : node.running ? 'RUNNING' : node.group === 'CORE' ? 'MISSING' : `OFF • ${node.group}`}
                    </span>
                  </div>
                ))}
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
