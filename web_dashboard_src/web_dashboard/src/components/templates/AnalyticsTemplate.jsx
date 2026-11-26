import React, { useState, useEffect, useRef } from 'react';
import {
  Terminal, Search, Download, Trash2, ArrowDownCircle
} from 'lucide-react';
import { batteryView } from '../../utils/battery';
import { ROBOT_SPEC } from '../../config/robotSpec';

const timeNow = () => new Date().toTimeString().split(' ')[0];
const fmtDuration = (ms) => {
  const t = Math.max(0, Math.floor(ms / 1000));
  const hh = String(Math.floor(t / 3600)).padStart(2, '0');
  const mm = String(Math.floor((t % 3600) / 60)).padStart(2, '0');
  const ss = String(t % 60).padStart(2, '0');
  return `${hh}:${mm}:${ss}`;
};

/**
 * Session analytics: every number is measured live from the robot since this
 * dashboard was opened. Nothing is pre-filled; the robot does not keep a
 * lifetime log the dashboard could read.
 */
export default function AnalyticsTemplate({
  systemState,
  systemHealth,
  robotPose,
  velocity,
  battery,
  navStatus,
  connected
}) {
  const [logs, setLogs] = useState(() => [{
    id: 1,
    time: timeNow(),
    level: 'INFO',
    category: 'DASHBOARD',
    msg: 'Session started. Showing live robot events from this session only.'
  }]);
  const [autoScroll, setAutoScroll] = useState(true);
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedLevel, setSelectedLevel] = useState('ALL');
  const logContainerRef = useRef(null);

  const [sessionStart] = useState(() => Date.now());
  const [now, setNow] = useState(() => Date.now());
  const [distanceM, setDistanceM] = useState(0);
  const [peakSpeed, setPeakSpeed] = useState(0);
  const [goals, setGoals] = useState({ succeeded: 0, failed: 0, canceled: 0 });
  const [estopCount, setEstopCount] = useState(0);
  const [healthErrors, setHealthErrors] = useState(0);

  const logIdRef = useRef(2);
  const addLog = (level, category, msg) => {
    const entry = { id: logIdRef.current++, time: timeNow(), level, category, msg };
    setLogs((prev) => [...prev.slice(-199), entry]);
  };

  // Auto-scroll when new logs arrive if autoScroll is enabled
  useEffect(() => {
    if (autoScroll && logContainerRef.current) {
      logContainerRef.current.scrollTop = logContainerRef.current.scrollHeight;
    }
  }, [logs, autoScroll]);

  // Session clock
  useEffect(() => {
    const timer = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(timer);
  }, []);

  // Distance travelled this session, from the live pose. Jumps (frame switch
  // odom -> map, relocalization) and pose sources changing are ignored.
  const lastPoseRef = useRef(null);
  useEffect(() => {
    if (!connected || robotPose?.x == null || robotPose?.y == null) return;
    const frame = robotPose.frame_id || 'map';
    const prev = lastPoseRef.current;
    lastPoseRef.current = { x: robotPose.x, y: robotPose.y, frame };
    if (!prev || prev.frame !== frame) return;
    const step = Math.hypot(robotPose.x - prev.x, robotPose.y - prev.y);
    if (step > 0.002 && step < 1.0) setDistanceM((d) => d + step);
  }, [robotPose, connected]);

  useEffect(() => {
    const speed = Math.abs(Number(velocity?.linear || 0));
    if (connected && speed > peakSpeed) setPeakSpeed(speed);
  }, [velocity, connected, peakSpeed]);

  // Connection events
  const prevConnectedRef = useRef(connected);
  useEffect(() => {
    if (prevConnectedRef.current === connected) return;
    prevConnectedRef.current = connected;
    addLog(connected ? 'INFO' : 'WARN', 'ROSBRIDGE', connected ? 'Connected to the robot.' : 'Disconnected from the robot.');
  }, [connected]);

  // Mode / state machine events
  const prevStateRef = useRef(null);
  useEffect(() => {
    if (!connected || !systemState?.state) return;
    const state = systemState.state;
    const prev = prevStateRef.current;
    prevStateRef.current = state;
    if (prev === null || prev === state) return;
    if (state === 'EMERGENCY_STOP') {
      setEstopCount((n) => n + 1);
      addLog('ERROR', 'SAFETY', `Emergency stop activated (${ROBOT_SPEC.safety.estop}).`);
    } else if (state === 'ERROR') {
      addLog('ERROR', 'SYSTEM', `Mode ${prev} → ERROR${systemState.error_reason ? `: ${systemState.error_reason}` : ''}`);
    } else {
      addLog('INFO', 'SYSTEM', `Mode ${prev} → ${state}${systemState.active_map ? ` (map: ${systemState.active_map})` : ''}`);
    }
  }, [systemState, connected]);

  // Navigation goal events
  const prevNavRef = useRef(null);
  useEffect(() => {
    if (!connected || !navStatus?.status) return;
    const status = navStatus.status;
    const prev = prevNavRef.current;
    prevNavRef.current = status;
    if (prev === null || prev === status) return;
    const target = navStatus.target ? ` "${navStatus.target}"` : '';
    if (status === 'PLANNING') addLog('INFO', 'NAV2', `Goal sent${target}.`);
    else if (status === 'NAVIGATING' && prev === 'PLANNING') addLog('INFO', 'NAV2', `Nav2 accepted the goal${target}.`);
    else if (status === 'SUCCEEDED') {
      setGoals((g) => ({ ...g, succeeded: g.succeeded + 1 }));
      addLog('INFO', 'NAV2', `Goal reached (within ${ROBOT_SPEC.navigation.goalToleranceM} m tolerance).`);
    } else if (status === 'FAILED') {
      setGoals((g) => ({ ...g, failed: g.failed + 1 }));
      addLog('WARN', 'NAV2', 'Goal failed (no path or aborted by Nav2).');
    } else if (status === 'CANCELED') {
      setGoals((g) => ({ ...g, canceled: g.canceled + 1 }));
      addLog('INFO', 'NAV2', 'Goal canceled.');
    }
  }, [navStatus, connected]);

  // Health component changes (LiDAR, odometry, drive, TF, ...)
  const prevHealthRef = useRef({});
  useEffect(() => {
    if (!connected || !systemHealth?.components) return;
    const prev = prevHealthRef.current;
    const next = {};
    Object.entries(systemHealth.components).forEach(([name, comp]) => {
      const status = comp?.status;
      next[name] = status;
      if (prev[name] === undefined || prev[name] === status) return;
      if (status === 'ERROR') setHealthErrors((n) => n + 1);
      const level = status === 'ERROR' ? 'ERROR' : status === 'WARNING' ? 'WARN' : 'INFO';
      addLog(level, name.toUpperCase(), `${prev[name]} → ${status}${comp?.message ? `: ${comp.message}` : ''}`);
    });
    prevHealthRef.current = next;
  }, [systemHealth, connected]);

  const handleExportLogs = () => {
    const dataStr = "data:text/json;charset=utf-8," + encodeURIComponent(JSON.stringify(logs, null, 2));
    const downloadAnchor = document.createElement('a');
    downloadAnchor.setAttribute("href", dataStr);
    downloadAnchor.setAttribute("download", `optimus_amr_logs_${Date.now()}.json`);
    document.body.appendChild(downloadAnchor);
    downloadAnchor.click();
    downloadAnchor.remove();
  };

  const filteredLogs = logs.filter(l => {
    const matchesSearch = searchTerm === '' ||
      l.msg.toLowerCase().includes(searchTerm.toLowerCase()) ||
      l.category.toLowerCase().includes(searchTerm.toLowerCase());
    const matchesLevel = selectedLevel === 'ALL' || l.level === selectedLevel;
    return matchesSearch && matchesLevel;
  });

  const bat = batteryView(battery);
  const finishedGoals = goals.succeeded + goals.failed;
  const successPct = finishedGoals ? Math.round((goals.succeeded / finishedGoals) * 100) : null;
  const speedNow = Math.abs(Number(velocity?.linear || 0));
  const tile = 'clay-card p-3.5 hover:border-slate-700 transition-all';
  const label = 'text-[10px] text-slate-400 font-bold uppercase tracking-wider';
  const unit = 'text-xs text-slate-400 font-normal';

  return (
    <div className="flex flex-col gap-3 flex-1">
      {/* Session KPI cards (live, since the dashboard was opened) */}
      <div className="grid grid-cols-2 lg:grid-cols-6 gap-2.5 font-mono">
        <div className={tile}>
          <div className={label}>DISTANCE (SESSION)</div>
          <div className="text-xl font-black text-slate-50">
            {distanceM < 1000 ? distanceM.toFixed(1) : (distanceM / 1000).toFixed(2)}{' '}
            <span className={unit}>{distanceM < 1000 ? 'm' : 'km'}</span>
          </div>
          <div className="text-[10px] text-slate-400 font-semibold">From live pose</div>
        </div>
        <div className={tile}>
          <div className={label}>SESSION TIME</div>
          <div className="text-xl font-black text-slate-50">{fmtDuration(now - sessionStart)}</div>
          <div className={`text-[10px] font-bold ${connected ? 'text-emerald-400' : 'text-red-500'}`}>
            {connected ? '● Robot connected' : '● Robot offline'}
          </div>
        </div>
        <div className={tile}>
          <div className={label}>NAV2 GOALS</div>
          <div className={`text-xl font-black ${successPct === null ? 'text-slate-50' : successPct >= 90 ? 'text-emerald-400' : 'text-amber-400'}`}>
            {successPct === null ? '--' : successPct} <span className={unit}>%</span>
          </div>
          <div className="text-[10px] text-slate-400 font-semibold">
            {goals.succeeded}/{finishedGoals} reached{goals.canceled ? ` • ${goals.canceled} canceled` : ''}
          </div>
        </div>
        <div className={tile}>
          <div className={label}>SPEED</div>
          <div className="text-xl font-black text-slate-50">{speedNow.toFixed(2)} <span className={unit}>m/s</span></div>
          <div className="text-[10px] text-slate-400 font-semibold">
            Peak {peakSpeed.toFixed(2)} • limit {ROBOT_SPEC.drivetrain.maxLinearMps} m/s
          </div>
        </div>
        <div className={tile}>
          <div className={label}>BATTERY ({ROBOT_SPEC.power.nominalV} V)</div>
          <div className="text-xl font-black text-orange-400">{bat.pctText} <span className={unit}>%</span></div>
          <div className="text-[10px] text-amber-400 font-bold" title="No battery monitor is connected; amr_system_manager publishes a simulated value">
            Simulated • no battery monitor
          </div>
        </div>
        <div className={tile}>
          <div className={label}>SAFETY EVENTS</div>
          <div className={`text-xl font-black ${estopCount || healthErrors ? 'text-red-400' : 'text-emerald-400'}`}>
            {estopCount + healthErrors} <span className={unit}>events</span>
          </div>
          <div className="text-[10px] text-slate-400 font-semibold">{estopCount} e-stop • {healthErrors} health errors</div>
        </div>
      </div>

      {/* Main Rolling Event Logs & Diagnostics Terminal */}
      <div className="clay-card flex-1 flex flex-col overflow-hidden min-h-[500px]">
        {/* Terminal Header Bar */}
        <div className="panel-header py-2.5 px-4 bg-slate-950 border-b border-slate-800 text-white">
          <div className="flex items-center gap-2">
            <Terminal className="w-4 h-4 text-orange-500" />
            <span className="font-bold text-xs text-white tracking-wider font-mono">
              LIVE ROBOT EVENT LOG (THIS SESSION)
            </span>
          </div>

          <div className="flex items-center gap-2">
            {/* Auto Scroll Toggle Button */}
            <button
              onClick={() => setAutoScroll(!autoScroll)}
              className={`px-3 py-1 rounded-xl border text-xs font-mono font-bold flex items-center gap-1.5 transition-all cursor-pointer ${
                autoScroll
                  ? 'btn-clay-orange font-extrabold shadow-xs'
                  : 'bg-slate-800 border-slate-700 text-slate-300 hover:text-white hover:bg-slate-700'
              }`}
              title="Automatically scroll down as new event logs arrive"
            >
              <ArrowDownCircle className={`w-3.5 h-3.5 ${autoScroll ? 'animate-bounce' : ''}`} />
              <span>AUTO-SCROLL: {autoScroll ? 'ON' : 'OFF'}</span>
            </button>

            <button
              onClick={handleExportLogs}
              className="p-1.5 rounded-xl bg-slate-800 border border-slate-700 hover:bg-slate-700 text-slate-300 hover:text-white transition-colors cursor-pointer"
              title="Export logs as JSON file"
            >
              <Download className="w-3.5 h-3.5" />
            </button>

            <button
              onClick={() => setLogs([])}
              className="p-1.5 rounded-xl bg-slate-800 border border-slate-700 hover:bg-red-900/60 text-slate-400 hover:text-red-400 transition-colors cursor-pointer"
              title="Clear log buffer"
            >
              <Trash2 className="w-3.5 h-3.5" />
            </button>
          </div>
        </div>

        {/* Filters Bar */}
        <div className="p-3 bg-slate-900 border-b border-slate-800 flex flex-wrap items-center justify-between gap-2.5 text-xs font-mono">
          <div className="flex items-center gap-2 flex-1 max-w-sm">
            <div className="relative flex-1">
              <Search className="w-3.5 h-3.5 text-slate-400 absolute left-3 top-1/2 -translate-y-1/2" />
              <input
                type="text"
                value={searchTerm}
                onChange={(e) => setSearchTerm(e.target.value)}
                placeholder="Filter logs by keyword..."
                className="w-full bg-slate-950 border border-slate-700 rounded-xl pl-8.5 pr-3 py-1.5 text-xs text-white placeholder-slate-400 focus:outline-none focus:border-orange-500 focus:ring-1 focus:ring-orange-500"
              />
            </div>
          </div>

          <div className="flex items-center gap-1.5">
            <span className="text-[11px] text-slate-400 font-semibold">LEVEL:</span>
            {['ALL', 'INFO', 'WARN', 'ERROR'].map(lvl => (
              <button
                key={lvl}
                onClick={() => setSelectedLevel(lvl)}
                className={`px-3 py-1 rounded-xl text-[10px] font-bold transition-all cursor-pointer ${
                  selectedLevel === lvl
                    ? 'bg-gradient-to-r from-orange-500 to-orange-600 text-white font-black shadow-xs'
                    : 'bg-slate-950 border border-slate-800 text-slate-400 hover:text-white hover:bg-slate-800'
                }`}
              >
                {lvl}
              </button>
            ))}
          </div>
        </div>

        {/* Smooth Scrolling Log Stream Console */}
        <div
          ref={logContainerRef}
          className="flex-1 p-3.5 overflow-y-auto font-mono text-xs space-y-2 bg-slate-800/60 smooth-scroll select-text"
        >
          {filteredLogs.length === 0 ? (
            <div className="text-center py-16 text-slate-400 italic">
              No matching log records found in buffer.
            </div>
          ) : (
            filteredLogs.map(log => {
              const isError = log.level === 'ERROR';
              const isWarn = log.level === 'WARN';

              return (
                <div
                  key={log.id}
                  className={`p-2.5 rounded-xl border font-mono text-[11px] flex items-start gap-2.5 transition-all shadow-2xs ${
                    isError
                      ? 'bg-red-500/10 border-red-500/40 text-red-200'
                      : isWarn
                      ? 'bg-orange-500/10 border-orange-500/40 text-orange-200'
                      : 'bg-slate-900 border-slate-800 text-slate-100'
                  }`}
                >
                  <span className="text-slate-400 select-none shrink-0 font-semibold">{log.time}</span>
                  <span className={`px-1.5 py-0.2 rounded-md text-[9px] font-extrabold uppercase shrink-0 ${
                    isError ? 'bg-red-600 text-white' :
                    isWarn ? 'bg-orange-500 text-white' :
                    'bg-slate-700 text-slate-100 border border-slate-700'
                  }`}>
                    {log.level}
                  </span>
                  <span className="px-1.5 py-0.2 rounded-md bg-orange-500/15 text-orange-200 text-[9px] font-extrabold uppercase border border-orange-500/30 shrink-0">
                    {log.category}
                  </span>
                  <span className="flex-1 text-slate-100 leading-relaxed break-words">{log.msg}</span>
                </div>
              );
            })
          )}
        </div>

        {/* Footer info bar */}
        <div className="p-2.5 px-4 bg-slate-950 border-t border-slate-800 flex items-center justify-between text-[11px] font-mono text-slate-400">
          <div>
            Showing <strong className="text-white">{filteredLogs.length}</strong> of <strong className="text-white">{logs.length}</strong> events
          </div>
          <div className="flex items-center gap-2">
            <span className="h-2 w-2 rounded-full bg-emerald-400 animate-pulse"></span>
            <span className="text-slate-300 font-semibold">ROS 2 /rosout Stream Live</span>
          </div>
        </div>
      </div>
    </div>
  );
}
