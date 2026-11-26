import React, { useState, useEffect, useRef } from 'react';
import { 
  Play, Pause, Square, RotateCw, Plus, ArrowUp, ArrowDown, Trash2, 
  MapPin, CheckCircle2, Clock, Navigation, Flag, Check, RotateCcw,
  Sparkles, Layers, BatteryCharging, Box, Truck, Eye, Compass, ArrowRightLeft
} from 'lucide-react';
import MapViewer from '../MapViewer';
import WaypointScrollBar from '../WaypointScrollBar';

export default function MissionDispatcherTemplate({
  locations = [],
  robotPose,
  mapData,
  laserScan,
  globalPath,
  localPath,
  globalCostmap,
  localCostmap,
  navStatus,
  systemState,
  onSendGoal,
  onCancelNavigation,
  onSetInitialPose,
  onNavigateToLocation,
  onSaveLocationAtPose
}) {
  const [route, setRoute] = useState([]);

  const [isRunning, setIsRunning] = useState(false);
  const [isLooping, setIsLooping] = useState(false);
  const [currentStopIndex, setCurrentStopIndex] = useState(0);
  const [loopCount, setLoopCount] = useState(0);
  const [missionLog, setMissionLog] = useState([]);
  const [selectedLocationToAdd, setSelectedLocationToAdd] = useState('');

  // Pending "dwell at stop, then dispatch the next goal" timer.
  const dwellTimerRef = useRef(null);
  const clearDwell = () => {
    if (dwellTimerRef.current) {
      clearTimeout(dwellTimerRef.current);
      dwellTimerRef.current = null;
    }
  };
  useEffect(() => clearDwell, []);

  useEffect(() => {
    if (!isRunning || route.length === 0) return;
    const status = navStatus?.status;

    // Previously only SUCCEEDED was handled, so a failed or cancelled leg
    // left the mission "running" forever, waiting for a success that never
    // came.
    if (status === 'FAILED' || status === 'CANCELED') {
      clearDwell();
      setIsRunning(false);
      addLog(`Leg to stop [${currentStopIndex + 1}/${route.length}] ${status.toLowerCase()}. Mission halted.`);
      return;
    }

    if (status === 'SUCCEEDED') {
      const currentStop = route[currentStopIndex];
      const nextIdx = currentStopIndex + 1;
      // Honour the per-stop dwell time shown in the route list.
      const dwellMs = Math.max(0, Number(currentStop?.waitSeconds ?? 3)) * 1000;
      const dispatchAfterDwell = (stop) => {
        clearDwell();
        dwellTimerRef.current = setTimeout(() => {
          dwellTimerRef.current = null;
          onSendGoal?.(stop.x, stop.y, stop.yaw || 0.0);
        }, dwellMs);
      };

      addLog(`Reached stop [${currentStopIndex + 1}/${route.length}]: ${currentStop?.name || 'Waypoint'}`);

      if (nextIdx < route.length) {
        setCurrentStopIndex(nextIdx);
        const nextStop = route[nextIdx];
        addLog(`Navigating to stop [${nextIdx + 1}/${route.length}]: ${nextStop.name}`);
        dispatchAfterDwell(nextStop);
      } else {
        if (isLooping) {
          setLoopCount(c => c + 1);
          setCurrentStopIndex(0);
          const firstStop = route[0];
          addLog(`Loop #${loopCount + 1} finished. Restarting loop at ${firstStop.name}`);
          dispatchAfterDwell(firstStop);
        } else {
          setIsRunning(false);
          addLog('Full mission route completed successfully!');
        }
      }
    }
  }, [navStatus?.status]);

  const addLog = (msg) => {
    const timestamp = new Date().toLocaleTimeString();
    setMissionLog(prev => [{ time: timestamp, message: msg }, ...prev.slice(0, 49)]);
  };

  const handleStartMission = () => {
    if (route.length === 0) return;
    setIsRunning(true);
    setCurrentStopIndex(0);
    const target = route[0];
    addLog(`Mission started. Dispatching to stop 1: ${target.name}`);
    onSendGoal?.(target.x, target.y, target.yaw || 0.0);
  };

  const handlePauseMission = () => {
    clearDwell();
    setIsRunning(false);
    addLog('Mission paused by operator.');
  };

  const handleAbortMission = () => {
    clearDwell();
    setIsRunning(false);
    setCurrentStopIndex(0);
    // Previously this only logged "AMR halted" while the active Nav2 goal
    // kept driving the robot.
    onCancelNavigation?.();
    addLog('Mission aborted. AMR halted.');
  };

  const handleAddToRoute = (loc) => {
    if (!loc) return;
    setRoute(prev => [
      ...prev,
      { ...loc, routeId: `route-${Date.now()}-${Math.random()}`, waitSeconds: 3 }
    ]);
    addLog(`Added ${loc.name} to route sequence.`);
  };

  const handleRemoveFromRoute = (index) => {
    setRoute(prev => prev.filter((_, i) => i !== index));
    if (currentStopIndex >= index && currentStopIndex > 0) {
      setCurrentStopIndex(currentStopIndex - 1);
    }
  };

  const handleMoveStop = (index, direction) => {
    const targetIdx = index + direction;
    if (targetIdx < 0 || targetIdx >= route.length) return;
    setRoute(prev => {
      const copy = [...prev];
      const item = copy.splice(index, 1)[0];
      copy.splice(targetIdx, 0, item);
      return copy;
    });
  };

  const handleClearRoute = () => {
    if (isRunning) return;
    setRoute([]);
    addLog('Cleared all stops from route.');
  };

  const handleReverseRoute = () => {
    if (isRunning || route.length <= 1) return;
    setRoute(prev => [...prev].reverse());
    addLog('Reversed mission route sequence.');
  };

  const handleUpdateDwellTime = (index, delta) => {
    setRoute(prev => prev.map((s, i) => {
      if (i === index) {
        const nextTime = Math.max(0, Math.min(60, (s.waitSeconds || 3) + delta));
        return { ...s, waitSeconds: nextTime };
      }
      return s;
    }));
  };

  const getTypeIcon = (type) => {
    switch (type) {
      case 'charging':
        return <BatteryCharging className="w-3.5 h-3.5 text-emerald-400" />;
      case 'loading':
        return <Truck className="w-3.5 h-3.5 text-purple-400" />;
      case 'inspection':
        return <Eye className="w-3.5 h-3.5 text-blue-400" />;
      default:
        return <Box className="w-3.5 h-3.5 text-orange-400" />;
    }
  };

  const currentStop = route[currentStopIndex];
  const progressPercent = route.length > 0 ? Math.round((currentStopIndex / route.length) * 100) : 0;

  return (
    <div className="flex flex-col gap-3 flex-1 font-mono">
      {/* 1. Quick Waypoint Dispatch Strip */}
      <WaypointScrollBar
        locations={locations}
        robotPose={robotPose}
        onNavigateToLocation={onNavigateToLocation}
      />

      {/* 2. Main Grid: Left Map + Right Mission Sequencer */}
      <div className="grid grid-cols-1 lg:grid-cols-12 gap-3.5 items-stretch min-h-[620px] flex-1">
        {/* Left: Interactive 2D Map (7 cols) */}
        <div className="lg:col-span-7 flex flex-col h-full min-h-[500px]">
          <MapViewer
            mapData={mapData}
            robotPose={robotPose}
            laserScan={laserScan}
            globalPath={globalPath}
            localPath={localPath}
            globalCostmap={globalCostmap}
            localCostmap={localCostmap}
            locations={locations}
            onSendGoal={onSendGoal}
            onSetInitialPose={onSetInitialPose}
            onSaveLocationAtPose={onSaveLocationAtPose}
          />
        </div>

        {/* Right: Mission Dispatcher Panel (5 cols) */}
        <div className="lg:col-span-5 flex flex-col clay-card overflow-hidden">
          {/* Header */}
          <div className="panel-header py-2.5 px-4 bg-slate-950 border-b border-slate-800 text-white flex items-center justify-between">
            <div className="flex items-center gap-2">
              <div className="p-1.5 rounded-xl bg-orange-500/20 text-orange-400 border border-orange-500/30">
                <Navigation className="w-4 h-4" />
              </div>
              <span className="font-bold text-xs text-white tracking-wider">
                POI ROUTE DISPATCHER
              </span>
            </div>

            <div className="flex items-center gap-2 text-[10px]">
              <span className={`px-2.5 py-0.5 rounded-full border font-bold ${
                isRunning 
                  ? 'bg-orange-500 text-white border-orange-400 animate-pulse' 
                  : 'bg-slate-800 border-slate-700 text-orange-400'
              }`}>
                {isRunning ? 'DISPATCHING' : 'IDLE'}
              </span>
              {isLooping && (
                <span className="px-2 py-0.5 rounded-full bg-purple-950/80 border border-purple-500/50 text-purple-300 font-bold">
                  LOOP x{loopCount}
                </span>
              )}
            </div>
          </div>

          {/* Mission Controls Banner */}
          <div className="p-3.5 bg-slate-800/40 border-b border-slate-800/80 flex flex-wrap items-center justify-between gap-2">
            <div className="flex items-center gap-1.5">
              {!isRunning ? (
                <button
                  onClick={handleStartMission}
                  disabled={route.length === 0}
                  className="btn-clay-orange text-xs px-3.5 py-2 flex items-center gap-1.5 shadow-sm font-extrabold cursor-pointer disabled:opacity-40"
                >
                  <Play className="w-3.5 h-3.5 fill-white" />
                  <span>START ROUTE</span>
                </button>
              ) : (
                <button
                  onClick={handlePauseMission}
                  className="btn-clay-white text-xs px-3.5 py-2 flex items-center gap-1.5 text-amber-300 border-amber-500/40 cursor-pointer font-bold"
                >
                  <Pause className="w-3.5 h-3.5" />
                  <span>PAUSE</span>
                </button>
              )}

              <button
                onClick={handleAbortMission}
                disabled={!isRunning}
                className="btn-clay-white text-xs px-3 py-2 flex items-center gap-1 text-red-400 border-red-500/30 hover:bg-red-500/10 cursor-pointer font-bold disabled:opacity-40"
              >
                <Square className="w-3 h-3 fill-red-600" />
                <span>ABORT</span>
              </button>
            </div>

            <div className="flex items-center gap-1.5">
              {/* Loop Patrol Toggle */}
              <button
                onClick={() => setIsLooping(!isLooping)}
                className={`px-2.5 py-1.5 rounded-xl border text-xs font-bold flex items-center gap-1 transition-all cursor-pointer ${
                  isLooping
                    ? 'btn-clay-orange text-white shadow-2xs font-extrabold'
                    : 'btn-clay-white text-slate-200 hover:text-slate-50'
                }`}
                title="Continuous patrol loop"
              >
                <RotateCw className={`w-3 h-3 ${isLooping ? 'animate-spin-slow' : ''}`} />
                <span>LOOP</span>
              </button>

              {/* Reverse Sequence */}
              <button
                onClick={handleReverseRoute}
                disabled={isRunning || route.length <= 1}
                className="btn-clay-white p-2 rounded-xl text-slate-300 hover:text-slate-50 transition-colors cursor-pointer disabled:opacity-40"
                title="Reverse stop order"
              >
                <ArrowRightLeft className="w-3.5 h-3.5 text-slate-300" />
              </button>

              {/* Clear Route */}
              <button
                onClick={handleClearRoute}
                disabled={isRunning || route.length === 0}
                className="btn-clay-white p-2 rounded-xl text-slate-400 hover:text-red-400 hover:border-red-500/30 transition-colors cursor-pointer disabled:opacity-40"
                title="Clear all stops"
              >
                <Trash2 className="w-3.5 h-3.5" />
              </button>
            </div>
          </div>

          {/* Active Mission Progress Bar */}
          {isRunning && (
            <div className="p-3 bg-orange-500/10 border-b border-orange-500/90 shadow-inner">
              <div className="flex items-center justify-between text-xs mb-1.5">
                <span className="text-orange-200 font-bold flex items-center gap-1.5">
                  <Flag className="w-3.5 h-3.5 text-orange-400 animate-bounce" />
                  Stop #{currentStopIndex + 1}: <strong className="text-slate-50">{currentStop?.name}</strong>
                </span>
                <span className="text-orange-300 font-bold">
                  {currentStopIndex + 1} / {route.length} ({progressPercent}%)
                </span>
              </div>
              <div className="h-2 w-full bg-slate-700 rounded-full overflow-hidden p-0.5">
                <div
                  className="h-full bg-gradient-to-r from-orange-500 to-orange-600 transition-all duration-300 rounded-full shadow-xs"
                  style={{ width: `${Math.max(5, progressPercent)}%` }}
                />
              </div>
            </div>
          )}

          {/* Route Sequencer List */}
          <div className="flex-1 p-3.5 overflow-y-auto space-y-2 max-h-[340px] smooth-scroll">
            {route.length === 0 ? (
              <div className="text-center py-10 px-4 rounded-2xl border border-dashed border-slate-700 bg-slate-800/40 text-slate-400 text-xs space-y-2">
                <MapPin className="w-7 h-7 text-orange-400 mx-auto" />
                <p className="font-bold text-slate-100">Mission route is empty</p>
                <p className="text-[11px] text-slate-400">
                  Tap any saved POI chip below to build an automated delivery or patrol route for Optimus.
                </p>
              </div>
            ) : (
              route.map((stop, idx) => {
                const isActive = isRunning && currentStopIndex === idx;
                const isCompleted = isRunning && currentStopIndex > idx;

                return (
                  <div
                    key={stop.routeId || idx}
                    className={`p-3 rounded-2xl border transition-all flex items-center justify-between gap-3 shadow-2xs ${
                      isActive
                        ? 'bg-orange-500/10 border-orange-400 text-slate-50 ring-2 ring-orange-500/80 shadow-xs'
                        : isCompleted
                        ? 'bg-emerald-500/70 border-emerald-500/40 text-slate-200'
                        : 'clay-inset bg-slate-800/40 border-slate-800/90 hover:border-slate-700 text-slate-100'
                    }`}
                  >
                    <div className="flex items-center gap-2.5 min-w-0">
                      <div className={`h-7 w-7 rounded-xl flex items-center justify-center font-bold text-xs shrink-0 shadow-2xs ${
                        isActive
                          ? 'bg-gradient-to-r from-orange-500 to-orange-600 text-white font-black'
                          : isCompleted
                          ? 'bg-emerald-500/15 text-emerald-200 border border-emerald-500/40'
                          : 'bg-slate-900 text-slate-200 border border-slate-800'
                      }`}>
                        {isCompleted ? <Check className="w-4 h-4 text-emerald-300" /> : idx + 1}
                      </div>

                      <div className="p-1 rounded-lg bg-slate-900 border border-slate-800 shrink-0 shadow-2xs">
                        {getTypeIcon(stop.type)}
                      </div>

                      <div className="min-w-0">
                        <div className="text-xs font-bold truncate flex items-center gap-1.5">
                          <span className="truncate">{stop.name}</span>
                          <span className="text-[9px] font-semibold uppercase px-1.5 py-0.2 rounded-md bg-slate-700/80 text-slate-300">
                            {stop.type || 'shelf'}
                          </span>
                        </div>
                        <div className="text-[10px] text-slate-400 flex items-center gap-1.5">
                          <span>({Number(stop.x).toFixed(1)}, {Number(stop.y).toFixed(1)})</span>
                          <span>• Dwell: {stop.waitSeconds || 3}s</span>
                        </div>
                      </div>
                    </div>

                    <div className="flex items-center gap-1 shrink-0">
                      {/* Dwell time adjustment */}
                      <button
                        onClick={() => handleUpdateDwellTime(idx, 1)}
                        className="px-1.5 py-1 rounded-lg btn-clay-white text-[9px] font-bold text-slate-300"
                        title="Increase dwell time (+1s)"
                      >
                        +{stop.waitSeconds || 3}s
                      </button>

                      <button
                        onClick={() => handleMoveStop(idx, -1)}
                        disabled={idx === 0}
                        className="p-1.5 rounded-lg btn-clay-white text-slate-200 disabled:opacity-30 disabled:cursor-not-allowed cursor-pointer"
                        title="Move Up"
                      >
                        <ArrowUp className="w-3 h-3" />
                      </button>
                      <button
                        onClick={() => handleMoveStop(idx, 1)}
                        disabled={idx === route.length - 1}
                        className="p-1.5 rounded-lg btn-clay-white text-slate-200 disabled:opacity-30 disabled:cursor-not-allowed cursor-pointer"
                        title="Move Down"
                      >
                        <ArrowDown className="w-3 h-3" />
                      </button>
                      <button
                        onClick={() => handleRemoveFromRoute(idx)}
                        className="p-1.5 rounded-lg btn-clay-white text-slate-400 hover:text-red-400 hover:border-red-500/30 cursor-pointer"
                        title="Remove from Route"
                      >
                        <Trash2 className="w-3 h-3" />
                      </button>
                    </div>
                  </div>
                );
              })
            )}
          </div>

          {/* Quick-Tap POI Palette (1-Click Waypoint Adding) */}
          <div className="p-3 bg-slate-800/40 border-t border-slate-800/80 flex flex-col gap-2">
            <div className="text-[10px] text-slate-300 font-bold uppercase flex items-center justify-between">
              <span className="flex items-center gap-1 text-slate-100">
                <MapPin className="w-3 h-3 text-orange-500" />
                TAP TO ADD WAYPOINT TO ROUTE:
              </span>
              <span className="text-[9px] text-slate-400 font-normal">{locations.length} POIs AVAILABLE</span>
            </div>

            {/* Quick POI Chips */}
            <div className="flex flex-wrap gap-1.5 max-h-24 overflow-y-auto no-scrollbar py-0.5">
              {locations.length === 0 ? (
                <div className="text-[11px] text-slate-400 italic">No POIs saved yet.</div>
              ) : (
                locations.map(loc => (
                  <button
                    key={loc.id || loc.name}
                    onClick={() => handleAddToRoute(loc)}
                    className="btn-clay-white text-[10px] py-1 px-2.5 rounded-xl flex items-center gap-1.5 cursor-pointer hover:border-orange-400 hover:text-orange-400 transition-all font-bold group shadow-2xs"
                    title={`Add ${loc.name} to route`}
                  >
                    <Plus className="w-3 h-3 text-orange-500 group-hover:rotate-90 transition-transform" />
                    <span className="truncate max-w-[120px]">{loc.name}</span>
                    <span className="text-[8px] uppercase px-1 rounded bg-slate-700/80 text-slate-300">{loc.type || 'shelf'}</span>
                  </button>
                ))
              )}
            </div>
          </div>

          {/* Mission Activity Log */}
          <div className="p-3 bg-slate-800/70 border-t border-slate-800/80">
            <div className="text-[10px] text-slate-300 font-bold uppercase mb-1.5 flex items-center justify-between">
              <span className="flex items-center gap-1">
                <Clock className="w-3 h-3 text-slate-400" />
                Mission Activity Log:
              </span>
              <span className="text-[9px] text-slate-400 font-normal">Real-time</span>
            </div>
            <div className="h-20 overflow-y-auto text-[10px] space-y-1 bg-slate-900 p-2.5 rounded-xl border border-slate-800 smooth-scroll shadow-inner">
              {missionLog.length === 0 ? (
                <div className="text-slate-400 italic">No mission events yet. Click "Start Route" to begin execution.</div>
              ) : (
                missionLog.map((entry, i) => (
                  <div key={i} className="flex items-start gap-1.5 text-slate-200">
                    <span className="text-orange-400 font-bold">[{entry.time}]</span>
                    <span>{entry.message}</span>
                  </div>
                ))
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
