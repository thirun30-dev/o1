import React from 'react';
import { Compass, Gauge, BatteryCharging, Navigation, Activity } from 'lucide-react';
import { batteryView } from '../utils/battery';
import { ROBOT_SPEC } from '../config/robotSpec';

export default function RobotStatus({ robotPose, velocity, battery, navStatus }) {
  const x = Number(robotPose?.x || 0).toFixed(3);
  const y = Number(robotPose?.y || 0).toFixed(3);
  const rawYaw = robotPose?.yaw || 0;
  const yawDeg = Number(robotPose?.yaw_deg ?? (rawYaw * 180 / Math.PI)).toFixed(1);
  const yawRad = Number(rawYaw).toFixed(2);
  // Signed: negative while reversing.
  const linVel = Number(velocity?.linear || 0).toFixed(2);
  const angVel = Number(velocity?.angular || 0).toFixed(2);
  const bat = batteryView(battery);
  const batPct = bat.pctText;
  const batVolt = bat.voltText;

  return (
    <div className="clay-card flex flex-col justify-between h-full overflow-hidden">
      {/* Header */}
      <div className="panel-header py-2.5 px-4 bg-slate-950 border-b border-slate-800 text-white">
        <span className="flex items-center gap-2 font-bold text-white text-xs">
          <Activity className="w-4 h-4 text-orange-500" />
          OPTIMUS TELEMETRY & KINEMATICS
        </span>
        <span className="text-[10px] font-mono px-2 py-0.5 rounded-full bg-slate-800 border border-slate-700 text-orange-400 font-bold">
          {ROBOT_SPEC.driver.model} • RS485
        </span>
      </div>

      <div className="p-3.5 grid grid-cols-2 sm:grid-cols-4 gap-2.5">
        {/* X Position */}
        <div className="clay-inset p-3 flex flex-col justify-between transition-all hover:border-slate-700">
          <div className="text-[9px] font-mono text-slate-400 font-bold uppercase tracking-wider mb-1">X POSITION</div>
          <div className="text-xl font-black font-mono text-slate-50 flex items-baseline gap-1">
            {x} <span className="text-[11px] font-medium text-slate-400">m</span>
          </div>
          <div className="text-[9px] font-mono text-slate-400 mt-0.5">Map Origin Frame</div>
        </div>

        {/* Y Position */}
        <div className="clay-inset p-3 flex flex-col justify-between transition-all hover:border-slate-700">
          <div className="text-[9px] font-mono text-slate-400 font-bold uppercase tracking-wider mb-1">Y POSITION</div>
          <div className="text-xl font-black font-mono text-slate-50 flex items-baseline gap-1">
            {y} <span className="text-[11px] font-medium text-slate-400">m</span>
          </div>
          <div className="text-[9px] font-mono text-slate-400 mt-0.5">Map Origin Frame</div>
        </div>

        {/* Heading Yaw + Precision Compass Dial */}
        <div className="clay-inset p-3 flex items-center justify-between transition-all hover:border-slate-700">
          <div>
            <div className="text-[9px] font-mono text-slate-400 font-bold uppercase tracking-wider mb-1">HEADING</div>
            <div className="text-xl font-black font-mono text-orange-400">
              {yawDeg}°
            </div>
            <div className="text-[9px] font-mono text-slate-400 mt-0.5">{yawRad} rad</div>
          </div>
          {/* Compass Dial */}
          <div className="relative h-10 w-10 rounded-full border border-slate-700 bg-slate-900 flex items-center justify-center shrink-0 shadow-inner">
            <div
              className="w-1 h-7 bg-gradient-to-t from-transparent via-orange-500 to-orange-600 rounded-full transition-transform duration-100 shadow-xs"
              style={{ transform: `rotate(${yawDeg}deg)` }}
            />
            <div className="absolute w-2 h-2 rounded-full bg-slate-900 border border-white" />
          </div>
        </div>

        {/* Speed */}
        <div className="clay-inset p-3 flex items-center justify-between transition-all hover:border-slate-700">
          <div>
            <div className="text-[9px] font-mono text-slate-400 font-bold uppercase tracking-wider mb-1">LINEAR VEL</div>
            <div className="text-xl font-black font-mono text-emerald-400 flex items-baseline gap-1">
              {linVel} <span className="text-[11px] font-medium text-slate-400">m/s</span>
            </div>
            <div className="text-[9px] font-mono text-slate-400 mt-0.5">ω: {angVel} rad/s</div>
          </div>
          <div className="p-2 rounded-xl bg-emerald-500/10 text-emerald-400 border border-emerald-500/30">
            <Gauge className="w-5 h-5" />
          </div>
        </div>
      </div>

      {/* Battery & Nav2 Status Bar */}
      <div className="px-3.5 pb-3.5 grid grid-cols-1 sm:grid-cols-2 gap-2.5">
        {/* Battery Health */}
        <div className="clay-inset p-3 flex items-center justify-between gap-3">
          <div className="flex items-center gap-2.5">
            <div className="p-2 rounded-xl bg-slate-900 text-orange-400 border border-slate-800 shadow-xs">
              <BatteryCharging className="w-4 h-4" />
            </div>
            <div>
              <div className="text-[11px] font-mono text-slate-50 font-bold">{ROBOT_SPEC.power.nominalV} V SUPPLY</div>
              <div className="text-[10px] font-mono text-slate-400" title="No battery monitor is connected; amr_system_manager publishes a simulated value">{batVolt} V • simulated (no battery monitor)</div>
            </div>
          </div>
          <div className="w-28">
            <div className="flex justify-between text-[10px] font-mono font-bold mb-1">
              <span className={batPct > 20 ? 'text-orange-400' : 'text-red-400'}>{batPct}%</span>
              <span className="text-slate-400">NOMINAL</span>
            </div>
            <div className="h-2 w-full bg-slate-700 rounded-full overflow-hidden p-0.5">
              <div
                className={`h-full transition-all duration-500 rounded-full ${
                  batPct > 25 ? 'bg-gradient-to-r from-orange-500 to-amber-500' : 'bg-red-500'
                }`}
                style={{ width: `${batPct}%` }}
              />
            </div>
          </div>
        </div>

        {/* Navigation Feedback */}
        <div className="clay-inset p-3 flex items-center justify-between gap-2.5">
          <div className="flex items-center gap-2.5 min-w-0">
            <div className={`p-2 rounded-xl border transition-all ${
              navStatus?.status === 'NAVIGATING' ? 'bg-orange-500 text-white border-orange-600 animate-pulse shadow-xs' :
              navStatus?.status === 'SUCCEEDED' ? 'bg-emerald-500/15 border-emerald-500/40 text-emerald-200' :
              'bg-slate-900 border-slate-800 text-orange-400 shadow-xs'
            }`}>
              <Navigation className="w-4 h-4" />
            </div>
            <div className="min-w-0">
              <div className="text-[11px] font-mono text-slate-50 font-bold flex items-center gap-1.5">
                NAV2: <span className="text-orange-400 uppercase">{navStatus?.status || 'IDLE'}</span>
              </div>
              <div className="text-[10px] font-mono text-slate-400 truncate">
                {navStatus?.target ? `Target: ${navStatus.target}` : 'Ready for goal dispatch'}
              </div>
            </div>
          </div>
          <div className="text-right font-mono shrink-0 pl-1">
            <div className="text-xs font-black text-slate-50">
              {navStatus?.distance_remaining != null ? `${navStatus.distance_remaining}m` : '--'}
            </div>
            <div className="text-[8px] text-slate-400 uppercase font-semibold">REMAINING</div>
          </div>
        </div>
      </div>
    </div>
  );
}
