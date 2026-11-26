import React, { useState } from 'react';
import {
  Activity, CheckCircle2, AlertTriangle, XCircle, Radio, Disc, Boxes,
  Compass, Layers, Map, Navigation
} from 'lucide-react';
import { batteryView } from '../utils/battery';

const STATUS_STYLE = {
  OK: { icon: CheckCircle2, cls: 'text-emerald-400' },
  WARNING: { icon: AlertTriangle, cls: 'text-amber-500' },
  ERROR: { icon: XCircle, cls: 'text-red-400' },
};

function StatusIcon({ status }) {
  const s = STATUS_STYLE[status] || STATUS_STYLE.WARNING;
  const Icon = s.icon;
  return <Icon className={`w-3.5 h-3.5 shrink-0 ${s.cls}`} />;
}

export default function SensorStatus({
  systemHealth,
  systemState,
  battery,
  connected = true,
}) {
  const [activeFilter, setActiveFilter] = useState('ALL'); // 'ALL' | 'HARDWARE' | 'NAV'

  const comps = systemHealth?.components || {};
  const overall = systemHealth?.overall || 'WARNING';
  const isHealthy = overall === 'OK';
  const bat = batteryView(battery);

  const nodes = [
    { key: 'lidar', label: 'RPLiDAR A1M8', topic: '/scan', category: 'HARDWARE', icon: Radio, detail: '360° • top of body, front end' },
    { key: 'controller', label: 'ZLAC8015D Drive', topic: '/joint_states', category: 'HARDWARE', icon: Boxes, detail: 'RS485 Modbus RTU • 2 × ZLLG50ASM200 hub motors' },
    { key: 'odometry', label: 'Wheel Odometry', topic: '/odom', category: 'HARDWARE', icon: Disc, detail: 'optimus_drivetrain_node' },
    { key: 'tf', label: 'TF2 Frames', topic: '/tf & /tf_static', category: 'HARDWARE', icon: Compass, detail: 'map ➔ odom ➔ base_link ➔ lidar_link' },
    { key: 'nav2', label: 'Nav2 Controller', topic: '/plan & /cmd_vel', category: 'NAV', icon: Navigation, detail: 'Costmaps & path follower' },
    { key: 'amcl', label: 'AMCL Localization', topic: '/amcl_pose', category: 'NAV', icon: Compass, detail: '2D Monte Carlo localization' },
    { key: 'map_server', label: 'Map Server', topic: '/map', category: 'NAV', icon: Map, detail: `${systemState?.active_map || 'no map'} active` },
    { key: 'slam_toolbox', label: 'SLAM Toolbox', topic: '/map', category: 'NAV', icon: Layers, detail: 'Mapping mode' },
  ];

  const visibleNodes = nodes.filter(n => activeFilter === 'ALL' || n.category === activeFilter);
  const hwCount = nodes.filter(n => n.category === 'HARDWARE').length;
  const navCount = nodes.filter(n => n.category === 'NAV').length;

  return (
    <div className="flex flex-col gap-3 font-mono">
      {/* 1. Health banner */}
      <div className="clay-card p-3.5 rounded-2xl border border-slate-800/90 flex flex-col gap-2.5">
        <div className="flex items-center gap-2">
          <div className={`p-2 rounded-xl shrink-0 ${
            isHealthy ? 'bg-emerald-500/10 text-emerald-400 border border-emerald-500/30' : 'bg-red-500/10 text-red-400 border border-red-500/30'
          }`}>
            <Activity className="w-4 h-4" />
          </div>
          <div>
            <div className="text-xs font-black text-slate-50 flex items-center gap-1.5">
              <span>OPTIMUS SYSTEM HEALTH</span>
              <span className={`text-[9px] px-2 py-0.5 rounded-full font-bold border ${
                isHealthy
                  ? 'bg-emerald-500/15 text-emerald-200 border-emerald-500/40'
                  : overall === 'ERROR'
                  ? 'bg-red-500/15 text-red-200 border-red-500/40'
                  : 'bg-amber-500/15 text-amber-200 border-amber-500/40'
              }`}>
                {isHealthy ? 'OPTIMAL' : overall === 'ERROR' ? 'FAULT' : 'ATTENTION'}
              </span>
            </div>
            <div className="text-[10px] text-slate-400">
              ROS 2 Humble • rosbridge :9090 {connected ? '' : '• OFFLINE (demo)'}
            </div>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-2 text-center text-[10px] pt-1 border-t border-slate-800/80">
          <div className="bg-slate-800/40 p-2 rounded-xl border border-slate-800/80">
            <span className="text-slate-400 block text-[9px]">DRIVE RS485</span>
            <span className="font-extrabold text-slate-50 text-xs">ZLAC8015D</span>
            <span className={`block text-[8px] font-bold ${comps.controller?.status === 'OK' ? 'text-emerald-400' : 'text-amber-400'}`}>
              ● {comps.controller?.status === 'OK' ? 'JOINT STATES' : 'WAITING'}
            </span>
          </div>

          <div className="bg-slate-800/40 p-2 rounded-xl border border-slate-800/80">
            <span className="text-slate-400 block text-[9px]">LIDAR</span>
            <span className="font-extrabold text-slate-50 text-xs">A1M8</span>
            <span className={`block text-[8px] font-bold ${comps.lidar?.status === 'OK' ? 'text-emerald-400' : 'text-amber-400'}`}>
              ● {comps.lidar?.status === 'OK' ? 'SCANNING' : 'WAITING'}
            </span>
          </div>

          <div className="bg-slate-800/40 p-2 rounded-xl border border-slate-800/80" title="No battery monitor fitted yet - simulated value">
            <span className="text-slate-400 block text-[9px]">BATTERY (SIM)</span>
            <span className="font-extrabold text-slate-50 text-xs">{bat.voltText} V</span>
            <span className="block text-[8px] font-bold text-slate-400">
              ● {bat.pct === null ? 'NO DATA' : `${bat.pct}% • NO SENSOR`}
            </span>
          </div>
        </div>
      </div>

      {/* 2. Filter */}
      <div className="flex items-center justify-between gap-1.5 pt-0.5">
        <span className="text-[10px] text-slate-400 font-bold uppercase">
          TOPICS & NODES ({visibleNodes.length})
        </span>
        <div className="flex items-center gap-1">
          {[
            { id: 'ALL', label: 'All' },
            { id: 'HARDWARE', label: `Hardware (${hwCount})` },
            { id: 'NAV', label: `Nav2 (${navCount})` },
          ].map(f => (
            <button
              key={f.id}
              onClick={() => setActiveFilter(f.id)}
              className={`px-2 py-0.5 rounded-lg text-[9px] font-bold transition-all cursor-pointer ${
                activeFilter === f.id
                  ? 'bg-slate-900 text-white font-extrabold shadow-2xs'
                  : 'bg-slate-800/70 text-slate-300 hover:text-slate-50'
              }`}
            >
              {f.label}
            </button>
          ))}
        </div>
      </div>

      {/* 3. Live component list from /amr/system_health */}
      <div className="space-y-2 overflow-y-auto max-h-[420px] smooth-scroll pr-1">
        {visibleNodes.map((item) => {
          const Icon = item.icon;
          const comp = comps[item.key] || {};
          return (
            <div
              key={item.key}
              className="clay-card p-2.5 rounded-xl border border-slate-800/90 hover:border-slate-700 transition-all flex items-center justify-between gap-2.5 shadow-2xs"
            >
              <div className="flex items-center gap-2.5 min-w-0">
                <div className="p-2 rounded-xl bg-slate-900 border border-slate-800 shadow-2xs shrink-0">
                  <Icon className="w-3.5 h-3.5 text-orange-400" />
                </div>
                <div className="min-w-0">
                  <div className="text-xs font-bold text-slate-50 truncate flex items-center gap-1.5">
                    <span className="truncate">{item.label}</span>
                    <span className="text-[9px] text-slate-400 font-mono font-normal">{item.topic}</span>
                  </div>
                  <div className="text-[10px] text-slate-400 truncate mt-0.5">
                    {comp.message || item.detail}
                  </div>
                </div>
              </div>
              <div className="flex items-center gap-1.5 shrink-0 text-right">
                <span className="text-[9px] px-1.5 py-0.5 rounded-md bg-slate-800/70 text-slate-200 font-bold border border-slate-800">
                  {comp.status || '—'}
                </span>
                <StatusIcon status={comp.status} />
              </div>
            </div>
          );
        })}
      </div>
    </div>
  );
}
