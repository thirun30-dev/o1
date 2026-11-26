import React, { useState, useMemo } from 'react';
import { 
  MapPin, Navigation, Trash2, Plus, BatteryCharging, Box, Truck, 
  Search, Check, Copy, Compass, Filter, ArrowUpDown, Eye, CheckCircle2,
  Sparkles, Layers
} from 'lucide-react';

export default function LocationManager({ 
  locations = [], 
  robotPose, 
  activeMap, 
  onSaveLocation, 
  onDeleteLocation, 
  onNavigateToLocation,
  onClearAllLocations
}) {
  const [modalOpen, setModalOpen] = useState(false);
  const [searchTerm, setSearchTerm] = useState('');
  const [selectedCategory, setSelectedCategory] = useState('ALL');
  const [sortBy, setSortBy] = useState('DISTANCE'); // 'DISTANCE' | 'NAME'
  const [copiedId, setCopiedId] = useState(null);

  // Form states
  const [locName, setLocName] = useState('');
  const [locType, setLocType] = useState('shelf');
  const [customX, setCustomX] = useState('');
  const [customY, setCustomY] = useState('');
  const [customYaw, setCustomYaw] = useState('0.00');
  const [useCurrentPose, setUseCurrentPose] = useState(true);

  const handleOpenModal = (presetX = null, presetY = null) => {
    if (presetX !== null && presetY !== null) {
      setCustomX(presetX.toFixed(2));
      setCustomY(presetY.toFixed(2));
      setCustomYaw('0.00');
      setUseCurrentPose(false);
    } else {
      setCustomX(Number(robotPose?.x || 0).toFixed(2));
      setCustomY(Number(robotPose?.y || 0).toFixed(2));
      setCustomYaw(Number(robotPose?.yaw || 0).toFixed(2));
      setUseCurrentPose(true);
    }
    setLocName(`Waypoint ${locations.length + 1}`);
    setModalOpen(true);
  };

  const handleUseCurrentPose = () => {
    setCustomX(Number(robotPose?.x || 0).toFixed(2));
    setCustomY(Number(robotPose?.y || 0).toFixed(2));
    setCustomYaw(Number(robotPose?.yaw || 0).toFixed(2));
    setUseCurrentPose(true);
  };

  const handleSave = (e) => {
    e.preventDefault();
    if (!locName.trim()) return;

    onSaveLocation({
      action: 'save_location',
      name: locName.trim(),
      type: locType,
      use_current_pose: useCurrentPose,
      x: parseFloat(customX) || 0.0,
      y: parseFloat(customY) || 0.0,
      yaw: parseFloat(customYaw) || 0.0,
      map: activeMap || 'warehouse_main'
    });
    setModalOpen(false);
    setLocName('');
  };

  const getDistanceToRobot = (locX, locY) => {
    if (!robotPose || robotPose.x == null) return null;
    const d = Math.hypot(locX - robotPose.x, locY - robotPose.y);
    return d;
  };

  const getRelativeBearing = (locX, locY) => {
    if (!robotPose || robotPose.x == null) return '';
    const dx = locX - robotPose.x;
    const dy = locY - robotPose.y;
    const angle = Math.atan2(dy, dx) * (180 / Math.PI);
    const normalized = (angle + 360) % 360;
    
    if (normalized >= 337.5 || normalized < 22.5) return 'E';
    if (normalized >= 22.5 && normalized < 67.5) return 'NE';
    if (normalized >= 67.5 && normalized < 112.5) return 'N';
    if (normalized >= 112.5 && normalized < 157.5) return 'NW';
    if (normalized >= 157.5 && normalized < 202.5) return 'W';
    if (normalized >= 202.5 && normalized < 247.5) return 'SW';
    if (normalized >= 247.5 && normalized < 292.5) return 'S';
    return 'SE';
  };

  const handleCopyCoords = (loc, e) => {
    e.stopPropagation();
    const text = `X: ${Number(loc.x).toFixed(2)}, Y: ${Number(loc.y).toFixed(2)}, Yaw: ${Number(loc.yaw || 0).toFixed(2)}`;
    navigator.clipboard?.writeText?.(text);
    setCopiedId(loc.id || loc.name);
    setTimeout(() => setCopiedId(null), 2000);
  };

  const getTypeMeta = (type) => {
    switch (type) {
      case 'charging':
        return {
          icon: BatteryCharging,
          label: 'Charger',
          bg: 'bg-emerald-500/10 text-emerald-300 border-emerald-500/30',
          iconColor: 'text-emerald-400'
        };
      case 'loading':
        return {
          icon: Truck,
          label: 'Dock',
          bg: 'bg-purple-500/10 text-purple-300 border-purple-500/30',
          iconColor: 'text-purple-400'
        };
      case 'inspection':
        return {
          icon: Eye,
          label: 'Inspect',
          bg: 'bg-blue-500/10 text-blue-300 border-blue-500/30',
          iconColor: 'text-blue-400'
        };
      case 'staging':
        return {
          icon: Compass,
          label: 'Staging',
          bg: 'bg-slate-800/70 text-slate-200 border-slate-700',
          iconColor: 'text-slate-300'
        };
      default:
        return {
          icon: Box,
          label: 'Shelf',
          bg: 'bg-orange-500/10 text-orange-300 border-orange-500/30',
          iconColor: 'text-orange-400'
        };
    }
  };

  // Filtered & Sorted locations
  const processedLocations = useMemo(() => {
    return locations
      .filter((loc) => {
        const matchesCategory = selectedCategory === 'ALL' || loc.type === selectedCategory;
        const matchesSearch = loc.name.toLowerCase().includes(searchTerm.toLowerCase()) ||
          (loc.type && loc.type.toLowerCase().includes(searchTerm.toLowerCase()));
        return matchesCategory && matchesSearch;
      })
      .sort((a, b) => {
        if (sortBy === 'DISTANCE') {
          const distA = getDistanceToRobot(a.x, a.y) ?? 999;
          const distB = getDistanceToRobot(b.x, b.y) ?? 999;
          return distA - distB;
        }
        return a.name.localeCompare(b.name);
      });
  }, [locations, selectedCategory, searchTerm, sortBy, robotPose]);

  // Nearest POI calculation
  const nearestPoi = useMemo(() => {
    if (!locations || locations.length === 0 || !robotPose || robotPose.x == null) return null;
    let closest = null;
    let minD = 999999;
    locations.forEach(l => {
      const d = Math.hypot(l.x - robotPose.x, l.y - robotPose.y);
      if (d < minD) {
        minD = d;
        closest = { ...l, distance: d };
      }
    });
    return closest;
  }, [locations, robotPose]);

  const categories = [
    { id: 'ALL', label: 'All', count: locations.length },
    { id: 'shelf', label: 'Shelves', count: locations.filter(l => l.type === 'shelf').length },
    { id: 'charging', label: 'Chargers', count: locations.filter(l => l.type === 'charging').length },
    { id: 'loading', label: 'Docks', count: locations.filter(l => l.type === 'loading').length },
  ];

  return (
    <div className="flex flex-col h-full gap-3 font-mono">
      {/* 1. Top Controls: Search + Add Pose + Sort */}
      <div className="flex flex-col gap-2">
        <div className="flex items-center gap-2">
          {/* Live Search Input */}
          <div className="relative flex-1">
            <Search className="w-3.5 h-3.5 absolute left-3 top-1/2 -translate-y-1/2 text-slate-400" />
            <input
              type="text"
              value={searchTerm}
              onChange={(e) => setSearchTerm(e.target.value)}
              placeholder="Search waypoints by name or type..."
              className="w-full clay-inset bg-slate-800/40 border border-slate-800/90 rounded-xl pl-9 pr-3 py-2 text-xs text-slate-50 placeholder-slate-400 focus:outline-none focus:border-orange-500 font-mono transition-all"
            />
            {searchTerm && (
              <button
                onClick={() => setSearchTerm('')}
                className="absolute right-2.5 top-1/2 -translate-y-1/2 text-xs text-slate-400 hover:text-slate-300 cursor-pointer"
              >
                ✕
              </button>
            )}
          </div>

          {/* Add Waypoint Button */}
          <button
            onClick={() => handleOpenModal()}
            className="btn-clay-orange text-xs px-3.5 py-2 flex items-center gap-1.5 shrink-0 font-extrabold shadow-xs cursor-pointer active:scale-95"
            title="Create a new waypoint at current or custom position"
          >
            <Plus className="w-3.5 h-3.5 text-white" />
            <span>SAVE POSE</span>
          </button>

          {/* Clear All Waypoints Button */}
          {locations.length > 0 && (
            <button
              onClick={() => {
                if (window.confirm('Remove all saved waypoints from POIs and the map?')) {
                  if (onClearAllLocations) {
                    onClearAllLocations();
                  } else {
                    onSaveLocation?.({ action: 'clear_all_locations' });
                  }
                }
              }}
              className="btn-clay-white text-xs px-3 py-2 flex items-center gap-1.5 shrink-0 text-red-400 hover:text-red-300 hover:bg-red-500/10 border-red-500/90 font-bold shadow-xs cursor-pointer active:scale-95"
              title="Remove all waypoints from POIs and map"
            >
              <Trash2 className="w-3.5 h-3.5 text-red-500" />
              <span>CLEAR ALL</span>
            </button>
          )}
        </div>

        {/* 2. Category Filter Pills & Sort Selector */}
        <div className="flex items-center justify-between gap-2 overflow-x-auto no-scrollbar py-0.5">
          <div className="flex items-center gap-1.5 shrink-0">
            {categories.map(cat => (
              <button
                key={cat.id}
                onClick={() => setSelectedCategory(cat.id)}
                className={`px-2.5 py-1 rounded-xl text-[10px] font-bold transition-all cursor-pointer whitespace-nowrap ${
                  selectedCategory === cat.id
                    ? 'btn-clay-orange shadow-2xs font-extrabold text-white'
                    : 'btn-clay-white text-slate-300 hover:text-slate-50'
                }`}
              >
                <span>{cat.label}</span>
                <span className={`ml-1 text-[9px] px-1 py-0.2 rounded-md ${
                  selectedCategory === cat.id ? 'bg-orange-700/60 text-white' : 'bg-slate-700/80 text-slate-300'
                }`}>
                  {cat.count}
                </span>
              </button>
            ))}
          </div>

          {/* Sort Selector */}
          <button
            onClick={() => setSortBy(sortBy === 'DISTANCE' ? 'NAME' : 'DISTANCE')}
            className="btn-clay-white text-[10px] px-2.5 py-1 flex items-center gap-1 text-slate-300 hover:text-slate-50 shrink-0 cursor-pointer"
            title="Toggle sort order"
          >
            <ArrowUpDown className="w-3 h-3 text-orange-500" />
            <span>{sortBy === 'DISTANCE' ? 'By Distance' : 'By Name'}</span>
          </button>
        </div>
      </div>

      {/* 3. Telemetry Quick Bar */}
      {nearestPoi && (
        <div className="px-3 py-1.5 rounded-xl bg-orange-500/10 border border-orange-500/80 flex items-center justify-between text-[10px] text-orange-200 shadow-2xs">
          <div className="flex items-center gap-1.5 truncate">
            <span className="w-2 h-2 rounded-full bg-orange-500 animate-pulse shrink-0"></span>
            <span className="font-bold truncate">NEAREST: {nearestPoi.name}</span>
          </div>
          <span className="font-mono font-extrabold text-orange-400 shrink-0">
            {nearestPoi.distance.toFixed(2)}m away
          </span>
        </div>
      )}

      {/* 4. POI Cards List View */}
      <div className="flex-1 overflow-y-auto space-y-2.5 smooth-scroll pr-1">
        {processedLocations.length === 0 ? (
          <div className="text-center py-12 px-4 rounded-2xl border border-dashed border-slate-700 bg-slate-800/40 text-slate-400 text-xs space-y-2.5">
            <div className="w-10 h-10 rounded-2xl bg-orange-500/15 border border-orange-500/30 text-orange-400 flex items-center justify-center mx-auto shadow-2xs">
              <MapPin className="w-5 h-5 text-orange-500" />
            </div>
            <p className="font-bold text-slate-100 text-sm">No Waypoints Saved</p>
            <p className="text-[11px] text-slate-400 max-w-xs mx-auto leading-relaxed">
              {searchTerm 
                ? `No POIs matching "${searchTerm}". Try another search.`
                : 'All points have been removed from POIs and the map. Click "SAVE POSE" above or "Mark Pin" on the map to add live points.'}
            </p>
          </div>
        ) : (
          processedLocations.map((loc) => {
            const rawDist = getDistanceToRobot(loc.x, loc.y);
            const dist = rawDist != null ? rawDist.toFixed(1) : null;
            const bearing = getRelativeBearing(loc.x, loc.y);
            const meta = getTypeMeta(loc.type);
            const Icon = meta.icon;
            const isCopied = copiedId === (loc.id || loc.name);

            return (
              <div
                key={loc.id || loc.name}
                className="clay-card p-3 rounded-2xl border border-slate-800/90 hover:border-orange-500/40 transition-all flex flex-col sm:flex-row sm:items-center justify-between gap-2.5 shadow-2xs hover:shadow-xs group"
              >
                {/* Left: Type Icon & Info */}
                <div className="flex items-center gap-3 min-w-0">
                  <div className={`p-2.5 rounded-2xl bg-slate-900 border border-slate-800 shadow-2xs shrink-0 group-hover:border-orange-400 transition-all`}>
                    <Icon className={`w-4 h-4 ${meta.iconColor}`} />
                  </div>

                  <div className="min-w-0 flex-1">
                    <div className="flex items-center gap-1.5 truncate">
                      <span className="text-xs font-bold text-slate-50 truncate">
                        {loc.name}
                      </span>
                      <span className={`text-[9px] uppercase font-bold px-1.5 py-0.2 rounded-md border shrink-0 ${meta.bg}`}>
                        {meta.label}
                      </span>
                    </div>

                    {/* Coordinates & Relative Bearing */}
                    <div className="flex items-center gap-2 text-[10px] text-slate-400 mt-0.5 flex-wrap">
                      <span className="text-slate-300">
                        ({Number(loc.x).toFixed(2)}, {Number(loc.y).toFixed(2)})
                      </span>
                      {dist && (
                        <span className="text-orange-400 font-bold flex items-center gap-0.5">
                          • {dist}m {bearing && `(${bearing})`}
                        </span>
                      )}
                    </div>
                  </div>
                </div>

                {/* Right: Actions */}
                <div className="flex items-center gap-1.5 self-end sm:self-center shrink-0">
                  {/* Copy Coords */}
                  <button
                    onClick={(e) => handleCopyCoords(loc, e)}
                    className="btn-clay-white p-2 rounded-xl text-slate-400 hover:text-slate-100 transition-colors cursor-pointer"
                    title="Copy coordinates to clipboard"
                  >
                    {isCopied ? (
                      <Check className="w-3.5 h-3.5 text-emerald-400" />
                    ) : (
                      <Copy className="w-3.5 h-3.5" />
                    )}
                  </button>

                  {/* Dispatch to Pose */}
                  <button
                    onClick={() => onNavigateToLocation?.(loc.name)}
                    className="btn-clay-orange text-white text-xs px-3 py-1.5 font-bold flex items-center gap-1 shadow-xs active:scale-95 cursor-pointer"
                    title={`Send Nav2 dispatch to ${loc.name}`}
                  >
                    <Navigation className="w-3 h-3 text-white" />
                    <span>DISPATCH</span>
                  </button>

                  {/* Delete Button */}
                  <button
                    onClick={() => {
                      if (window.confirm(`Delete waypoint "${loc.name}"?`)) {
                        onDeleteLocation?.(loc.name);
                      }
                    }}
                    className="btn-clay-white p-2 rounded-xl text-slate-400 hover:text-red-400 hover:border-red-500/30 transition-colors cursor-pointer"
                    title="Delete location"
                  >
                    <Trash2 className="w-3.5 h-3.5" />
                  </button>
                </div>
              </div>
            );
          })
        )}
      </div>

      {/* 5. Glassmorphic Save Waypoint Dialog Modal */}
      {modalOpen && (
        <div className="fixed inset-0 bg-slate-950/60 backdrop-blur-md z-50 flex items-center justify-center p-4">
          <div className="clay-card bg-slate-900 border border-slate-800 rounded-3xl max-w-md w-full p-6 shadow-2xl space-y-4 font-mono animate-in fade-in zoom-in-95 duration-150">
            {/* Modal Header */}
            <div className="flex items-center justify-between border-b border-slate-800 pb-3">
              <div className="flex items-center gap-2">
                <div className="p-2 rounded-xl bg-orange-500/10 border border-orange-500/30 text-orange-400 shadow-2xs">
                  <MapPin className="w-4 h-4" />
                </div>
                <div>
                  <h3 className="text-sm font-black text-slate-50 tracking-tight">
                    SAVE WAYPOINT POI
                  </h3>
                  <p className="text-[10px] text-slate-400">Record point of interest on {activeMap || 'warehouse_main'}</p>
                </div>
              </div>
              <button 
                onClick={() => setModalOpen(false)} 
                className="text-slate-400 hover:text-slate-200 p-1 rounded-lg cursor-pointer font-bold text-sm"
              >
                ✕
              </button>
            </div>

            <form onSubmit={handleSave} className="space-y-3.5 text-xs">
              {/* Waypoint Name */}
              <div>
                <label className="block text-slate-200 mb-1 font-bold">
                  WAYPOINT NAME
                </label>
                <input
                  type="text"
                  required
                  value={locName}
                  onChange={(e) => setLocName(e.target.value)}
                  placeholder="e.g. Shelf 02, Inbound Dock B"
                  className="w-full clay-inset bg-slate-800/40 border border-slate-700 rounded-xl px-3 py-2 text-slate-50 placeholder-slate-400 focus:outline-none focus:border-orange-500 font-bold"
                />
              </div>

              {/* Visual Category Selector */}
              <div>
                <label className="block text-slate-200 mb-1.5 font-bold">
                  CATEGORY TYPE
                </label>
                <div className="grid grid-cols-4 gap-2">
                  {[
                    { id: 'shelf', label: 'Shelf', icon: Box },
                    { id: 'charging', label: 'Charger', icon: BatteryCharging },
                    { id: 'loading', label: 'Dock', icon: Truck },
                    { id: 'inspection', label: 'Inspect', icon: Eye }
                  ].map(t => {
                    const TIcon = t.icon;
                    return (
                      <button
                        key={t.id}
                        type="button"
                        onClick={() => setLocType(t.id)}
                        className={`p-2 rounded-xl border flex flex-col items-center gap-1 transition-all cursor-pointer ${
                          locType === t.id
                            ? 'bg-gradient-to-r from-orange-500 to-orange-600 text-white border-orange-400 shadow-xs font-bold'
                            : 'btn-clay-white text-slate-200 hover:text-slate-50'
                        }`}
                      >
                        <TIcon className="w-4 h-4" />
                        <span className="text-[10px]">{t.label}</span>
                      </button>
                    );
                  })}
                </div>
              </div>

              {/* Coordinates with One-Click Current Pose Grabber */}
              <div className="space-y-2 pt-1 border-t border-slate-800">
                <div className="flex items-center justify-between">
                  <span className="font-bold text-slate-200 text-[11px]">COORDINATES</span>
                  <button
                    type="button"
                    onClick={handleUseCurrentPose}
                    className="text-[10px] text-orange-400 hover:text-orange-300 font-bold flex items-center gap-1 bg-orange-500/10 px-2 py-0.5 rounded-lg border border-orange-500/30 cursor-pointer"
                  >
                    <Compass className="w-3 h-3 text-orange-500" />
                    <span>Use Optimus Live Pose</span>
                  </button>
                </div>

                <div className="grid grid-cols-3 gap-2">
                  <div>
                    <label className="block text-slate-400 text-[10px] mb-0.5">X (m)</label>
                    <input
                      type="number"
                      step="0.01"
                      required
                      value={customX}
                      onChange={(e) => {
                        setCustomX(e.target.value);
                        setUseCurrentPose(false);
                      }}
                      className="w-full clay-inset bg-slate-800/40 border border-slate-700 rounded-xl px-2.5 py-1.5 text-slate-50 focus:outline-none focus:border-orange-500 font-mono text-center font-bold"
                    />
                  </div>

                  <div>
                    <label className="block text-slate-400 text-[10px] mb-0.5">Y (m)</label>
                    <input
                      type="number"
                      step="0.01"
                      required
                      value={customY}
                      onChange={(e) => {
                        setCustomY(e.target.value);
                        setUseCurrentPose(false);
                      }}
                      className="w-full clay-inset bg-slate-800/40 border border-slate-700 rounded-xl px-2.5 py-1.5 text-slate-50 focus:outline-none focus:border-orange-500 font-mono text-center font-bold"
                    />
                  </div>

                  <div>
                    <label className="block text-slate-400 text-[10px] mb-0.5">YAW (rad)</label>
                    <input
                      type="number"
                      step="0.01"
                      value={customYaw}
                      onChange={(e) => {
                        setCustomYaw(e.target.value);
                        setUseCurrentPose(false);
                      }}
                      className="w-full clay-inset bg-slate-800/40 border border-slate-700 rounded-xl px-2.5 py-1.5 text-slate-50 focus:outline-none focus:border-orange-500 font-mono text-center font-bold"
                    />
                  </div>
                </div>
              </div>

              {/* Modal Actions */}
              <div className="flex items-center justify-end gap-2 pt-3 border-t border-slate-800">
                <button
                  type="button"
                  onClick={() => setModalOpen(false)}
                  className="btn-clay-white text-xs px-4 py-2 font-bold cursor-pointer"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  className="btn-clay-orange text-xs px-4 py-2 font-extrabold flex items-center gap-1.5 cursor-pointer shadow-xs"
                >
                  <CheckCircle2 className="w-3.5 h-3.5" />
                  <span>Save POI</span>
                </button>
              </div>
            </form>
          </div>
        </div>
      )}
    </div>
  );
}
