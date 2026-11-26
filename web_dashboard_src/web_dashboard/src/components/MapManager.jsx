import React, { useState } from 'react';
import { Map, Save, FolderOpen, Check, AlertCircle } from 'lucide-react';

export default function MapManager({
  maps,
  activeMap,
  onSaveMap,
  onSerializePoseGraph,
  onResumeMapping,
  onLoadMap,
  operationResult
}) {
  const [mapInput, setMapInput] = useState('');
  const [saveStatus, setSaveStatus] = useState(null);

  const validateName = (name) => {
    if (!name || !name.trim()) return false;
    return /^[a-zA-Z0-9_\-]+$/.test(name.trim());
  };

  const handleSave = (e) => {
    e.preventDefault();
    const name = mapInput.trim();
    if (!validateName(name)) {
      setSaveStatus({ ok: false, msg: 'Invalid name. Letters, numbers, - and _ only.' });
      return;
    }
    onSaveMap(name);
    setSaveStatus({ ok: true, msg: `Map '${name}' occupancy save requested.` });
    setTimeout(() => setSaveStatus(null), 4000);
  };

  const handleSerialize = () => {
    const name = mapInput.trim() || activeMap || (maps[0] || 'warehouse_main');
    if (!validateName(name)) {
      setSaveStatus({ ok: false, msg: 'Enter or select a valid map name to serialize.' });
      return;
    }
    if (onSerializePoseGraph) {
      onSerializePoseGraph(name);
    }
    setSaveStatus({ ok: true, msg: `Pose graph serialization for '${name}' requested.` });
    setTimeout(() => setSaveStatus(null), 4000);
  };

  return (
    <div className="clay-card flex flex-col h-full overflow-hidden">
      {/* Header */}
      <div className="panel-header py-2.5 px-4 bg-slate-950 border-b border-slate-800 text-white">
        <span className="flex items-center gap-2 font-bold text-white text-xs">
          <Map className="w-4 h-4 text-orange-500" />
          MAP MANAGEMENT & STORAGE
        </span>
        <span className="text-[10px] font-mono px-2 py-0.5 rounded-full bg-slate-800 border border-slate-700 text-orange-400 font-bold">
          STORAGE
        </span>
      </div>

      <div className="p-4 space-y-4">
        {/* Save Map Form */}
        <form onSubmit={handleSave} className="space-y-2">
          <label className="block text-xs font-mono font-bold text-slate-200 uppercase">
            CURRENT SLAM OCCUPANCY MAP (/map)
          </label>
          <div className="flex gap-2">
            <input
              type="text"
              value={mapInput}
              onChange={(e) => setMapInput(e.target.value)}
              placeholder="e.g. warehouse_sector_b"
              className="flex-1 clay-inset bg-slate-800/40 border border-slate-700 rounded-xl px-3 py-2 text-xs text-slate-50 font-mono focus:border-orange-500 focus:outline-none"
            />
            <button
              type="submit"
              className="btn-clay-orange text-xs px-3.5 py-2 flex items-center gap-1.5 shrink-0 font-bold cursor-pointer"
              title="Save 2D occupancy grid (.yaml and .pgm)"
            >
              <Save className="w-3.5 h-3.5" />
              SAVE
            </button>
            <button
              type="button"
              onClick={handleSerialize}
              className="btn-clay-white text-xs px-3 py-2 flex items-center gap-1 shrink-0 font-bold hover:border-purple-500/40 text-purple-300 cursor-pointer"
              title="Serialize SLAM Toolbox pose graph (.posegraph and .data)"
            >
              SERIALIZE
            </button>
          </div>

          {(saveStatus || operationResult) && (
            <div className={`text-[11px] font-mono flex items-center gap-1.5 mt-1.5 ${
              (operationResult ? operationResult.success : saveStatus?.ok) ? 'text-emerald-400' : 'text-red-400'
            }`}>
              {(operationResult ? operationResult.success : saveStatus?.ok) ? (
                <Check className="w-3.5 h-3.5 shrink-0" />
              ) : (
                <AlertCircle className="w-3.5 h-3.5 shrink-0" />
              )}
              <span className="truncate">
                {operationResult?.message || saveStatus?.msg}
              </span>
            </div>
          )}
        </form>

        {/* Available Maps List */}
        <div>
          <label className="block text-xs font-mono font-bold text-slate-200 mb-2 uppercase">
            STORED MAPS ({maps.length})
          </label>
          <div className="space-y-2 max-h-[250px] overflow-y-auto pr-1 smooth-scroll">
            {maps.length === 0 ? (
              <div className="text-center py-6 text-xs font-mono text-slate-400">
                No saved maps found in storage.
              </div>
            ) : (
              maps.map((name) => {
                const isActive = name === activeMap;
                return (
                  <div
                    key={name}
                    className={`p-3 rounded-2xl border text-xs font-mono flex items-center justify-between gap-2.5 transition-all shadow-2xs ${
                      isActive 
                        ? 'bg-orange-500/10 border-orange-500/40 text-orange-200 font-bold ring-1 ring-orange-500/40' 
                        : 'clay-inset bg-slate-800/40 border-slate-800 text-slate-100 hover:border-slate-700'
                    }`}
                  >
                    <div className="flex items-center gap-2.5 truncate min-w-0">
                      <FolderOpen className={`w-4 h-4 shrink-0 ${isActive ? 'text-orange-400' : 'text-slate-400'}`} />
                      <span className="truncate font-bold">{name}</span>
                      {isActive && (
                        <span className="text-[9px] px-2 py-0.5 rounded-full bg-orange-500/15 text-orange-200 border border-orange-500/30 font-bold shrink-0">
                          ACTIVE
                        </span>
                      )}
                    </div>
                    <div className="flex items-center gap-1.5 shrink-0">
                      {onResumeMapping && (
                        <button
                          onClick={() => onResumeMapping(name)}
                          className="btn-clay-white text-[10px] px-2.5 py-1 hover:border-purple-500/40 hover:text-purple-300 cursor-pointer font-bold"
                          title="Resume SLAM mapping using serialized pose graph"
                        >
                          RESUME
                        </button>
                      )}
                      <button
                        onClick={() => onLoadMap(name)}
                        disabled={isActive}
                        className={`text-[11px] px-3 py-1 rounded-xl shrink-0 font-bold transition-all ${
                          isActive 
                            ? 'btn-clay-white opacity-50 cursor-not-allowed' 
                            : 'btn-clay-orange shadow-xs'
                        }`}
                        title="Load map for AMCL localization"
                      >
                        {isActive ? 'LOADED' : 'LOAD MAP'}
                      </button>
                    </div>
                  </div>
                );
              })
            )}
          </div>
        </div>
      </div>
    </div>
  );
}
