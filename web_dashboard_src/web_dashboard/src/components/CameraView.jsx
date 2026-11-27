import React, { useEffect, useRef, useState } from 'react';
import { Camera, Download, Maximize2, Pause, Play, VideoOff } from 'lucide-react';
import {
  FEED_INFO, useCameraFeeds, useCameraStream, streamStatus, emptyFeedText, toggleFullscreen,
} from '../hooks/useCameraStream';

/**
 * Compact live camera view for the cockpit side panel. A feed is only
 * streamed while this panel is open on that feed. The full multi-feed view
 * is the Cameras page (templates/CamerasTemplate.jsx).
 */

const STORAGE_KEY = 'optimus_camera_feed';

function readSavedFeed() {
  try {
    return localStorage.getItem(STORAGE_KEY) || 'front';
  } catch (_) {
    return 'front';
  }
}

export default function CameraView({ connected }) {
  const feeds = useCameraFeeds();
  const [feed, setFeed] = useState(readSavedFeed);
  const [paused, setPaused] = useState(false);
  const boxRef = useRef(null);
  const { imgRef, hasFrame, stale, fps, snapshot } = useCameraStream(feed, { connected, paused });

  // Fall back to a live feed if the saved one does not exist on this robot.
  useEffect(() => {
    if (feeds.some((f) => f.name === feed)) return;
    const live = feeds.find((f) => f.live) || feeds[0];
    if (live) setFeed(live.name);
  }, [feeds, feed]);

  useEffect(() => {
    try {
      localStorage.setItem(STORAGE_KEY, feed);
    } catch (_) {}
  }, [feed]);

  const fullscreen = () => toggleFullscreen(boxRef.current);

  const current = feeds.find((f) => f.name === feed);
  const status = streamStatus({ connected, paused, hasFrame, stale, fps });
  const emptyText = emptyFeedText(feed, { connected, paused, live: current?.live });

  return (
    <div className="flex flex-col gap-3 font-mono">
      {/* Header */}
      <div className="flex items-center justify-between gap-2">
        <div className="flex items-center gap-2 min-w-0">
          <div className="p-2 rounded-xl bg-slate-900 border border-slate-800 shrink-0">
            <Camera className="w-4 h-4 text-orange-400" />
          </div>
          <div className="min-w-0">
            <div className="text-xs font-black text-slate-50">CAMERA VIEW</div>
            <div className="text-[10px] text-slate-400 truncate">/dashboard/camera/{feed}</div>
          </div>
        </div>
        <span className={`text-[9px] px-2 py-0.5 rounded-full font-bold border whitespace-nowrap ${status.cls}`}>
          {status.text}
        </span>
      </div>

      {/* Feed selector */}
      <div className="flex flex-wrap gap-1.5" role="tablist" aria-label="Camera feed">
        {feeds.map((f) => {
          const info = FEED_INFO[f.name] || { label: f.name, title: f.topic || f.name };
          const active = f.name === feed;
          return (
            <button
              key={f.name}
              role="tab"
              aria-selected={active}
              title={info.title}
              onClick={() => setFeed(f.name)}
              className={`px-2.5 py-1 rounded-lg text-[10px] font-bold flex items-center gap-1.5 transition-all cursor-pointer border ${
                active
                  ? 'bg-gradient-to-r from-orange-500 to-orange-600 text-white border-orange-500/60'
                  : 'bg-slate-800/70 text-slate-300 hover:text-slate-50 border-slate-800'
              }`}
            >
              <span
                className={`h-1.5 w-1.5 rounded-full ${
                  f.live ? 'bg-emerald-400' : f.live === false ? 'bg-slate-500' : 'bg-amber-400'
                }`}
              />
              {info.label}
            </button>
          );
        })}
      </div>

      {/* Image */}
      <div
        ref={boxRef}
        className="relative w-full aspect-[4/3] rounded-2xl overflow-hidden bg-black border border-slate-800 flex items-center justify-center"
      >
        <img
          ref={imgRef}
          alt={`${FEED_INFO[feed]?.label || feed} camera`}
          className={`w-full h-full object-contain ${hasFrame ? '' : 'hidden'} ${stale ? 'opacity-50' : ''}`}
        />
        {!hasFrame && (
          <div className="flex flex-col items-center gap-2 text-center px-6">
            <VideoOff className="w-7 h-7 text-slate-600" />
            <span className="text-[11px] text-slate-400 leading-relaxed">{emptyText}</span>
          </div>
        )}
        {feed === 'depth' && hasFrame && (
          <div className="absolute bottom-2 left-2 right-2 flex items-center gap-2 text-[9px] text-slate-200 bg-black/60 rounded-lg px-2 py-1">
            <span>NEAR</span>
            <div className="flex-1 h-1.5 rounded-full bg-gradient-to-r from-red-500 via-yellow-400 via-50% to-blue-600" />
            <span>4 m</span>
          </div>
        )}
      </div>

      {/* Controls */}
      <div className="grid grid-cols-3 gap-2">
        <button
          onClick={() => setPaused((p) => !p)}
          disabled={!connected}
          className="py-2 rounded-xl text-[10px] font-bold bg-slate-800/70 text-slate-200 hover:text-white border border-slate-800 flex items-center justify-center gap-1.5 cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed"
        >
          {paused ? <Play className="w-3.5 h-3.5" /> : <Pause className="w-3.5 h-3.5" />}
          {paused ? 'RESUME' : 'PAUSE'}
        </button>
        <button
          onClick={snapshot}
          disabled={!hasFrame}
          className="py-2 rounded-xl text-[10px] font-bold bg-slate-800/70 text-slate-200 hover:text-white border border-slate-800 flex items-center justify-center gap-1.5 cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed"
        >
          <Download className="w-3.5 h-3.5" />
          SNAPSHOT
        </button>
        <button
          onClick={fullscreen}
          className="py-2 rounded-xl text-[10px] font-bold bg-slate-800/70 text-slate-200 hover:text-white border border-slate-800 flex items-center justify-center gap-1.5 cursor-pointer"
        >
          <Maximize2 className="w-3.5 h-3.5" />
          FULL
        </button>
      </div>

      <p className="text-[10px] text-slate-500 leading-relaxed">
        {FEED_INFO[feed]?.title || current?.topic}. Streams only while this tab is open.
      </p>
    </div>
  );
}
