import React, { useEffect, useRef, useState } from 'react';
import {
  Camera, Download, Maximize2, Pause, Play, VideoOff, LayoutGrid, Square, Gauge, Navigation,
} from 'lucide-react';
import {
  FEED_INFO, feedLabel, useCameraFeeds, useCameraStream, streamStatus, emptyFeedText, toggleFullscreen,
} from '../../hooks/useCameraStream';

/**
 * Cameras page: every robot camera feed at once (GRID) or one large feed with
 * live thumbnails of the others (FOCUS). Each visible tile holds its own
 * rosbridge subscription; thumbnails are throttled to save bandwidth.
 */

const LAYOUT_KEY = 'optimus_cameras_layout';
const FOCUS_KEY = 'optimus_cameras_focus';

function readPref(key, fallback) {
  try {
    return localStorage.getItem(key) || fallback;
  } catch (_) {
    return fallback;
  }
}

function writePref(key, value) {
  try {
    localStorage.setItem(key, value);
  } catch (_) {}
}

function DepthLegend() {
  return (
    <div className="absolute bottom-2 left-2 right-2 flex items-center gap-2 text-[9px] text-slate-200 bg-black/60 rounded-lg px-2 py-1">
      <span>NEAR</span>
      <div className="flex-1 h-1.5 rounded-full bg-gradient-to-r from-red-500 via-yellow-400 via-50% to-blue-600" />
      <span>4 m</span>
    </div>
  );
}

function IconButton({ onClick, disabled, title, children }) {
  return (
    <button
      onClick={onClick}
      disabled={disabled}
      title={title}
      aria-label={title}
      className="p-1.5 rounded-lg bg-black/50 text-slate-200 hover:text-white hover:bg-black/70 border border-white/10 cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed"
    >
      {children}
    </button>
  );
}

/** One camera feed: image, status pill, snapshot and fullscreen. */
function FeedTile({ feed, live, connected, paused, size = 'grid', onSelect, hud }) {
  const boxRef = useRef(null);
  const throttleMs = size === 'thumb' ? 400 : 100;
  const { imgRef, hasFrame, stale, fps, snapshot } = useCameraStream(feed, { connected, paused, throttleMs });
  const status = streamStatus({ connected, paused, hasFrame, stale, fps });
  const thumb = size === 'thumb';

  return (
    <div
      ref={boxRef}
      onClick={thumb ? onSelect : undefined}
      className={`relative w-full ${size === 'grid' ? 'aspect-video' : 'aspect-[4/3]'} rounded-2xl overflow-hidden bg-black border flex items-center justify-center font-mono ${
        thumb ? 'border-slate-800 hover:border-orange-500/60 cursor-pointer' : 'border-slate-800'
      }`}
    >
      <img
        ref={imgRef}
        alt={`${feedLabel(feed)} camera`}
        className={`w-full h-full object-contain ${hasFrame ? '' : 'hidden'} ${stale ? 'opacity-50' : ''}`}
      />

      {!hasFrame && (
        <div className="flex flex-col items-center gap-2 text-center px-4">
          <VideoOff className={thumb ? 'w-5 h-5 text-slate-600' : 'w-8 h-8 text-slate-600'} />
          {!thumb && (
            <span className="text-[11px] text-slate-400 leading-relaxed max-w-xs">
              {emptyFeedText(feed, { connected, paused, live })}
            </span>
          )}
        </div>
      )}

      {/* Top bar: name + status */}
      <div className="absolute top-2 left-2 right-2 flex items-center justify-between gap-2 pointer-events-none">
        <span className="text-[10px] font-black text-white bg-black/60 rounded-lg px-2 py-0.5 flex items-center gap-1.5">
          <span className={`h-1.5 w-1.5 rounded-full ${
            live ? 'bg-emerald-400' : live === false ? 'bg-slate-500' : 'bg-amber-400'
          }`} />
          {feedLabel(feed).toUpperCase()}
        </span>
        {!thumb && (
          <span className={`text-[9px] px-2 py-0.5 rounded-full font-bold border whitespace-nowrap backdrop-blur-sm ${status.cls}`}>
            {status.text}
          </span>
        )}
      </div>

      {hud}

      {feed === 'depth' && hasFrame && !thumb && <DepthLegend />}

      {/* Tile actions */}
      {!thumb && (
        <div className={`absolute right-2 flex gap-1.5 ${feed === 'depth' && hasFrame ? 'bottom-9' : 'bottom-2'}`}>
          {size === 'grid' && onSelect && (
            <IconButton onClick={onSelect} title="Focus this feed">
              <Square className="w-3.5 h-3.5" />
            </IconButton>
          )}
          <IconButton onClick={snapshot} disabled={!hasFrame} title="Save snapshot">
            <Download className="w-3.5 h-3.5" />
          </IconButton>
          <IconButton onClick={() => toggleFullscreen(boxRef.current)} title="Fullscreen">
            <Maximize2 className="w-3.5 h-3.5" />
          </IconButton>
        </div>
      )}
    </div>
  );
}

/** Speed + navigation readout drawn over the focused feed. */
function DriveHud({ velocity, navStatus }) {
  const lin = Number(velocity?.linear || 0);
  const ang = Number(velocity?.angular || 0);
  const nav = navStatus?.status || 'IDLE';
  return (
    <div className="absolute top-9 left-2 right-2 flex flex-wrap gap-1.5 pointer-events-none text-[10px] font-bold">
      <span className="bg-black/60 text-slate-100 rounded-lg px-2 py-1 flex items-center gap-1.5">
        <Gauge className="w-3 h-3 text-orange-400" />
        {lin.toFixed(2)} m/s · {ang.toFixed(2)} rad/s
      </span>
      <span className={`rounded-lg px-2 py-1 flex items-center gap-1.5 ${
        nav === 'NAVIGATING' ? 'bg-orange-500/90 text-white' : 'bg-black/60 text-slate-100'
      }`}>
        <Navigation className="w-3 h-3" />
        NAV2 {nav}
        {navStatus?.distance_remaining != null && ` · ${navStatus.distance_remaining} m`}
      </span>
    </div>
  );
}

export default function CamerasTemplate({ connected, velocity, navStatus }) {
  const feeds = useCameraFeeds();
  const [layout, setLayout] = useState(() => readPref(LAYOUT_KEY, 'GRID'));
  const [focus, setFocus] = useState(() => readPref(FOCUS_KEY, 'front'));
  const [paused, setPaused] = useState(false);

  useEffect(() => writePref(LAYOUT_KEY, layout), [layout]);
  useEffect(() => writePref(FOCUS_KEY, focus), [focus]);

  // Keep the focused feed valid for this robot.
  useEffect(() => {
    if (feeds.some((f) => f.name === focus)) return;
    const live = feeds.find((f) => f.live) || feeds[0];
    if (live) setFocus(live.name);
  }, [feeds, focus]);

  const liveCount = feeds.filter((f) => f.live).length;
  const focused = feeds.find((f) => f.name === focus) || feeds[0];
  const others = feeds.filter((f) => f.name !== focused?.name);

  const selectFeed = (name) => {
    setFocus(name);
    setLayout('FOCUS');
  };

  const layoutBtn = (id, Icon, label) => (
    <button
      onClick={() => setLayout(id)}
      aria-pressed={layout === id}
      className={`px-3 py-1.5 rounded-xl text-xs font-bold flex items-center gap-1.5 transition-all cursor-pointer ${
        layout === id
          ? 'bg-gradient-to-r from-orange-500 to-orange-600 text-white shadow-xs'
          : 'text-slate-400 hover:text-white hover:bg-slate-800/80'
      }`}
    >
      <Icon className="w-3.5 h-3.5" />
      {label}
    </button>
  );

  return (
    <div className="flex flex-col gap-3.5 flex-1 font-mono">
      {/* Toolbar */}
      <div className="clay-card p-3 flex flex-wrap items-center justify-between gap-3">
        <div className="flex items-center gap-2.5 min-w-0">
          <div className="p-2 rounded-xl bg-slate-900 border border-slate-800 shrink-0">
            <Camera className="w-4 h-4 text-orange-400" />
          </div>
          <div className="min-w-0">
            <div className="text-sm font-black text-slate-50">CAMERAS</div>
            <div className="text-[10px] text-slate-400">
              {connected ? `${liveCount} of ${feeds.length} feeds live` : 'Robot offline'} · /dashboard/camera/*
            </div>
          </div>
        </div>

        <div className="flex items-center gap-2">
          <div className="flex items-center p-1 rounded-2xl bg-slate-900/90 border border-slate-800 gap-1">
            {layoutBtn('GRID', LayoutGrid, 'GRID')}
            {layoutBtn('FOCUS', Square, 'FOCUS')}
          </div>
          <button
            onClick={() => setPaused((p) => !p)}
            disabled={!connected}
            className="px-3 py-2 rounded-xl text-xs font-bold bg-slate-800/70 text-slate-200 hover:text-white border border-slate-800 flex items-center gap-1.5 cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed"
          >
            {paused ? <Play className="w-3.5 h-3.5" /> : <Pause className="w-3.5 h-3.5" />}
            {paused ? 'RESUME ALL' : 'PAUSE ALL'}
          </button>
        </div>
      </div>

      {/* GRID: all feeds at once */}
      {layout === 'GRID' && (
        <div className="grid grid-cols-1 md:grid-cols-2 gap-3.5">
          {feeds.map((f) => (
            <FeedTile
              key={f.name}
              feed={f.name}
              live={f.live}
              connected={connected}
              paused={paused}
              onSelect={() => selectFeed(f.name)}
            />
          ))}
        </div>
      )}

      {/* FOCUS: one large feed + live thumbnails */}
      {layout === 'FOCUS' && focused && (
        <div className="grid grid-cols-1 lg:grid-cols-12 gap-3.5">
          <div className="lg:col-span-9">
            <FeedTile
              feed={focused.name}
              live={focused.live}
              connected={connected}
              paused={paused}
              size="focus"
              hud={<DriveHud velocity={velocity} navStatus={navStatus} />}
            />
          </div>
          <div className="lg:col-span-3 grid grid-cols-3 lg:grid-cols-1 gap-2.5 content-start">
            {others.map((f) => (
              <FeedTile
                key={f.name}
                feed={f.name}
                live={f.live}
                connected={connected}
                paused={paused}
                size="thumb"
                onSelect={() => setFocus(f.name)}
              />
            ))}
          </div>
        </div>
      )}

      <p className="text-[10px] text-slate-500 leading-relaxed">
        {layout === 'FOCUS' && focused
          ? `${FEED_INFO[focused.name]?.title || focused.name}. Click a thumbnail to switch.`
          : 'Each feed streams only while it is on screen. Use FOCUS on a slow link: thumbnails refresh at 2.5 fps.'}
      </p>
    </div>
  );
}
