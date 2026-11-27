import { useEffect, useRef, useState } from 'react';
import rosConnection from '../ros/rosConnection';

/**
 * Shared camera plumbing for the cockpit CAMERA tab and the Cameras page.
 * Frames come from camera_stream_node as JPEGs over rosbridge:
 * /dashboard/camera/<feed>/compressed. A feed is streamed only while
 * something is subscribed to it.
 */

export const FEED_INFO = {
  front: { label: 'Front', title: 'Front camera (laptop webcam / Pi camera)' },
  obstacles: { label: 'Obstacles', title: 'Camera obstacle detection: red = not floor, green = obstacle base' },
  depth_color: { label: 'Depth RGB', title: 'Depth camera colour image' },
  depth: { label: 'Depth', title: 'Depth map: red = near, blue = far, black = no reading' },
};

// Shown before the robot reports its feed list.
export const FALLBACK_FEEDS = ['front', 'obstacles', 'depth_color', 'depth'].map((name) => ({ name, live: null }));

export const feedLabel = (name) => FEED_INFO[name]?.label || name;

/** Live feed listing from /dashboard/camera/feeds. */
export function useCameraFeeds() {
  const [feeds, setFeeds] = useState(
    rosConnection.cameraFeeds.length ? rosConnection.cameraFeeds : FALLBACK_FEEDS
  );
  useEffect(() => {
    const onFeeds = (list) => setFeeds(list.length ? list : FALLBACK_FEEDS);
    rosConnection.on('cameraFeeds', onFeeds);
    return () => rosConnection.off('cameraFeeds', onFeeds);
  }, []);
  return feeds;
}

/**
 * Stream one feed into an <img>. Frames are written straight to the element
 * (imgRef) so a 10 fps stream does not re-render React on every frame.
 */
export function useCameraStream(feed, { connected, paused = false, throttleMs = 100 }) {
  const imgRef = useRef(null);
  const latestSrcRef = useRef(null);
  const frameCountRef = useRef(0);
  const lastFrameAtRef = useRef(0);
  const [hasFrame, setHasFrame] = useState(false);
  const [stale, setStale] = useState(false);
  const [fps, setFps] = useState(0);

  useEffect(() => {
    setHasFrame(false);
    setStale(false);
    lastFrameAtRef.current = 0;
    latestSrcRef.current = null;
    if (!connected || paused || !feed) return undefined;
    return rosConnection.subscribeCamera(feed, (src) => {
      latestSrcRef.current = src;
      if (imgRef.current) imgRef.current.src = src;
      frameCountRef.current += 1;
      if (!lastFrameAtRef.current) setHasFrame(true);
      lastFrameAtRef.current = Date.now();
    }, throttleMs);
  }, [feed, connected, paused, throttleMs]);

  // fps + stale indicator, once a second.
  useEffect(() => {
    const timer = setInterval(() => {
      setFps(frameCountRef.current);
      frameCountRef.current = 0;
      setStale(lastFrameAtRef.current > 0 && Date.now() - lastFrameAtRef.current > 3000);
    }, 1000);
    return () => clearInterval(timer);
  }, []);

  const snapshot = () => {
    if (!latestSrcRef.current) return;
    const a = document.createElement('a');
    a.href = latestSrcRef.current;
    a.download = `optimus_${feed}_${new Date().toISOString().replace(/[:.]/g, '-')}.jpg`;
    a.click();
  };

  return { imgRef, hasFrame, stale, fps, snapshot };
}

/** Status pill for a stream. */
export function streamStatus({ connected, paused, hasFrame, stale, fps }) {
  if (!connected) return { text: 'OFFLINE', cls: 'bg-slate-800 text-slate-300 border-slate-700' };
  if (paused) return { text: 'PAUSED', cls: 'bg-slate-800 text-slate-200 border-slate-700' };
  if (hasFrame && !stale) return { text: `LIVE • ${fps} FPS`, cls: 'bg-emerald-500/15 text-emerald-300 border-emerald-500/40' };
  if (stale) return { text: 'STALLED', cls: 'bg-amber-500/15 text-amber-300 border-amber-500/40' };
  return { text: 'NO SIGNAL', cls: 'bg-amber-500/15 text-amber-300 border-amber-500/40' };
}

/** Placeholder text shown while a feed has no frame. */
export function emptyFeedText(feed, { connected, paused, live }) {
  if (!connected) return 'Connect to the robot to see its cameras.';
  if (paused) return 'Stream paused.';
  if (live === false) {
    return feed.startsWith('depth')
      ? 'No depth camera running. Start the robot with depth_camera:=realsense (or orbbec).'
      : 'Camera not running. Start the robot with camera:=laptop (or picam).';
  }
  return 'Waiting for frames…';
}

export function toggleFullscreen(el) {
  if (!el) return;
  if (document.fullscreenElement) document.exitFullscreen?.();
  else el.requestFullscreen?.();
}
