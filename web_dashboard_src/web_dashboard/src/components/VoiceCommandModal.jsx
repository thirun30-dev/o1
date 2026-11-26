import React, { useCallback, useEffect, useRef, useState } from 'react';
import { Ear, Keyboard, Mic, MicOff, Radio, Send, Volume2, VolumeX, X } from 'lucide-react';
import rosConnection from '../ros/rosConnection';

// Spellings browsers commonly produce for "Optimus" (mirrors optimus_voice's
// intent_parser.WAKE_WORDS; "optimal" is deliberately excluded).
const WAKE = /\b(optimus|optimas|optimist|optimums|optimum)\b[\s,.]*/i;
const ARM_WINDOW_MS = 6000;

const QUICK_COMMANDS = [
  { label: 'Where are you?', command: 'where are you' },
  { label: 'Go home', command: 'go home' },
  { label: 'Start patrol', command: 'start auto navigation' },
  { label: 'List places', command: 'what places can you go' },
  { label: 'Battery', command: 'battery' },
];

const RecognitionCtor = typeof window !== 'undefined'
  ? (window.SpeechRecognition || window.webkitSpeechRecognition)
  : null;

function speakReply(text) {
  if (typeof window === 'undefined' || !window.speechSynthesis) return;
  const utterance = new SpeechSynthesisUtterance(text);
  const voices = window.speechSynthesis.getVoices();
  utterance.voice = voices.find((v) => /male|daniel|david|google uk english male/i.test(v.name)) || voices[0];
  utterance.pitch = 0.3;
  utterance.rate = 0.88;
  window.speechSynthesis.cancel();
  window.speechSynthesis.speak(utterance);
}

function stripWakeWord(text) {
  const match = text.match(WAKE);
  if (!match) return { woke: false, rest: text.trim() };
  return { woke: true, rest: text.slice(match.index + match[0].length).trim() };
}

/**
 * Voice command panel for the optimus_voice node.
 *
 * - Push-to-talk: one spoken command, no wake word needed (Ctrl + , or the mic button).
 * - Hands-free: listens continuously; only speech after "Optimus" is sent.
 * - Typed commands and quick buttons always work, even without speech support.
 *
 * `listenRequest` is a counter: each increment (the Ctrl + , shortcut) starts push-to-talk.
 */
export default function VoiceCommandModal({ isOpen, onClose, connected, listenRequest }) {
  const [mode, setMode] = useState('idle'); // idle | push | handsfree
  const [armed, setArmed] = useState(false);
  const [interim, setInterim] = useState('');
  const [typed, setTyped] = useState('');
  const [log, setLog] = useState([]);
  const [robotState, setRobotState] = useState('unknown');
  const [nodeOnline, setNodeOnline] = useState(null);
  const [micError, setMicError] = useState('');
  const [speakReplies, setSpeakReplies] = useState(() => {
    try {
      return localStorage.getItem('optimus_voice_speak') === '1';
    } catch (_) {
      return false;
    }
  });

  const recognitionRef = useRef(null);
  const armTimerRef = useRef(null);
  const speakRef = useRef(speakReplies);
  const inputRef = useRef(null);
  const logEndRef = useRef(null);
  speakRef.current = speakReplies;

  const push = useCallback((who, text) => {
    setLog((prev) => [...prev.slice(-60), { who, text, time: new Date().toLocaleTimeString() }]);
  }, []);

  const send = useCallback((text) => {
    const command = String(text || '').trim();
    if (!command) return;
    push('YOU', command);
    if (!rosConnection.sendVoiceCommand(command)) {
      push('SYSTEM', 'Not sent: the dashboard is not connected to the robot.');
    }
  }, [push]);

  const disarm = useCallback(() => {
    clearTimeout(armTimerRef.current);
    armTimerRef.current = null;
    setArmed(false);
  }, []);

  const stopListening = useCallback(() => {
    const recognition = recognitionRef.current;
    recognitionRef.current = null;
    if (recognition) {
      try {
        recognition.abort();
      } catch (_) {}
    }
    disarm();
    setInterim('');
    setMode('idle');
  }, [disarm]);

  const startListening = useCallback((nextMode) => {
    if (!RecognitionCtor) {
      setMicError('Speech recognition needs Chrome or Edge. Typed commands still work.');
      return;
    }
    stopListening();
    setMicError('');

    const recognition = new RecognitionCtor();
    recognition.lang = navigator.language?.startsWith('en') ? navigator.language : 'en-IN';
    recognition.continuous = nextMode === 'handsfree';
    recognition.interimResults = true;

    recognition.onresult = (event) => {
      let partial = '';
      for (let i = event.resultIndex; i < event.results.length; i += 1) {
        const result = event.results[i];
        const said = result[0].transcript.trim();
        if (!result.isFinal) {
          partial += `${said} `;
          continue;
        }
        const { woke, rest } = stripWakeWord(said);
        if (nextMode === 'push') {
          // Push-to-talk: the operator asked to be heard, no wake word needed.
          send(rest || said);
        } else if (woke && rest) {
          disarm();
          send(rest);
        } else if (woke) {
          push('OPTIMUS', 'Listening...');
          clearTimeout(armTimerRef.current);
          armTimerRef.current = setTimeout(disarm, ARM_WINDOW_MS);
          setArmed(true);
        } else if (armTimerRef.current) {
          disarm();
          send(said);
        }
      }
      setInterim(partial.trim());
    };

    recognition.onerror = (event) => {
      if (event.error === 'no-speech' || event.error === 'aborted') return;
      const reasons = {
        'not-allowed': 'Microphone blocked. Allow mic access; browsers only allow it on https:// or localhost.',
        'service-not-allowed': 'Speech service not allowed on this page (needs https:// or localhost).',
        'audio-capture': 'No microphone found.',
        network: 'Speech recognition needs an internet connection in this browser.',
      };
      setMicError(reasons[event.error] || `Speech recognition error: ${event.error}`);
      recognitionRef.current = null;
      setMode('idle');
    };

    recognition.onend = () => {
      if (recognitionRef.current !== recognition) return;
      if (nextMode === 'handsfree') {
        // Browsers end continuous sessions after silence; keep listening.
        try {
          recognition.start();
          return;
        } catch (_) {}
      }
      recognitionRef.current = null;
      setInterim('');
      setMode('idle');
    };

    recognitionRef.current = recognition;
    try {
      recognition.start();
      setMode(nextMode);
    } catch (err) {
      recognitionRef.current = null;
      setMicError(`Could not start the microphone: ${err.message}`);
    }
  }, [disarm, push, send, stopListening]);

  // Robot replies and state from optimus_voice.
  useEffect(() => {
    const onResponse = (text) => {
      push('OPTIMUS', text);
      if (speakRef.current) speakReply(text);
    };
    const onState = (state) => {
      setRobotState(state);
      setNodeOnline(true);
    };
    rosConnection.on('voiceResponse', onResponse);
    rosConnection.on('voiceState', onState);
    return () => {
      rosConnection.off('voiceResponse', onResponse);
      rosConnection.off('voiceState', onState);
    };
  }, [push]);

  // Check that the voice node is actually running whenever the panel opens.
  useEffect(() => {
    if (!isOpen) return undefined;
    let cancelled = false;
    if (!connected) {
      setNodeOnline(null);
      return undefined;
    }
    rosConnection.isVoiceNodeOnline().then((online) => {
      if (!cancelled) setNodeOnline(online);
    });
    return () => {
      cancelled = true;
    };
  }, [isOpen, connected]);

  // Ctrl + , (listenRequest increments) starts push-to-talk.
  useEffect(() => {
    if (isOpen && listenRequest > 0) startListening('push');
    // Only react to new shortcut presses, not to startListening identity changes.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [listenRequest]);

  // Closing the panel always releases the microphone.
  useEffect(() => {
    if (!isOpen) stopListening();
  }, [isOpen, stopListening]);

  useEffect(() => () => stopListening(), [stopListening]);

  // Esc closes the panel.
  useEffect(() => {
    if (!isOpen) return undefined;
    const onKey = (e) => {
      if (e.key === 'Escape') {
        e.preventDefault();
        onClose();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [isOpen, onClose]);

  useEffect(() => {
    logEndRef.current?.scrollIntoView({ block: 'nearest' });
  }, [log, interim]);

  useEffect(() => {
    try {
      localStorage.setItem('optimus_voice_speak', speakReplies ? '1' : '0');
    } catch (_) {}
    if (!speakReplies && typeof window !== 'undefined') window.speechSynthesis?.cancel();
  }, [speakReplies]);

  if (!isOpen) return null;

  const listening = mode !== 'idle';
  const statusText = !connected
    ? 'ROBOT DISCONNECTED'
    : nodeOnline === false
      ? 'VOICE NODE OFFLINE'
      : armed
        ? 'SAY YOUR COMMAND'
        : mode === 'push'
          ? 'LISTENING'
          : mode === 'handsfree'
            ? 'WAITING FOR "OPTIMUS"'
            : 'READY';

  return (
    <div
      className="fixed inset-0 z-50 flex items-end sm:items-center justify-center p-3 sm:p-5 bg-slate-950/70 backdrop-blur-md animate-fade-in font-mono"
      onMouseDown={(e) => {
        if (e.target === e.currentTarget) onClose();
      }}
      role="dialog"
      aria-modal="true"
      aria-label="Voice command"
    >
      <div className="bg-slate-900 border border-slate-700/80 rounded-3xl max-w-xl w-full max-h-[90vh] flex flex-col shadow-2xl overflow-hidden text-white">
        {/* Header */}
        <div className="px-5 py-3.5 bg-slate-950 border-b border-slate-800 flex items-center justify-between gap-3">
          <div className="flex items-center gap-3 min-w-0">
            <div className="p-2 rounded-2xl bg-orange-500/20 text-orange-400 border border-orange-500/30">
              <Mic className="w-5 h-5" />
            </div>
            <div className="min-w-0">
              <h2 className="text-sm sm:text-base font-black tracking-tight">VOICE COMMAND</h2>
              <p className="text-[10px] text-slate-400 truncate">
                <kbd className="px-1 rounded bg-slate-800 border border-slate-700 text-slate-300">Ctrl</kbd>
                {' + '}
                <kbd className="px-1 rounded bg-slate-800 border border-slate-700 text-slate-300">,</kbd>
                {' to speak · '}
                <kbd className="px-1 rounded bg-slate-800 border border-slate-700 text-slate-300">Esc</kbd>
                {' to close · robot: '}
                <span className="text-orange-400">{robotState}</span>
              </p>
            </div>
          </div>
          <button
            onClick={onClose}
            className="p-2 rounded-xl text-slate-400 hover:text-white hover:bg-slate-800 transition-colors cursor-pointer"
            title="Close (Esc)"
            aria-label="Close voice command"
          >
            <X className="w-4 h-4" />
          </button>
        </div>

        <div className="p-5 flex flex-col gap-4 overflow-y-auto">
          {/* Status + main mic button */}
          <div className="flex items-center gap-4">
            <button
              onClick={() => (listening ? stopListening() : startListening('push'))}
              disabled={!RecognitionCtor}
              className={`relative shrink-0 w-16 h-16 rounded-full flex items-center justify-center border-2 transition-all cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed active:scale-95 ${
                listening
                  ? 'bg-red-600 border-red-400 shadow-[0_0_24px_rgba(239,68,68,0.55)]'
                  : 'bg-orange-500 border-orange-500/40 hover:bg-orange-400 shadow-md'
              }`}
              title={listening ? 'Stop listening' : 'Speak one command (Ctrl + ,)'}
              aria-label={listening ? 'Stop listening' : 'Speak one command'}
            >
              {listening && <span className="absolute inset-0 rounded-full border-2 border-red-400 animate-ping" />}
              {listening ? <MicOff className="w-7 h-7" /> : <Mic className="w-7 h-7" />}
            </button>
            <div className="min-w-0 flex-1">
              <div className={`text-xs font-extrabold tracking-wide ${
                !connected || nodeOnline === false ? 'text-red-400' : listening ? 'text-emerald-400' : 'text-slate-300'
              }`}>
                {statusText}
              </div>
              <div className="text-[11px] text-slate-400 mt-1 min-h-[1.25rem] truncate">
                {interim
                  ? <span className="text-slate-200 italic">“{interim}”</span>
                  : listening && mode === 'push'
                    ? 'Say e.g. "go to the loading bay"'
                    : 'Press the mic or Ctrl + , and speak'}
              </div>
            </div>
          </div>

          {nodeOnline === false && connected && (
            <div className="text-[11px] rounded-xl border border-red-800 bg-red-950/60 text-red-200 px-3 py-2">
              The optimus_voice node is not running, so commands will not be executed. Start it with{' '}
              <code className="text-red-100">ros2 launch optimus_voice voice.launch.py enable_mic:=false</code>
              {' '}or <code className="text-red-100">bringup.launch.py use_voice:=true</code>.
            </div>
          )}
          {micError && (
            <div className="text-[11px] rounded-xl border border-amber-800 bg-amber-950/60 text-amber-200 px-3 py-2">
              {micError}
            </div>
          )}

          {/* Options */}
          <div className="flex flex-wrap gap-2 text-[11px]">
            <button
              onClick={() => (mode === 'handsfree' ? stopListening() : startListening('handsfree'))}
              disabled={!RecognitionCtor}
              className={`flex items-center gap-1.5 px-3 py-1.5 rounded-xl border font-bold transition-all cursor-pointer disabled:opacity-40 disabled:cursor-not-allowed ${
                mode === 'handsfree'
                  ? 'bg-emerald-900/70 border-emerald-600 text-emerald-200'
                  : 'bg-slate-800 border-slate-700 text-slate-300 hover:text-white'
              }`}
              title='Listen continuously and act on anything said after "Optimus"'
            >
              <Ear className="w-3.5 h-3.5" />
              {mode === 'handsfree' ? 'Hands-free ON' : 'Hands-free ("Optimus, ...")'}
            </button>
            <button
              onClick={() => setSpeakReplies((v) => !v)}
              className={`flex items-center gap-1.5 px-3 py-1.5 rounded-xl border font-bold transition-all cursor-pointer ${
                speakReplies
                  ? 'bg-sky-900/70 border-sky-600 text-sky-200'
                  : 'bg-slate-800 border-slate-700 text-slate-300 hover:text-white'
              }`}
              title="Also speak the robot's replies in this browser"
            >
              {speakReplies ? <Volume2 className="w-3.5 h-3.5" /> : <VolumeX className="w-3.5 h-3.5" />}
              Speak replies here
            </button>
          </div>

          {/* Quick commands */}
          <div className="flex flex-wrap gap-2">
            {QUICK_COMMANDS.map(({ label, command }) => (
              <button
                key={command}
                onClick={() => send(command)}
                className="px-3 py-1.5 rounded-xl bg-slate-800 hover:bg-slate-700 border border-slate-700 text-xs font-bold text-slate-200 transition-all cursor-pointer active:scale-95"
              >
                {label}
              </button>
            ))}
            <button
              onClick={() => send('stop')}
              className="px-3 py-1.5 rounded-xl bg-red-700 hover:bg-red-600 border border-red-500/50 text-xs font-extrabold text-white transition-all cursor-pointer active:scale-95"
            >
              STOP
            </button>
          </div>

          {/* Typed command */}
          <form
            onSubmit={(e) => {
              e.preventDefault();
              send(typed);
              setTyped('');
            }}
            className="flex gap-2"
          >
            <div className="flex-1 flex items-center gap-2 bg-slate-950 border border-slate-700 rounded-xl px-3 focus-within:border-orange-500">
              <Keyboard className="w-3.5 h-3.5 text-slate-400 shrink-0" />
              <input
                ref={inputRef}
                value={typed}
                onChange={(e) => setTyped(e.target.value)}
                placeholder="Type a command, e.g. move forward 1 meter"
                className="flex-1 bg-transparent py-2 text-xs text-white placeholder:text-slate-400 outline-none min-w-0"
                aria-label="Typed voice command"
              />
            </div>
            <button
              type="submit"
              disabled={!typed.trim()}
              className="flex items-center gap-1.5 px-3 py-2 rounded-xl bg-orange-500 hover:bg-orange-400 disabled:opacity-40 disabled:cursor-not-allowed text-xs font-extrabold text-white transition-all cursor-pointer"
            >
              <Send className="w-3.5 h-3.5" />
              Send
            </button>
          </form>

          {/* Conversation log */}
          <div className="rounded-2xl bg-slate-950 border border-slate-800 p-3 h-48 overflow-y-auto text-[11px] leading-relaxed">
            {log.length === 0 ? (
              <div className="h-full flex flex-col items-center justify-center text-slate-400 gap-1.5 text-center">
                <Radio className="w-4 h-4" />
                <span>Try “go to the storage rack”, “turn left 90 degrees” or “stop”.</span>
              </div>
            ) : (
              log.map((entry, i) => (
                <div key={i} className="py-1 border-b border-slate-900 last:border-0">
                  <span className="text-slate-300">{entry.time} </span>
                  <span className={`font-extrabold ${
                    entry.who === 'OPTIMUS' ? 'text-sky-400' : entry.who === 'SYSTEM' ? 'text-amber-400' : 'text-emerald-400'
                  }`}>
                    {entry.who}:
                  </span>{' '}
                  <span className="text-slate-200">{entry.text}</span>
                </div>
              ))
            )}
            <div ref={logEndRef} />
          </div>
        </div>
      </div>
    </div>
  );
}
