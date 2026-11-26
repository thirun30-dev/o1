/**
 * VoiceCommandPanel - drop into web_dashboard.
 *
 *   npm install roslib
 *   import VoiceCommandPanel from "./VoiceCommandPanel";
 *   <VoiceCommandPanel ros={ros} />      // ros = new ROSLIB.Ros({url: "ws://<laptop-ip>:9090"})
 *
 * Browser listens continuously (Chrome / Edge, on localhost or https).
 * When it hears "Optimus ...", the rest is sent to /optimus/voice/text_command.
 * Replies from /optimus/voice/response are shown and, optionally, spoken
 * in the browser with a low-pitched voice (the robot laptop also speaks them).
 */
import { useEffect, useRef, useState } from "react";
import ROSLIB from "roslib";

const WAKE = /\b(optimus|optimas|optimist|optimum)\b[\s,.]*/i;
const QUICK = [
  ["Start auto navigation", "start auto navigation"],
  ["Go home", "go home"],
  ["Where are you?", "where are you"],
  ["STOP", "stop"],
];

export default function VoiceCommandPanel({ ros }) {
  const [listening, setListening] = useState(false);
  const [armed, setArmed] = useState(false);
  const [state, setState] = useState("idle");
  const [log, setLog] = useState([]);
  const [typed, setTyped] = useState("");
  const [speakInBrowser, setSpeakInBrowser] = useState(false);
  const recRef = useRef(null);
  const armedRef = useRef(null);       // timeout id while waiting after bare "Optimus"
  const cmdTopic = useRef(null);
  const speakRef = useRef(speakInBrowser);
  speakRef.current = speakInBrowser;

  const supported = typeof window !== "undefined" &&
    (window.SpeechRecognition || window.webkitSpeechRecognition);

  const push = (who, text) =>
    setLog((l) => [...l.slice(-40), { who, text, t: new Date().toLocaleTimeString() }]);

  // ---- ROS wiring
  useEffect(() => {
    if (!ros) return;
    cmdTopic.current = new ROSLIB.Topic({ ros, name: "/optimus/voice/text_command", messageType: "std_msgs/String" });
    const resp = new ROSLIB.Topic({ ros, name: "/optimus/voice/response", messageType: "std_msgs/String" });
    const st = new ROSLIB.Topic({ ros, name: "/optimus/voice/state", messageType: "std_msgs/String" });
    resp.subscribe((m) => { push("OPTIMUS", m.data); if (speakRef.current) speak(m.data); });
    st.subscribe((m) => setState(m.data));
    return () => { resp.unsubscribe(); st.unsubscribe(); };
  }, [ros]);

  const send = (text) => {
    if (!text.trim()) return;
    push("You", text);
    cmdTopic.current?.publish(new ROSLIB.Message({ data: text }));
  };

  // ---- Browser speech recognition with "Optimus" wake word
  const start = () => {
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;
    const rec = new SR();
    rec.lang = "en-IN";
    rec.continuous = true;
    rec.interimResults = false;
    rec.onresult = (e) => {
      const said = e.results[e.results.length - 1][0].transcript.trim();
      const m = said.match(WAKE);
      if (m) {
        const rest = said.slice(m.index + m[0].length).trim();
        if (rest) { disarm(); send(rest); }
        else arm();
      } else if (armedRef.current) {
        disarm();
        send(said);
      }
    };
    rec.onend = () => { if (recRef.current === rec) rec.start(); }; // keep alive
    rec.onerror = (e) => { if (e.error === "not-allowed") stop(); };
    recRef.current = rec;
    rec.start();
    setListening(true);
  };
  const stop = () => {
    const r = recRef.current; recRef.current = null; r?.stop();
    setListening(false); disarm();
  };
  const arm = () => {
    clearTimeout(armedRef.current);
    armedRef.current = setTimeout(disarm, 6000);
    setArmed(true);
    push("OPTIMUS", "Listening...");
  };
  const disarm = () => { clearTimeout(armedRef.current); armedRef.current = null; setArmed(false); };
  useEffect(() => () => stop(), []);

  // ---- styles
  const box = { background: "#0d1117", color: "#e6edf3", border: "1px solid #30363d", borderRadius: 12, padding: 16, fontFamily: "system-ui, sans-serif" };
  const btn = (bg) => ({ background: bg, color: "#fff", border: 0, borderRadius: 8, padding: "8px 12px", cursor: "pointer", fontWeight: 600 });
  const dot = { width: 12, height: 12, borderRadius: "50%", display: "inline-block", marginRight: 8,
    background: armed ? "#f0b429" : listening ? "#2ea043" : "#6e7681",
    boxShadow: armed ? "0 0 10px #f0b429" : "none" };

  return (
    <div style={box}>
      <div style={{ display: "flex", alignItems: "center", justifyContent: "space-between", marginBottom: 12 }}>
        <h3 style={{ margin: 0 }}><span style={dot} />Voice command</h3>
        <span style={{ fontSize: 12, opacity: 0.7 }}>robot: {state}</span>
      </div>

      {supported ? (
        <button style={btn(listening ? "#da3633" : "#1f6feb")} onClick={listening ? stop : start}>
          {listening ? "Stop listening" : "🎙 Start listening"}
        </button>
      ) : (
        <p style={{ color: "#f85149" }}>Speech recognition needs Chrome or Edge. Typed commands still work.</p>
      )}
      <label style={{ marginLeft: 12, fontSize: 13 }}>
        <input type="checkbox" checked={speakInBrowser} onChange={(e) => setSpeakInBrowser(e.target.checked)} /> speak replies here too
      </label>
      <p style={{ fontSize: 12, opacity: 0.7 }}>Say <b>"Optimus, go to the loading bay"</b>, "Optimus, start auto navigation", "Optimus, stop".</p>

      <div style={{ display: "flex", gap: 8, flexWrap: "wrap", margin: "8px 0" }}>
        {QUICK.map(([label, c]) => (
          <button key={c} style={btn(c === "stop" ? "#da3633" : "#30363d")} onClick={() => send(c)}>{label}</button>
        ))}
      </div>

      <form onSubmit={(e) => { e.preventDefault(); send(typed); setTyped(""); }} style={{ display: "flex", gap: 8 }}>
        <input value={typed} onChange={(e) => setTyped(e.target.value)} placeholder="Type a command, e.g. move forward 1 meter"
          style={{ flex: 1, background: "#161b22", color: "#e6edf3", border: "1px solid #30363d", borderRadius: 8, padding: 8 }} />
        <button style={btn("#238636")}>Send</button>
      </form>

      <div style={{ marginTop: 12, maxHeight: 220, overflowY: "auto", fontSize: 13 }}>
        {log.slice().reverse().map((e, i) => (
          <div key={i} style={{ padding: "4px 0", borderBottom: "1px solid #21262d" }}>
            <span style={{ opacity: 0.5 }}>{e.t} </span>
            <b style={{ color: e.who === "OPTIMUS" ? "#58a6ff" : "#7ee787" }}>{e.who}:</b> {e.text}
          </div>
        ))}
      </div>
    </div>
  );
}

// Low, slow browser voice. Browsers can't apply audio effects to speechSynthesis,
// so the full commander effect only plays from the robot laptop.
function speak(text) {
  const u = new SpeechSynthesisUtterance(text);
  const voices = window.speechSynthesis.getVoices();
  u.voice = voices.find((v) => /male|daniel|david|google uk english male/i.test(v.name)) || voices[0];
  u.pitch = 0.3;
  u.rate = 0.88;
  window.speechSynthesis.cancel();
  window.speechSynthesis.speak(u);
}
