"""Deep 'commander robot' voice: offline TTS + sox effect chain.

Engine order: Piper (neural, best quality) -> espeak-ng (always available).
Effects: pitch down, bass boost, light overdrive, short metallic echo, reverb.
Speech runs in a background thread; `speaking` is set while audio plays so the
microphone loop can ignore Optimus hearing itself.
"""
import os
import queue
import shutil
import subprocess
import tempfile
import threading

# Effect chain (sox). Tune pitch (cents) and echo delay (ms) to taste.
SOX_FX = [
    "pitch", "-300",
    "bass", "+6",
    "treble", "-2",
    "overdrive", "3", "8",
    "echo", "0.8", "0.7", "9", "0.35",     # short comb -> metallic body
    "reverb", "22", "50", "60",
    "norm", "-1",
]


class CommanderVoice:
    def __init__(self, engine="auto", piper_model="", effects=True, logger=None,
                 rate=0.95):
        self.log = logger
        self.effects = effects and shutil.which("sox") is not None
        self.piper_model = os.path.expanduser(piper_model or "")
        self.rate = rate
        if engine == "auto":
            if shutil.which("piper") and os.path.isfile(self.piper_model):
                engine = "piper"
            else:
                engine = "espeak"
        self.engine = engine
        self.player = next((p for p in ("paplay", "aplay", "ffplay") if shutil.which(p)), None)
        self.speaking = threading.Event()
        self._q = queue.Queue()
        threading.Thread(target=self._worker, daemon=True).start()
        self._info(f"voice engine={self.engine} effects={self.effects} player={self.player}")

    def _info(self, msg):
        if self.log:
            self.log.info(msg)

    def _warn(self, msg):
        if self.log:
            self.log.warning(msg)

    def say(self, text: str, interrupt=False):
        if interrupt:
            while not self._q.empty():
                try:
                    self._q.get_nowait()
                except queue.Empty:
                    break
        self._q.put(text)

    # ---------------------------------------------------------------- internals
    def _synth(self, text, wav):
        if self.engine == "piper":
            cmd = ["piper", "--model", self.piper_model, "--output_file", wav,
                   "--length_scale", str(1.0 / self.rate)]
            subprocess.run(cmd, input=text.encode(), check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            exe = shutil.which("espeak-ng") or shutil.which("espeak")
            if not exe:
                raise RuntimeError("no TTS engine: install espeak-ng or piper")
            subprocess.run([exe, "-v", "en-us+m3", "-s", str(int(140 * self.rate)),
                            "-p", "20", "-w", wav, text], check=True,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _play(self, wav):
        if not self.player:
            self._warn("no audio player (paplay/aplay/ffplay) found")
            return
        cmd = {"paplay": ["paplay", wav], "aplay": ["aplay", "-q", wav],
               "ffplay": ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", wav]}[self.player]
        subprocess.run(cmd, check=False)

    def _worker(self):
        while True:
            text = self._q.get()
            with tempfile.TemporaryDirectory() as d:
                raw, fx = os.path.join(d, "raw.wav"), os.path.join(d, "fx.wav")
                try:
                    self._synth(text, raw)
                    out = raw
                    if self.effects:
                        r = subprocess.run(["sox", raw, fx] + SOX_FX,
                                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                        if r.returncode == 0:
                            out = fx
                    self.speaking.set()
                    self._play(out)
                except Exception as e:  # never kill the worker
                    self._warn(f"TTS failed: {e}")
                finally:
                    self.speaking.clear()


def main():
    """ros2 run optimus_voice say "text" [piper_model.onnx]  - test the voice."""
    import sys
    import time
    text = sys.argv[1] if len(sys.argv) > 1 else \
        "Optimus online. All systems operational. Awaiting your orders."
    model = sys.argv[2] if len(sys.argv) > 2 else "~/piper/en_US-ryan-high.onnx"
    v = CommanderVoice(piper_model=model)
    v.say(text)
    time.sleep(1.5)
    while v.speaking.is_set() or not v._q.empty():
        time.sleep(0.2)


if __name__ == "__main__":
    main()
