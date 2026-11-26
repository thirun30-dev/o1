#!/usr/bin/env python3
"""OPTIMUS voice command node.

Inputs
  * Laptop microphone  -> offline Vosk STT -> wake word "Optimus" -> command
  * /optimus/voice/text_command (std_msgs/String) from the web dashboard
    (wake word optional there; the browser already checks it)

Outputs
  * Nav2 actions: NavigateToPose, FollowWaypoints, Spin, DriveOnHeading
  * /optimus/voice/transcript, /optimus/voice/response, /optimus/voice/state
  * Spoken reply in the commander voice
"""
import json
import math
import os
import queue
import threading

import rclpy
import rclpy.duration
import yaml
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav2_msgs.action import DriveOnHeading, FollowWaypoints, NavigateToPose, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from .commander_voice import CommanderVoice
from .intent_parser import IntentParser, split_wake_word
from .lines import line


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OptimusVoiceNode(Node):
    def __init__(self):
        super().__init__("optimus_voice")
        p = self.declare_parameter
        p("enable_mic", True)
        p("vosk_model_path", "~/vosk-model-small-en-in-0.4")
        p("mic_device", -1)                 # -1 = system default
        p("sample_rate", 16000)
        p("listen_window_sec", 6.0)         # after a bare "Optimus"
        p("require_wake_word_for_text", False)
        p("waypoints_file", "")
        p("home_waypoint", "home")
        p("map_frame", "map")
        p("base_frame", "base_link")
        p("cmd_vel_topic", "/cmd_vel")
        p("battery_topic", "/amr/battery_state")  # published by amr_system_manager
        p("drive_speed", 0.15)              # m/s for relative moves
        p("tts_engine", "auto")             # auto | piper | espeak
        p("piper_model", "~/piper/en_US-ryan-high.onnx")
        p("voice_effects", True)
        p("speech_rate", 0.95)
        gp = lambda n: self.get_parameter(n).value  # noqa: E731

        self.map_frame, self.base_frame = gp("map_frame"), gp("base_frame")
        self.drive_speed = gp("drive_speed")
        self.home_name = gp("home_waypoint")
        self.require_wake_text = gp("require_wake_word_for_text")
        self.listen_window = gp("listen_window_sec")

        self.waypoints_file = os.path.expanduser(gp("waypoints_file"))
        self.waypoints = self._load_waypoints()
        self.parser = IntentParser(self.waypoints)

        self.voice = CommanderVoice(gp("tts_engine"), gp("piper_model"),
                                    gp("voice_effects"), self.get_logger(), gp("speech_rate"))

        self.pub_transcript = self.create_publisher(String, "/optimus/voice/transcript", 10)
        self.pub_response = self.create_publisher(String, "/optimus/voice/response", 10)
        self.pub_state = self.create_publisher(String, "/optimus/voice/state", 10)
        self.pub_cmd = self.create_publisher(Twist, gp("cmd_vel_topic"), 10)
        self.create_subscription(String, "/optimus/voice/text_command", self._on_text, 10)
        self.battery = None
        self.create_subscription(BatteryState, gp("battery_topic"), self._on_battery, 10)

        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.follow = ActionClient(self, FollowWaypoints, "follow_waypoints")
        self.spin = ActionClient(self, Spin, "spin")
        self.drive = ActionClient(self, DriveOnHeading, "drive_on_heading")
        self.goal_handle = None
        self.goal_seq = 0               # ignore results of preempted goals
        self.patrol_loop = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.commands = queue.Queue()   # mic thread -> executor thread
        self.create_timer(0.1, self._drain_commands)
        self.armed_until = None

        if gp("enable_mic"):
            threading.Thread(target=self._mic_loop, args=(
                os.path.expanduser(gp("vosk_model_path")), gp("sample_rate"),
                gp("mic_device")), daemon=True).start()

        self._state("idle")
        self._reply(line("online"))

    # ------------------------------------------------------------ waypoints
    def _load_waypoints(self):
        self.patrol_order = []
        if not self.waypoints_file or not os.path.isfile(self.waypoints_file):
            self.get_logger().warn(f"waypoints file not found: {self.waypoints_file!r}")
            return {}
        with open(self.waypoints_file) as f:
            data = yaml.safe_load(f) or {}
        wps = data.get("waypoints", {})
        self.patrol_order = data.get("patrol", [n for n in wps if n != self.home_name])
        self.get_logger().info(f"loaded {len(wps)} waypoints: {', '.join(wps)}")
        return wps

    def _save_waypoints(self):
        if not self.waypoints_file:
            return
        with open(self.waypoints_file, "w") as f:
            yaml.safe_dump({"waypoints": self.waypoints, "patrol": self.patrol_order},
                           f, sort_keys=False)

    def _pose_of(self, name):
        wp = self.waypoints[name]
        ps = PoseStamped()
        ps.header.frame_id = self.map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = float(wp["x"]), float(wp["y"])
        q = yaw_to_quat(math.radians(float(wp.get("yaw_deg", 0.0))))
        (ps.pose.orientation.x, ps.pose.orientation.y,
         ps.pose.orientation.z, ps.pose.orientation.w) = q
        return ps

    def _current_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except Exception:
            return None
        tr = t.transform
        return tr.translation.x, tr.translation.y, quat_to_yaw(tr.rotation)

    def _places(self):
        return ", ".join(n.replace("_", " ") for n in self.waypoints) or "none"

    # ------------------------------------------------------------ I/O helpers
    def _state(self, s):
        self.pub_state.publish(String(data=s))

    def _reply(self, text):
        self.get_logger().info(f"OPTIMUS: {text}")
        self.pub_response.publish(String(data=text))
        self.voice.say(text)

    def _on_battery(self, msg):
        self.battery = msg

    def _on_text(self, msg):
        text = msg.data.strip()
        woke, rest = split_wake_word(text)
        if self.require_wake_text and not woke:
            return
        self.commands.put(("web", rest if woke else text))

    # ------------------------------------------------------------ microphone
    def _mic_loop(self, model_path, rate, device):
        try:
            import sounddevice as sd
            import vosk
        except ImportError as e:
            self.get_logger().error(f"mic disabled, missing package: {e}. "
                                    "pip install vosk sounddevice")
            return
        if not os.path.isdir(model_path):
            self.get_logger().error(f"Vosk model not found at {model_path}")
            return
        vosk.SetLogLevel(-1)
        rec = vosk.KaldiRecognizer(vosk.Model(model_path), rate)
        audio = queue.Queue()

        def cb(indata, frames, t, status):
            if not self.voice.speaking.is_set():
                audio.put(bytes(indata))

        dev = None if device < 0 else device
        self.get_logger().info("microphone live - say 'Optimus, ...'")
        with sd.RawInputStream(samplerate=rate, blocksize=4000, dtype="int16",
                               channels=1, device=dev, callback=cb):
            was_speaking = False
            while rclpy.ok():
                try:
                    data = audio.get(timeout=0.5)
                except queue.Empty:
                    continue
                speaking = self.voice.speaking.is_set()
                if was_speaking and not speaking:
                    rec.Reset()
                was_speaking = speaking
                if rec.AcceptWaveform(data):
                    text = json.loads(rec.Result()).get("text", "")
                    if text:
                        self._heard(text)

    def _heard(self, text):
        now = self.get_clock().now()
        woke, rest = split_wake_word(text)
        armed = self.armed_until is not None and now < self.armed_until
        if woke and rest:
            self.commands.put(("mic", rest))
        elif woke:
            self.armed_until = now + rclpy.duration.Duration(seconds=self.listen_window)
            self._state("listening")
            self._reply(line("wake"))
        elif armed:
            self.armed_until = None
            self.commands.put(("mic", text))
        # otherwise: background speech, ignore

    # ------------------------------------------------------------ dispatch
    def _drain_commands(self):
        while not self.commands.empty():
            src, text = self.commands.get()
            self.pub_transcript.publish(String(data=f"[{src}] {text}"))
            self.get_logger().info(f"heard ({src}): {text}")
            self._execute(self.parser.parse(text))

    def _execute(self, it):
        k, a = it.kind, it.args
        if k == "stop":
            self._stop()
            self._reply(line("stop"))
        elif k == "goto":
            if not a.get("name"):
                self._reply(line("unknown_place", spoken=a.get("spoken", ""), places=self._places()))
            else:
                self._go_to(a["name"])
        elif k == "home":
            if self.home_name in self.waypoints:
                self._reply(line("home"))
                self._go_to(self.home_name, announce=False)
            else:
                self._reply(f"No {self.home_name} waypoint is defined.")
        elif k == "patrol":
            self._patrol(a.get("loop", False))
        elif k == "move":
            self._move(a["distance"])
        elif k == "turn":
            self._turn(a["degrees"])
        elif k == "status":
            self._status()
        elif k == "battery":
            b = self.battery
            if b is None:
                self._reply(line("battery_none"))
            else:
                pct = b.percentage * 100 if b.percentage <= 1.0 else b.percentage
                self._reply(line("battery", pct=round(pct), volt=round(b.voltage, 1)))
        elif k == "list":
            self._reply(line("list", places=self._places()))
        elif k == "save":
            self._save_here(a["name"])
        elif k == "greet":
            self._reply(line("greet"))
        else:
            self._reply(line("unknown"))
        if k not in ("goto", "home", "patrol", "move", "turn"):
            self._state("idle")

    # ------------------------------------------------------------ actions
    def _send(self, client, goal, on_done, label):
        if not client.wait_for_server(timeout_sec=1.0):
            self._reply(line("not_ready"))
            self._state("idle")
            return
        self.goal_seq += 1
        seq = self.goal_seq
        self._state(f"executing:{label}")

        def accepted(fut):
            gh = fut.result()
            if seq != self.goal_seq:
                # "stop" (or a newer command) arrived before Nav2 accepted
                # this goal; _stop() had no handle to cancel, so cancel here
                # or the robot would start moving after being told to stop.
                if gh is not None and gh.accepted:
                    gh.cancel_goal_async()
                return
            if gh is None or not gh.accepted:
                self._reply(line("failed", name=label))
                self._state("idle")
                return
            self.goal_handle = gh
            gh.get_result_async().add_done_callback(
                lambda f: self._finished(seq, f.result().status, on_done))
        client.send_goal_async(goal).add_done_callback(accepted)

    def _finished(self, seq, status, on_done):
        if seq != self.goal_seq:          # superseded by a newer command
            return
        self.goal_handle = None
        self._state("idle")
        if status != GoalStatus.STATUS_CANCELED:
            on_done(status == GoalStatus.STATUS_SUCCEEDED)

    def _go_to(self, name, announce=True):
        if announce:
            self._reply(line("goto", name=name.replace("_", " ")))
        g = NavigateToPose.Goal()
        g.pose = self._pose_of(name)
        spoken = name.replace("_", " ")
        self._send(self.nav, g, lambda ok: self._reply(
            line("arrived" if ok else "failed", name=spoken)), name)

    def _patrol(self, loop):
        order = [n for n in self.patrol_order if n in self.waypoints]
        if not order:
            self._reply("No patrol route is defined.")
            return
        self.patrol_loop = loop
        self._reply(line("patrol", n=len(order)))
        g = FollowWaypoints.Goal()
        g.poses = [self._pose_of(n) for n in order]

        def done(ok):
            if self.patrol_loop and ok:
                self._reply(line("patrol_loop"))
                self._patrol(True)
            else:
                self._reply(line("patrol_done") if ok else line("failed", name="the patrol route"))
        self._send(self.follow, g, done, "patrol")

    def _move(self, dist):
        self._reply(line("move", dir="forward" if dist >= 0 else "back", dist=abs(round(dist, 2))))
        g = DriveOnHeading.Goal()
        g.target = Point(x=float(dist), y=0.0, z=0.0)
        g.speed = float(math.copysign(self.drive_speed, dist))
        g.time_allowance = Duration(sec=int(abs(dist) / self.drive_speed * 2) + 5)
        self._send(self.drive, g, lambda ok: self._reply(
            line("done") if ok else line("failed", name="that position")), "move")

    def _turn(self, deg):
        self._reply(line("turn", dir="left" if deg >= 0 else "right", deg=abs(round(deg))))
        g = Spin.Goal()
        g.target_yaw = math.radians(deg)
        g.time_allowance = Duration(sec=int(abs(deg) / 30) + 8)
        self._send(self.spin, g, lambda ok: self._reply(
            line("done") if ok else line("failed", name="that heading")), "turn")

    def _stop(self):
        self.goal_seq += 1
        self.patrol_loop = False
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
        for _ in range(3):
            self.pub_cmd.publish(Twist())
        self.armed_until = None

    def _status(self):
        pose = self._current_pose()
        if pose is None:
            self._reply(line("status_none"))
            return
        x, y, _ = pose
        near = min(self.waypoints, default="nowhere known",
                   key=lambda n: math.hypot(self.waypoints[n]["x"] - x, self.waypoints[n]["y"] - y))
        self._reply(line("status_pose", x=round(x, 1), y=round(y, 1), near=near.replace("_", " ")))

    def _save_here(self, name):
        pose = self._current_pose()
        if pose is None:
            self._reply(line("save_fail"))
            return
        x, y, yaw = pose
        self.waypoints[name] = {"x": round(x, 3), "y": round(y, 3),
                                "yaw_deg": round(math.degrees(yaw), 1), "aliases": []}
        if name != self.home_name and name not in self.patrol_order:
            self.patrol_order.append(name)
        self.parser.set_waypoints(self.waypoints)
        self._save_waypoints()
        self._reply(line("saved", name=name.replace("_", " ")))


def main():
    rclpy.init()
    node = OptimusVoiceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
