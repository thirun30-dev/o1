#!/usr/bin/env python3
"""Obstacle-response test runner for Optimus.

ros2 run optimus_sim obstacle_test                    # run every scenario in order
ros2 run optimus_sim obstacle_test --only ambush blocked_aisle
ros2 run optimus_sim obstacle_test --list

For each scenario: drive to the start pose (not scored), then navigate to the
goal while recording how Optimus reacts. Report -> ~/optimus_obstacle_reports/
"""
import argparse
import datetime
import json
import math
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CollisionMonitorState
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from .gz_tools import remove, spawn

# Robot body (base_footprint frame) and LiDAR mount, metres
HALF_L, HALF_W, LIDAR_X = 0.30, 0.20, 0.24   # body half-size, LiDAR forward offset (urdf/optimus.urdf)
NEAR_MISS, CONTACT = 0.10, 0.03

SCENARIOS = [
    dict(name="ambush", start=(0.0, 0.0, 0), goal=(5.5, 0.0, 0),
         desc="A box drops 1.3 m in front of Optimus while it is driving",
         trigger=dict(after_dist=1.5, ahead=("box", 1.3))),
    dict(name="corridor_crossing", start=(5.5, 0.0, 0), goal=(14.5, 0.0, 0),
         desc="A worker walks back and forth across the main corridor"),
    dict(name="gap_or_detour", start=(14.5, 0.0, 0), goal=(18.0, 0.0, 0),
         desc="0.9 m gap between two crates: squeeze through or detour"),
    dict(name="blocked_aisle", start=(13.6, 4.0, 180), goal=(6.8, 4.0, 180),
         desc="Aisle A-B (with a dropped box) gets blocked by a wall: must replan",
         trigger=dict(after_dist=1.0, at=("wall", 9.5, 4.0))),
    dict(name="trolley_pass", start=(15.2, -3.5, 180), goal=(5.8, -3.5, 180),
         desc="Pass the moving trolley in aisle C-D (1.25 m free lane beside it)"),
    dict(name="return_home", start=None, goal=(0.0, 0.0, 0),
         desc="Back to base past pillars, crates and a walking worker"),
]


def body_clearance(px, py):
    dx, dy = max(abs(px) - HALF_L, 0.0), max(abs(py) - HALF_W, 0.0)
    return math.hypot(dx, dy)


class Runner(Node):
    def __init__(self):
        super().__init__("obstacle_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.nav = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.create_subscription(LaserScan, "/scan", self._scan, 10)
        self.create_subscription(Odometry, "/odom", self._odom, 20)
        self.create_subscription(CollisionMonitorState, "/collision_monitor_state", self._cm, 10)
        self.recording = False
        self.speed = 0.0
        self.reset()

    # ---------------------------------------------------------------- metrics
    def reset(self):
        self.m = dict(min_clearance=float("inf"), near_misses=0, contacts=0, cm_stops=0,
                      cm_slowdowns=0, stopped_time=0.0, distance=0.0, reaction_s=None)
        self._near = self._contact = False
        self._last_xy = None
        self._cm_action = 0
        self._cm_since = None
        self.dist_since_start = 0.0
        self.spawn_time = None
        self.speed_at_spawn = 0.0

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan(self, msg):
        if not self.recording:
            return
        best = float("inf")
        for i, r in enumerate(msg.ranges):
            if not (msg.range_min <= r <= msg.range_max):
                continue
            a = msg.angle_min + i * msg.angle_increment
            c = body_clearance(LIDAR_X + r * math.cos(a), r * math.sin(a))
            if c > 0.0:            # c == 0 would be a ray hitting Optimus itself
                best = min(best, c)
        self.m["min_clearance"] = min(self.m["min_clearance"], best)
        near, contact = best < NEAR_MISS, best < CONTACT
        if near and not self._near:
            self.m["near_misses"] += 1
        if contact and not self._contact:
            self.m["contacts"] += 1
        self._near, self._contact = near, contact

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.speed = abs(msg.twist.twist.linear.x)
        if self._last_xy is not None:
            d = math.hypot(p.x - self._last_xy[0], p.y - self._last_xy[1])
            self.dist_since_start += d
            if self.recording:
                self.m["distance"] += d
        self._last_xy = (p.x, p.y)
        self._check_reaction(speed_drop=True)

    def _cm(self, msg):
        if not self.recording:
            return
        a = msg.action_type
        t = self.now()
        if self._cm_action == CollisionMonitorState.STOP and self._cm_since is not None:
            self.m["stopped_time"] += t - self._cm_since
        if a != self._cm_action:
            if a == CollisionMonitorState.STOP:
                self.m["cm_stops"] += 1
            elif a == CollisionMonitorState.SLOWDOWN:
                self.m["cm_slowdowns"] += 1
        self._cm_action, self._cm_since = a, t
        if a in (CollisionMonitorState.STOP, CollisionMonitorState.SLOWDOWN):
            self._check_reaction()

    def _check_reaction(self, speed_drop=False):
        if self.spawn_time is None or self.m["reaction_s"] is not None:
            return
        if speed_drop and not (self.speed_at_spawn > 0.1 and self.speed < 0.5 * self.speed_at_spawn):
            return
        self.m["reaction_s"] = round(self.now() - self.spawn_time, 2)

    # ---------------------------------------------------------------- helpers
    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", Time())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def goal_msg(self, x, y, yaw_deg):
        g = NavigateToPose.Goal()
        g.pose = PoseStamped()
        g.pose.header.frame_id = "map"
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
        yaw = math.radians(yaw_deg)
        g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return g

    def navigate(self, goal, timeout, on_tick=None):
        fut = self.nav.send_goal_async(self.goal_msg(*goal))
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return "rejected", 0.0
        res = gh.get_result_async()
        t0 = time.monotonic()
        while rclpy.ok() and not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if on_tick:
                on_tick()
            if time.monotonic() - t0 > timeout:
                cf = gh.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cf, timeout_sec=5.0)
                return "timeout", time.monotonic() - t0
        status = res.result().status
        name = {GoalStatus.STATUS_SUCCEEDED: "succeeded", GoalStatus.STATUS_ABORTED: "aborted",
                GoalStatus.STATUS_CANCELED: "canceled"}.get(status, f"status_{status}")
        return name, time.monotonic() - t0

    # ---------------------------------------------------------------- scenario
    def run(self, sc, timeout):
        print(f"\n=== {sc['name']}: {sc['desc']}")
        if sc["start"] is not None:
            print(f"    driving to start {sc['start'][:2]} (not scored)...")
            st, _ = self.navigate(sc["start"], timeout)
            if st != "succeeded":
                print(f"    could not reach start ({st}); scoring from here anyway")
        self.reset()
        spawned = []
        trig = sc.get("trigger")
        fired = [False]

        def do_spawn(kind, x, y):
            name = f"test_{sc['name']}"
            if spawn(name, kind, x, y):
                spawned.append(name)
                self.spawn_time = self.now()
                self.speed_at_spawn = self.speed
                print(f"    >>> spawned {kind} at ({x:.1f}, {y:.1f})")
            else:
                print("    !!! spawn failed (is `ros2 run ros_gz_sim create` available?)")

        def tick():
            if not trig or fired[0] or self.dist_since_start < trig["after_dist"]:
                return
            fired[0] = True
            if "ahead" in trig:
                p = self.pose()
                if p is None:
                    return
                kind, d = trig["ahead"]
                args = (kind, p[0] + d * math.cos(p[2]), p[1] + d * math.sin(p[2]))
            else:
                args = trig["at"]
            threading.Thread(target=do_spawn, args=args, daemon=True).start()

        self.recording = True
        status, dur = self.navigate(sc["goal"], timeout, tick)
        self.recording = False
        for n in spawned:
            remove(n)

        m = dict(self.m)
        m["min_clearance"] = None if math.isinf(m["min_clearance"]) else round(m["min_clearance"], 3)
        m.update(scenario=sc["name"], description=sc["desc"], result=status,
                 duration_s=round(dur, 1), distance=round(m["distance"], 2),
                 stopped_time=round(m["stopped_time"], 1))
        if m["contacts"]:
            verdict = "FAIL - likely collision"
        elif status == "succeeded":
            verdict = "PASS" if not m["near_misses"] else "PASS (near miss)"
        else:
            verdict = "SAFE STOP - goal not reached, no contact"
        m["verdict"] = verdict
        print(f"    result={status}  time={m['duration_s']}s  dist={m['distance']}m  "
              f"min clearance={m['min_clearance']}m  stops={m['cm_stops']} slowdowns={m['cm_slowdowns']}  "
              f"reaction={m['reaction_s']}s  -> {verdict}")
        return m


def write_report(results):
    d = os.path.expanduser("~/optimus_obstacle_reports")
    os.makedirs(d, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    with open(os.path.join(d, f"report_{stamp}.json"), "w") as f:
        json.dump(results, f, indent=2)
    lines = ["# Optimus obstacle response report", "", f"Run: {stamp}", "",
             "| Scenario | Result | Time (s) | Dist (m) | Min clearance (m) | CM stops | CM slowdowns | Stopped (s) | Reaction (s) | Verdict |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in results:
        lines.append(f"| {r['scenario']} | {r['result']} | {r['duration_s']} | {r['distance']} | {r['min_clearance']} | "
                     f"{r['cm_stops']} | {r['cm_slowdowns']} | {r['stopped_time']} | {r['reaction_s']} | {r['verdict']} |")
    lines += ["", "Min clearance = closest LiDAR return to the robot body. Near miss < 0.10 m, contact < 0.03 m.",
              "Objects below the LiDAR plane (~0.38 m), e.g. a bare pallet, are invisible to Optimus."]
    path = os.path.join(d, f"report_{stamp}.md")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(["", *lines[4:]]))
    print(f"\nreport saved: {path}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", nargs="*", help="scenario names")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--timeout", type=float, default=120.0, help="wall seconds per navigation")
    a = ap.parse_args()
    if a.list:
        for s in SCENARIOS:
            print(f"{s['name']:18s} {s['desc']}")
        return
    todo = [s for s in SCENARIOS if not a.only or s["name"] in a.only]

    rclpy.init()
    r = Runner()
    print("waiting for Nav2 (navigate_to_pose)...")
    if not r.nav.wait_for_server(timeout_sec=60.0):
        raise SystemExit("Nav2 not available - is warehouse.launch.py running?")
    results = []
    try:
        for s in todo:
            results.append(r.run(s, a.timeout))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        if results:
            write_report(results)
        r.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
