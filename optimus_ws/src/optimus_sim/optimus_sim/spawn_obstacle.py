#!/usr/bin/env python3
"""Drop obstacles into the running warehouse.

ros2 run optimus_sim spawn_obstacle --ahead 1.5              # box 1.5 m in front of Optimus
ros2 run optimus_sim spawn_obstacle --ahead 1.2 --kind person
ros2 run optimus_sim spawn_obstacle --x 10 --y 4 --kind crate
ros2 run optimus_sim spawn_obstacle --remove obs_1
ros2 run optimus_sim spawn_obstacle --clear                   # remove everything this tool spawned
"""
import argparse
import json
import math
import os

from .gz_tools import SHAPES, remove, spawn

STATE = os.path.expanduser("~/.optimus_sim_obstacles.json")


def _load():
    try:
        with open(STATE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return []


def _save(names):
    with open(STATE, "w") as f:
        json.dump(names, f)


def robot_pose():
    import time
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
    rclpy.init()
    n = Node("spawn_obstacle_tf")
    n.set_parameters([Parameter("use_sim_time", value=True)])
    buf = Buffer()
    TransformListener(buf, n)
    pose = None
    for frame in ("map", "odom"):
        end = time.monotonic() + 3.0
        while rclpy.ok() and pose is None:
            rclpy.spin_once(n, timeout_sec=0.1)
            try:
                t = buf.lookup_transform(frame, "base_footprint", Time())
                q = t.transform.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                pose = (t.transform.translation.x, t.transform.translation.y, yaw)
            except Exception:
                if time.monotonic() > end:
                    break
        if pose:
            break
    n.destroy_node()
    rclpy.shutdown()
    return pose


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--kind", default="box", choices=sorted(SHAPES))
    ap.add_argument("--ahead", type=float, help="metres in front of the robot")
    ap.add_argument("--side", type=float, default=0.0, help="lateral offset, + = left")
    ap.add_argument("--x", type=float)
    ap.add_argument("--y", type=float)
    ap.add_argument("--name")
    ap.add_argument("--remove")
    ap.add_argument("--clear", action="store_true")
    a = ap.parse_args()

    names = _load()
    if a.clear:
        for n in names:
            print("remove", n, "ok" if remove(n) else "FAILED")
        _save([])
        return
    if a.remove:
        print("remove", a.remove, "ok" if remove(a.remove) else "FAILED")
        _save([n for n in names if n != a.remove])
        return

    if a.ahead is not None:
        pose = robot_pose()
        if pose is None:
            raise SystemExit("could not get robot pose from TF (is the sim running?)")
        x0, y0, yaw = pose
        x = x0 + a.ahead * math.cos(yaw) - a.side * math.sin(yaw)
        y = y0 + a.ahead * math.sin(yaw) + a.side * math.cos(yaw)
    elif a.x is not None and a.y is not None:
        x, y = a.x, a.y
    else:
        raise SystemExit("give --ahead D or --x X --y Y")

    i = 1
    while f"obs_{i}" in names:
        i += 1
    name = a.name or f"obs_{i}"
    ok = spawn(name, a.kind, x, y)
    print(f"spawn {a.kind} '{name}' at ({x:.2f}, {y:.2f})", "ok" if ok else "FAILED")
    if ok:
        _save(names + [name])


if __name__ == "__main__":
    main()
