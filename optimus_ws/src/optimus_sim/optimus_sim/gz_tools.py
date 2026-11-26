"""Spawn / remove obstacles in the running Gazebo world (via ros_gz_sim + gz CLI)."""
import subprocess

WORLD = "warehouse"

SHAPES = {
    # kind: (sdf geometry, height, colour)
    "box":     ("<box><size>0.5 0.5 0.6</size></box>", 0.6, "0.8 0.1 0.1"),
    "crate":   ("<box><size>0.8 0.8 0.8</size></box>", 0.8, "0.55 0.38 0.2"),
    "pallet":  ("<box><size>1.2 1.0 0.15</size></box>", 0.15, "0.6 0.45 0.25"),  # below scan plane!
    "person":  ("<cylinder><radius>0.25</radius><length>1.7</length></cylinder>", 1.7, "0.9 0.2 0.6"),
    "cone":    ("<cylinder><radius>0.12</radius><length>0.5</length></cylinder>", 0.5, "1.0 0.5 0.0"),
    "wall":    ("<box><size>0.2 2.0 1.2</size></box>", 1.2, "0.6 0.6 0.6"),
}


def obstacle_sdf(name, kind):
    geom, h, c = SHAPES[kind]
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="{name}"><static>true</static>'
            f'<link name="link"><pose>0 0 {h / 2} 0 0 0</pose>'
            f'<collision name="c"><geometry>{geom}</geometry></collision>'
            f'<visual name="v"><geometry>{geom}</geometry><material><ambient>{c} 1</ambient>'
            f'<diffuse>{c} 1</diffuse></material></visual></link></model></sdf>')


def spawn(name, kind, x, y, yaw=0.0):
    cmd = ["ros2", "run", "ros_gz_sim", "create", "-world", WORLD, "-name", name,
           "-string", obstacle_sdf(name, kind), "-x", str(x), "-y", str(y), "-z", "0.0", "-Y", str(yaw)]
    return subprocess.run(cmd, capture_output=True, text=True, timeout=20).returncode == 0


def remove(name):
    cmd = ["gz", "service", "-s", f"/world/{WORLD}/remove", "--reqtype", "gz.msgs.Entity",
           "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", f'name: "{name}" type: MODEL']
    return subprocess.run(cmd, capture_output=True, text=True, timeout=10).returncode == 0
