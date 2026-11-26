#!/usr/bin/env python3
"""Generate worlds/warehouse.sdf (Gazebo Harmonic) and a top-down preview PNG.

Edit LAYOUT below, then:  python3 tools/generate_world.py [--png docs/warehouse_map.png]
World frame == SLAM map frame because Optimus spawns at (0, 0, yaw 0).
Floor: x -4..20, y -8..8 (24 m x 16 m).
"""
import argparse
import os

HERE = os.path.dirname(os.path.abspath(__file__))
WALK = "https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae"

# name, x, y, size_x, size_y, height, colour(r g b), kind
LAYOUT = {
    "walls": [
        ("wall_west", -4.0, 0.0, 0.2, 16.0, 3.0),
        ("wall_east", 20.0, 0.0, 0.2, 16.0, 3.0),
        ("wall_north", 8.0, 8.0, 24.0, 0.2, 3.0),
        ("wall_south", 8.0, -8.0, 24.0, 0.2, 3.0),
    ],
    # storage racks: 8 m long, 0.9 m deep, aisles 2.1 m, main corridor 4.1 m
    "racks": [
        ("rack_A", 10.0, 5.5, 8.0, 0.9, 2.5),
        ("rack_B", 10.0, 2.5, 8.0, 0.9, 2.5),
        ("rack_C", 10.0, -2.5, 8.0, 0.9, 2.5),
        ("rack_D", 10.0, -5.5, 8.0, 0.9, 2.5),
    ],
    # static clutter in the open floor
    "crates": [
        ("crate_1", 3.0, 1.5, 0.8, 0.8, 0.8),
        ("crate_2", 3.6, -1.8, 1.0, 0.8, 0.6),
        ("pallet_stack_1", 19.2, 6.8, 1.2, 1.0, 1.2),
        ("pallet_stack_2", 19.2, 3.2, 1.2, 1.0, 1.2),
        ("gap_left", 16.5, 0.75, 0.6, 0.6, 1.0),    # 0.9 m gap between these two
        ("gap_right", 16.5, -0.75, 0.6, 0.6, 1.0),
        ("packing_table", 2.0, -7.0, 2.4, 0.8, 0.9),
        ("dropped_box", 12.5, 4.0, 0.5, 0.5, 0.4),  # partly blocks aisle A-B
    ],
    "pillars": [  # name, x, y, radius, height
        ("pillar_1", 5.0, 4.5, 0.25, 3.0),
        ("pillar_2", 5.0, -4.5, 0.25, 3.0),
        ("pillar_3", 16.0, 4.5, 0.25, 3.0),
        ("pillar_4", 16.0, -4.5, 0.25, 3.0),
    ],
    # walking people (visual only: seen by LiDAR, no physical collision)
    "actors": [
        ("worker_corridor", [(10.0, -1.8), (10.0, 1.8)], 0.6),   # crosses main corridor
        ("worker_front", [(1.0, -4.0), (6.0, -4.0)], 0.7),       # walks across open floor
    ],
    # physical moving trolley driven by optimus_sim/traffic node (aisle C-D)
    "trolley": ("trolley", 6.5, -4.5, 0.6, 0.6, 0.9),   # south lane of aisle C-D
}

COL = {"wall": "0.75 0.75 0.72", "rack": "0.85 0.45 0.10", "crate": "0.55 0.38 0.20",
       "pillar": "0.9 0.85 0.1", "trolley": "0.1 0.4 0.85", "table": "0.4 0.4 0.45"}


def box_model(name, x, y, sx, sy, h, colour, static=True, extra=""):
    return f"""
    <model name="{name}">
      <static>{'true' if static else 'false'}</static>
      <pose>{x} {y} {h / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx} {sy} {h}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{sx} {sy} {h}</size></box></geometry>
          <material><ambient>{colour} 1</ambient><diffuse>{colour} 1</diffuse></material></visual>
      </link>{extra}
    </model>"""


def pillar_model(name, x, y, r, h):
    c = COL["pillar"]
    return f"""
    <model name="{name}"><static>true</static><pose>{x} {y} {h / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>
        <visual name="visual"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>
          <material><ambient>{c} 1</ambient><diffuse>{c} 1</diffuse></material></visual>
      </link>
    </model>"""


def actor(name, pts, speed):
    (x0, y0), (x1, y1) = pts
    import math
    d = math.hypot(x1 - x0, y1 - y0)
    yaw = math.atan2(y1 - y0, x1 - x0)
    back = yaw + math.pi
    t = d / speed
    wps = [(0, x0, y0, yaw), (t, x1, y1, yaw), (t + 1.0, x1, y1, back),
           (2 * t + 1.0, x0, y0, back), (2 * t + 2.0, x0, y0, yaw)]
    w = "".join(f"<waypoint><time>{tt:.2f}</time><pose>{px} {py} 1.0 0 0 {pyaw:.3f}</pose></waypoint>"
                for tt, px, py, pyaw in wps)
    return f"""
    <actor name="{name}">
      <skin><filename>{WALK}</filename><scale>1.0</scale></skin>
      <animation name="walking"><filename>{WALK}</filename><scale>1.0</scale><interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><delay_start>0</delay_start><auto_start>true</auto_start>
        <trajectory id="0" type="walking">{w}</trajectory>
      </script>
    </actor>"""


def trolley(name, x, y, sx, sy, h):
    plugins = """
      <plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl"/>
      <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
        <dimensions>2</dimensions><odom_publish_frequency>20</odom_publish_frequency>
      </plugin>"""
    m = box_model(name, x, y, sx, sy, h, COL["trolley"], static=False, extra=plugins)
    # light, frictionless contact so the velocity controller glides it
    return m.replace('<collision name="collision">',
                     '<inertial><mass>15</mass></inertial>\n        <collision name="collision">'
                     '<surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface>')


def world_sdf():
    L = LAYOUT
    parts = []
    parts += [box_model(n, x, y, sx, sy, h, COL["wall"]) for n, x, y, sx, sy, h in L["walls"]]
    parts += [box_model(n, x, y, sx, sy, h, COL["rack"]) for n, x, y, sx, sy, h in L["racks"]]
    parts += [box_model(n, x, y, sx, sy, h, COL["table"] if "table" in n else COL["crate"])
              for n, x, y, sx, sy, h in L["crates"]]
    parts += [pillar_model(*p) for p in L["pillars"]]
    parts += [actor(*a) for a in L["actors"]]
    parts.append(trolley(*L["trolley"]))
    return f"""<?xml version="1.0"?>
<!-- GENERATED by tools/generate_world.py - edit LAYOUT there, not here -->
<sdf version="1.9">
  <world name="warehouse">
    <physics name="1ms" type="ignored"><max_step_size>0.002</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.75 0.8 1</background><shadows>false</shadows></scene>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 15 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.3 0.2 -1</direction></light>
    <model name="floor"><static>true</static>
      <link name="link">
        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry></collision>
        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>40 30</size></plane></geometry>
          <material><ambient>0.35 0.36 0.38 1</ambient><diffuse>0.35 0.36 0.38 1</diffuse></material></visual>
      </link>
    </model>
    {''.join(parts)}
  </world>
</sdf>
"""


def preview(path, waypoints=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, FancyArrowPatch, Rectangle
    L = LAYOUT
    fig, ax = plt.subplots(figsize=(12, 8.4))
    def rect(x, y, sx, sy, c, label=None):
        ax.add_patch(Rectangle((x - sx / 2, y - sy / 2), sx, sy, color=c))
        if label:
            ax.text(x, y, label, ha="center", va="center", fontsize=7, color="white", weight="bold")
    for n, x, y, sx, sy, h in L["walls"]:
        rect(x, y, sx, sy, "#555")
    for n, x, y, sx, sy, h in L["racks"]:
        rect(x, y, sx, sy, "#d9731a", n.replace("_", " "))
    for n, x, y, sx, sy, h in L["crates"]:
        rect(x, y, sx, sy, "#666b73" if "table" in n else "#8c6033")
    for n, x, y, r, h in L["pillars"]:
        ax.add_patch(Circle((x, y), r, color="#e6d21a"))
    for n, pts, s in L["actors"]:
        (x0, y0), (x1, y1) = pts
        ax.add_patch(FancyArrowPatch((x0, y0), (x1, y1), arrowstyle="<->", mutation_scale=14, color="#d62728", lw=2, ls="--"))
        ax.text((x0 + x1) / 2 + 0.2, (y0 + y1) / 2 + 0.3, "walking person", color="#d62728", fontsize=8)
    n, x, y, sx, sy, h = L["trolley"]
    rect(x, y, sx, sy, "#1f66d9")
    ax.add_patch(FancyArrowPatch((x, y - 0.6), (x + 8, y - 0.6), arrowstyle="<->", mutation_scale=14, color="#1f66d9", lw=2))
    ax.text(x + 3, y - 1.1, "moving trolley (physical)", color="#1f66d9", fontsize=8)
    ax.text(16.9, 0, "0.9 m\ngap", fontsize=7, va="center")
    ax.text(12.5, 3.55, "dropped box", fontsize=7, ha="center")
    for name, wp in (waypoints or {}).items():
        ax.plot(wp["x"], wp["y"], marker="*", ms=16, color="#2ca02c", mec="k")
        ax.text(wp["x"] + 0.25, wp["y"] + 0.3, name, fontsize=9, color="#145214", weight="bold")
    ax.add_patch(Rectangle((-0.3, -0.2), 0.6, 0.4, color="#111"))
    ax.text(0, -0.6, "Optimus spawn", ha="center", fontsize=8)
    ax.set_xlim(-4.5, 20.5); ax.set_ylim(-8.5, 8.5); ax.set_aspect("equal")
    ax.set_xticks(range(-4, 21, 2)); ax.set_yticks(range(-8, 9, 2)); ax.grid(alpha=0.25)
    ax.set_title("Optimus warehouse test world (top view, metres, map frame)")
    fig.tight_layout(); fig.savefig(path, dpi=130)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(HERE, "..", "worlds", "warehouse.sdf"))
    ap.add_argument("--png", default="")
    a = ap.parse_args()
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    with open(a.out, "w") as f:
        f.write(world_sdf())
    print("wrote", os.path.abspath(a.out))
    if a.png:
        import yaml
        with open(os.path.join(HERE, "..", "config", "sim_waypoints.yaml")) as f:
            wps = yaml.safe_load(f)["waypoints"]
        os.makedirs(os.path.dirname(a.png) or ".", exist_ok=True)
        preview(a.png, wps)
        print("wrote", os.path.abspath(a.png))
