# Nav2 Autonomous Navigation Guide

## Overview

Autonomous warehouse navigation is implemented using **Nav2** configured specifically for the Optimus robot footprint ($0.60\,\text{m}\times0.40\,\text{m}$) in `optimus_bot/config/nav2.yaml`.

---

## 1. Nav2 Stack Architecture

`navigation.launch.py` starts exactly these nodes under
`lifecycle_manager_navigation` (it does not include nav2_bringup's
`navigation_launch.py`, which from Jazzy on also starts collision monitor,
docking and route servers that Optimus does not configure):

| Node | Role |
|---|---|
| `planner_server` | Global planner: NavFn (A*) on the global costmap |
| `controller_server` | Local controller: RotationShim + DWB, output `/cmd_vel_nav` |
| `smoother_server` | Path smoothing |
| `behavior_server` | Recoveries: Spin, BackUp, DriveOnHeading, Wait |
| `bt_navigator` | Behavior-tree navigator (`/navigate_to_pose`, `/navigate_through_poses`) |
| `waypoint_follower` | `/follow_waypoints` |
| `velocity_smoother` | `/cmd_vel_nav` → `/cmd_vel` (max 0.5 m/s, 1.0 rad/s, reverse 0.2 m/s) |

- **Costmaps**:
  - Global: Static layer + Obstacle layer (/scan) + Inflation layer (radius 0.55 m)
  - Local: Rolling window (4 m × 4 m) + Obstacle layer + Inflation layer
- **Footprint**: Matches the chassis: `"[ [0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2] ]"`
- **Goal tolerance**: 0.15 m, 0.20 rad
- **Plugin names**: `config/nav2.yaml` uses the Humble spellings
  (`nav2_behaviors/Spin`); on newer distros the launch file translates them to
  `nav2_behaviors::Spin` automatically.

---

## 2. Setting Goals from Web Dashboard

### Method A: Interactive Map Click
1. Select **"Set Nav Goal"** on the Map Viewer toolbar.
2. Click (and drag for heading) on the occupancy grid map.
3. The dashboard sends `{"action": "navigate_to_pose", ...}` on `/amr/command`;
   `amr_location_manager` sends one `NavigateToPose` action goal to Nav2.
4. Real-time feedback on `/amr/navigation_status`:
   - Status: `PLANNING` $\to$ `NAVIGATING` $\to$ `SUCCEEDED` / `FAILED` / `CANCELED`
   - Distance remaining (meters)
   - Global path in green on the map

### Method B: Saved Warehouse Locations
1. Open the **POIs** panel (or the waypoint bar) on the dashboard.
2. Find the desired target (e.g. `Shelf 1` or `Charging Station`).
3. Click **[ GO ]**.
4. The location manager dispatches the goal directly to Nav2.

### Method C: Mission route
The **POI Dispatcher** page runs a sequence of saved locations, waiting each
stop's dwell time, optionally in a loop. A failed or canceled leg halts the
mission; **Abort** cancels the active goal.

### RViz / scripts
RViz "Nav2 Goal" publishes `/goal_pose`, which Nav2's BT navigator handles
directly. Scripts can call `/amr/navigate_to_pose` (`optimus_bot/srv/NavigateToPose`).

---

## 3. Canceling or Pausing Navigation

- Click **Cancel goal**, **[ STOP NAVIGATION ]** in the Mode Selector, or the Emergency Stop button.
- The active goal is canceled via the Nav2 action interface (also when it has
  not been accepted yet).
