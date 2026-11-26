# SLAM Mapping Guide

## Overview

Mapping uses **SLAM Toolbox** (`async_slam_toolbox_node`) configured in `optimus_bot/config/slam.yaml` to generate high-resolution occupancy grids from simulated or real LiDAR and odometry. The asynchronous mapper keeps scan processing responsive while the robot is moving.

On Humble SLAM Toolbox starts mapping immediately. From Jazzy on it is a lifecycle node; `slam.launch.py` configures and activates it automatically (check with `ros2 lifecycle get /slam_toolbox` → `active`).

---

## 1. Mapping Workflow

```text
Start Robot Stack
       ↓
Click [ MAPPING (SLAM) ] on Web Dashboard
       ↓
Pre-Flight Validation Check:
   - /scan active?
   - /odom active?
   - TF odom -> base_link active?
   - TF base_link -> lidar_link active?
       ↓
SLAM Toolbox Process Launched
       ↓
Drive Robot via Dashboard Teleoperation or WASD
       ↓
Review Generated Map on Interactive Viewer
       ↓
Enter Map Name in Dashboard (e.g., warehouse_main)
       ↓
Click [ SAVE MAP ]
       ↓
Map saved to ~/optimus_data/maps/<name>.yaml and <name>.pgm
       ↓
Click [ SERIALIZE GRAPH ]
       ↓
SLAM pose graph saved as ~/optimus_data/maps/<name>.posegraph and <name>.data
       ↓
Click [ STOP MAPPING ]
```

---

## 2. Pre-Flight Sensor Checks

Before SLAM starts, `amr_system_manager` automatically verifies all requirements:

1. `/scan` received within the last 2 s
2. `/odom` received within the last 2 s
3. TF `odom -> base_link` present
4. TF `base_link -> lidar_link` present

If any prerequisite fails, the state machine rejects the transition and displays an actionable diagnostic error on the web dashboard.

---

## 3. Saving Maps

Maps are stored in `~/optimus_data/maps/` (the `maps_dir` argument of `bringup.launch.py`), outside the workspace so rebuilds never touch them. Maps shipped with the package (`optimus_bot/maps/`) are also listed in the dashboard.

From the Web Dashboard:
- Enter a name in **Map Management** (e.g. `warehouse_zone_1`).
- Click **[ SAVE MAP ]**.
- Click **[ SERIALIZE GRAPH ]** to persist the SLAM Toolbox pose graph for later mapping continuation.

To resume a serialized map, select the map and click **[ RESUME MAPPING ]**. This starts asynchronous SLAM and requests pose-graph deserialization.
- The system executes `map_saver_cli` with proper timeouts, producing both `<name>.yaml` and `<name>.pgm`.

From the Command Line (for debugging):

```bash
ros2 run nav2_map_server map_saver_cli -f ~/optimus_data/maps/warehouse_main --ros-args -p save_map_timeout:=10.0
```
