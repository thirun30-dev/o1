# optimus_depth

Depth camera support for the Optimus AMR: Intel RealSense (D435 / D435i /
D455) and Orbbec (Astra / Gemini).

```bash
ros2 launch optimus_bot bringup.launch.py depth_camera:=realsense   # with the robot
ros2 launch optimus_depth depth_camera.launch.py depth_camera:=orbbec  # on its own
```

| Piece | What it does |
|---|---|
| `config/depth_camera.yaml` | Driver profiles and the camera **mount** (measure it!) plus detector settings |
| `launch/depth_camera.launch.py` | Driver (node `depth_camera`), static TF `base_link -> depth_camera_link`, `depth_obstacle_node` |
| `depth_obstacle_node` | Depth image -> `/depth_camera/obstacles` + `/depth_camera/clearing` (PointCloud2, base_link) |

`depth_obstacle_node` marks everything between 4 cm and 80 cm above the floor
(low pallets, forks, table tops and shelf edges the LiDAR scans under), and
anything more than 8 cm **below** the floor, i.e. drop-offs such as a dock
edge. Nav2 uses both clouds in `depth_layer` (optimus_bot `config/nav2.yaml`).

Install the driver for your camera:

```bash
sudo apt install ros-$ROS_DISTRO-realsense2-camera   # RealSense (also via rosdep)
sudo apt install ros-$ROS_DISTRO-orbbec-camera       # Orbbec
```

The colour and depth images also appear in the web dashboard
(Cockpit -> CAMERA). See `docs/CAMERA.md`.
