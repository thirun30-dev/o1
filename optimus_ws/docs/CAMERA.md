# Cameras: Obstacle Avoidance and Dashboard View

The LiDAR scans one plane about 0.34 m above the floor, so it can miss
anything lower: pallets, boxes, cables, forklift forks, feet. A forward camera
covers that gap. Its detections go into Nav2's costmaps as a second obstacle
source, and the planner and controller steer around them.

| Phase | Camera | Command |
|---|---|---|
| Now | Robot laptop's built-in webcam | `camera:=laptop` (default) |
| Later | Raspberry Pi camera module | `camera:=picam` |
| Off | - | `camera:=none` |

```bash
ros2 launch optimus_bot bringup.launch.py                   # laptop webcam
ros2 launch optimus_bot bringup.launch.py camera:=picam     # Pi camera
ros2 launch optimus_bot camera.launch.py camera:=laptop     # camera stack on its own
```

## How it works

```text
 laptop webcam ── camera_publisher_node.py (OpenCV/V4L2) ─┐
                                                          ├─► /camera/image_raw ─► camera_obstacle_node.py ─► /camera/scan (LaserScan)
 Pi camera ────── camera_node (camera_ros / libcamera) ───┘                                │                         │
                                                                                           ▼                         ▼
                                                                          /camera/obstacle_debug        Nav2 local + global costmap
                                                                                                         "camera_layer"
```

`camera_obstacle_node` needs no depth camera and no neural network, so it is
light enough for a Pi:

1. **Floor model.** A patch at the bottom-centre of the image (blue box in
   the debug view) is assumed to be floor. The node learns its colour and
   brightness and keeps updating them slowly. Any pixel whose colour is rare
   on the floor counts as an obstacle.
2. **Ground-plane geometry.** With the camera height and downward tilt known,
   every image row maps to a distance on the floor. In each column, the lowest
   obstacle pixel is where the object touches the floor, which gives its
   distance and bearing.
3. These hits become a LaserScan in frame `camera_scan_link`. Rays with no
   obstacle are `+inf`, so Nav2 also clears space the camera sees as free.

The camera gets its own costmap layer (`camera_layer` in `config/nav2.yaml`),
so camera clearing never erases LiDAR obstacles.

## Setup (do this for each camera)

All settings are in **`config/camera.yaml`**. Each profile holds the driver,
resolution, field of view and **mount**. `camera.launch.py` publishes the
mount as TF (`base_link → camera_link → camera_optical_frame`, plus the level
`camera_scan_link`). Nothing in the URDF needs changing.

1. **Mount the camera** facing forward and tilted down, so the floor from
   about 0.5 m to 2 m fills most of the image. A level camera only sees floor
   from about 1.3 m away. For the laptop, tilt the lid forward.
2. **Measure** and enter `z_above_floor` (lens height above the floor),
   `pitch_deg` (downward tilt) and `x`, `y` (lens position from the robot
   centre). Obstacle distances come straight from these numbers.
3. **Check the result.** Run the camera stack and view the debug image:

   ```bash
   ros2 launch optimus_bot camera.launch.py camera:=laptop
   ros2 run rqt_image_view rqt_image_view /camera/obstacle_debug
   ```

   Put a box on the floor 1.0 m in front of the camera. Its bottom edge (green
   dots) should sit on the `1.0m` guide line. If it sits too close, increase
   `pitch_deg`; if too far, decrease it. Red shading marks pixels classified
   as "not floor".
4. In RViz, add `/camera/scan` (LaserScan) next to `/scan`, and check that the
   box shows up at the right spot in the local costmap.

Start the robot on **clear floor**: the first frame teaches the floor model.

## Switching to the Raspberry Pi camera

1. Install the driver on the Pi: `sudo apt install ros-$ROS_DISTRO-camera-ros`
   (or `rosdep install --from-paths src --ignore-src -y`).
2. Check that libcamera sees the camera:
   `ros2 run camera_ros camera_node --ros-args -p camera:=0`.
   It should log the sensor (imx219, imx708, ...). On a **Pi 5**, the upstream
   libcamera in the ROS packages may not support the PiSP pipeline yet. If no
   camera is found, build the [Raspberry Pi libcamera fork](https://github.com/raspberrypi/libcamera)
   and camera_ros from source.
3. Enter the Pi camera mount and `hfov_deg` in the `picam` profile
   (Camera Module v2 = 62.2°, Module 3 = 66°, Module 3 Wide = 102°). Set
   `rotate_180: true` if the camera is mounted upside down.
4. Start with `camera:=picam` (`optimus camera:=picam`). For the boot service,
   append `camera:=picam` to `ExecStart` in `tools/optimus-bringup.service`.
5. Repeat the 1.0 m box check above.

No Nav2 or detector settings change. Both drivers publish the same topics.

## Tuning (`camera_obstacle_node` in config/camera.yaml)

| Parameter | Effect |
|---|---|
| `max_range` | How far the camera reports (default 2.0 m). Accuracy drops with distance. |
| `obstacle_threshold` | Higher = more pixels count as obstacles (more sensitive, more false alarms). |
| `saturation_min` | Below this saturation, hue is ignored. Raise it if a grey floor shows noise. |
| `min_run_px` | Minimum vertical obstacle size in pixels; raise to reject floor marks. |
| `bottom_crop_ratio` | Ignore the image bottom, e.g. if the robot's own body or bumper is visible. |
| `learn_rate`, `relearn_after_s` | How fast the floor model follows lighting and floor-type changes. |

## Limits

- Objects the **same colour as the floor** are invisible to it.
- Shadows, glare and painted floor lines can show up as obstacles. Use
  `obstacle_threshold`, `min_run_px` and the floor-line colour to tune.
- The floor reference patch, roughly the nearest 0.5 m straight ahead, is
  assumed free while the floor model is learning. Once the model is learned,
  an object that enters the patch is still detected.
- It is a supplement to the LiDAR, not a safety device. The hardware stop
  button is still the safety system.

---

## Depth camera (optimus_depth)

A depth camera measures distance directly, so it does not depend on the floor
colour and also sees **drop-offs** (dock edges, stairs). The `optimus_depth`
package supports Intel RealSense and Orbbec cameras:

```bash
sudo apt install ros-$ROS_DISTRO-realsense2-camera        # or ros-$ROS_DISTRO-orbbec-camera
ros2 launch optimus_bot bringup.launch.py depth_camera:=realsense   # default: none
```

1. Measure the mount into `src/optimus_depth/config/depth_camera.yaml`
   (`x`, `y`, `z_above_floor`, `pitch_deg`).
2. Check it in RViz: `/depth_camera/obstacles` should show a box on the floor
   at the right distance, with no points on bare floor. If the floor shows up,
   raise `min_obstacle_height` or correct `z_above_floor` / `pitch_deg`.

`depth_obstacle_node` publishes two clouds in `base_link`: obstacles (4-80 cm
above the floor, plus anything 8 cm or more below it) for marking, and every
point for clearing. Nav2 uses them in `depth_layer`. The RGB camera, depth
camera and LiDAR can all run together.

## Camera view in the web dashboard

**Cockpit -> CAMERA** shows the robot's cameras live, on both the local and
the Vercel (wss://) dashboard:

| Feed | Source |
|---|---|
| Front | `/camera/image_raw` (laptop webcam / Pi camera) |
| Obstacles | `/camera/obstacle_debug` (what the obstacle detector sees) |
| Depth RGB | depth camera colour image |
| Depth | depth image, colourised: red = near, blue = 4 m, black = no reading |

`camera_stream_node` (started by bringup together with rosbridge) re-encodes
each feed as a small JPEG (480 px wide, 8 fps max) on
`/dashboard/camera/<feed>/compressed`. A feed is only encoded while a
dashboard is watching it, so a closed tab costs no CPU or Wi-Fi. Size, frame
rate and quality are the node's `max_width`, `fps` and `jpeg_quality`
parameters. Feeds whose camera is not running are greyed out, and the panel
shows which launch argument starts them. The panel also has Pause, Snapshot
(saves a JPEG) and Full screen buttons.
