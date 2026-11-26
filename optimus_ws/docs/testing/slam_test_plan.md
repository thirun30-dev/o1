# SLAM Test Plan

SLAM (Simultaneous Localization and Mapping) merges LiDAR and Odometry to create a static 2D map.

## Procedure
1. Launch hardware and LiDAR.
2. Launch SLAM Toolbox:
   ```bash
   ros2 launch optimus_bot slam.launch.py
   ros2 lifecycle get /slam_toolbox   # Jazzy: must be "active" (Humble: not a lifecycle node)
   ```
3. Open RViz2 and add the Map display targeting `/map`.
4. Teleop the robot slowly around the environment.
5. **Validation**: SLAM marks a cell only after it has been seen in at least two scans, and adds a scan only after the robot moves 0.05 m or turns 0.05 rad, so the map stays nearly empty until the robot moves. The map should then build cleanly without "smearing". Smearing typically indicates a poorly calibrated encoder CPR (Odometry issue) or incorrect `wheel_separation` kinematics.
6. Save the map:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/optimus_data/maps/my_map
   ```
