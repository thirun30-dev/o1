# Navigation Test Plan (Nav2)

Once SLAM is functional and a map is saved, Nav2 can execute autonomous trajectory planning.

## Procedure
1. Launch the robot hardware and LiDAR.
2. Launch localization and navigation (or use the dashboard modes):
   ```bash
   ros2 launch optimus_bot localization.launch.py map:=$HOME/optimus_data/maps/<name>.yaml
   ros2 launch optimus_bot navigation.launch.py
   ```
   The navigation log must end with `Managed nodes are active`.
3. Launch RViz2. Use the "2D Pose Estimate" tool to initialize the AMCL particle filter at the robot's physical starting location on the map.
4. Use the "Nav2 Goal" tool to send a target destination.
5. **Validation**: The robot should generate a global path, and the DWB local planner should adjust velocities (`cmd_vel`) to trace that path while avoiding dynamic obstacles.
