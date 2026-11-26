# Simulation Test Plan

The simulation pipeline uses Gazebo Classic (`gazebo_ros2_control/GazeboSystem`)
on Humble or Gazebo Sim (`gz_ros2_control/GazeboSimSystem`) on Jazzy; it drives
the simulated wheel joints through the same `forward_command_controller` used
by Optimus.

## Running the Simulation

1. **Launch the stack**:
   ```bash
   ros2 launch optimus_bot simulation.launch.py
   ```
2. **Verify control and physics**:
   In a separate terminal, echo the odometry:
   ```bash
   ros2 topic echo /odom
   ```
3. **Send Commands**:
   Send a twist command:
   ```bash
   ros2 launch optimus_bot teleop.launch.py
   ```
4. **Validation**:
   The Optimus model should move **forward** in Gazebo for a positive
   `linear.x` and turn left for a positive `angular.z`, and `/odom` should
   change with it. `/odom` comes from wheel speeds, so compare against the true
   pose too (`gz model -m optimus -p` on Gazebo Sim).
