# Transform (TF) Tree

The Optimus robot uses a standard ROS 2 transform hierarchy. Strict adherence to this tree ensures that SLAM and Nav2 function properly.

## Hierarchy

```text
map
 └── odom
      └── base_link
           ├── chassis
           ├── storage_container
           ├── left_wheel        (+y, robot's left)
           ├── right_wheel       (-y, robot's right)
           ├── front_left_caster
           ├── front_right_caster
           ├── rear_left_caster
           ├── rear_right_caster
           └── lidar_link        (x 0.24 m, z 0.34 m)
```

## Frame Responsibilities
- **`map`**: the fixed, globally consistent frame. `map -> odom` is published by `slam_toolbox` (mapping) or `amcl` (localization) - never both.
- **`odom`**: the continuous, drift-prone odometry frame. `odom -> base_link` is published by `optimus_drivetrain_node` from wheel speeds.
- **`base_link`**: the robot's rotation centre, midway between the two hub motors at wheel-axle height. Frames follow REP-103: +x forward, +y left.
- **`lidar_link`**: the RPLIDAR scan centre; published by `robot_state_publisher` from `lidar.xacro`.
