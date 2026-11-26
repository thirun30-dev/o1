/**
 * Optimus AMR hardware specification - the single source for every number the
 * dashboard shows about the robot itself.
 *
 * Each value mirrors the robot software; keep them in sync when the robot
 * changes (paths are relative to optimus_ws/src):
 *   drivetrain   optimus_bot/config/drivetrain.yaml
 *   motor        zlac_drive/include/zlac_drive/driver/motor.hpp (motor sheet)
 *   driver       optimus_bot/description/control/real_hardware.xacro,
 *                zlac_drive/config/zlac_drive_defaults.yaml
 *   body         optimus_bot/description/base/base.xacro
 *   lidar        optimus_bot/description/sensors/lidar.xacro,
 *                optimus_bot/config/lidar.yaml
 *   navigation   optimus_bot/config/nav2.yaml
 */
export const ROBOT_SPEC = Object.freeze({
  name: 'OPTIMUS AMR',

  body: {
    lengthM: 0.60,
    widthM: 0.40,
    heightM: 0.30,
    // Nav2 footprint is the body rectangle; corner distance from base_link.
    footprintCornerM: Math.hypot(0.30, 0.20),
  },

  drivetrain: {
    type: 'Differential drive, 2 hub motors + 4 casters',
    wheelRadiusM: 0.0655,          // 131 mm wheels
    wheelDiameterMm: 131,
    wheelSeparationM: 0.265,       // wheel-centre to wheel-centre (verified)
    maxLinearMps: 0.5,             // drivetrain safety limit
    maxAngularRadps: 1.0,
    maxReverseMps: 0.2,            // Nav2 velocity smoother
    cmdVelTimeoutS: 0.5,           // wheels stop if /cmd_vel goes quiet
    controlRateHz: 50,
  },

  motor: {
    model: 'ZLLG50ASM200 V2.0 hub motor',
    ratedRpm: 270,
    peakRpm: 350,                  // also the commanded-speed ceiling
    ratedTorqueNm: 3,
    peakTorqueNm: 9,
    ratedCurrentA: 5,
    peakCurrentA: 15,
    encoderLines: 4096,
  },

  driver: {
    model: 'ZLAC8015D',
    description: 'dual-channel hub motor driver',
    bus: 'RS485 Modbus RTU',
    baud: 115200,
    slaveId: 1,                    // one driver, two channels (left / right)
    supplyV: 24,
  },

  lidar: {
    model: 'RPLIDAR A1M8',
    fovDeg: 360,
    nominalRateHz: 10,
    rangeM: 12,
    mountForwardM: 0.24,           // base_link -> lidar_link (x)
    mountHeightM: 0.34,            // scan plane above the wheel axle
    publishedRaysMax: 720,         // front_scan_filter caps /scan
  },

  navigation: {
    stack: 'Nav2 (DWB + RotationShim, NavFn planner)',
    goalToleranceM: 0.15,
    yawToleranceRad: 0.20,
    inflationRadiusM: 0.55,
    mapResolutionM: 0.05,
  },

  safety: {
    // The robot has a hardware stop button that cuts motor power. The
    // dashboard e-stop is software: amr_system_manager publishes zero
    // /cmd_vel at 20 Hz while EMERGENCY_STOP is active. There is no
    // distance-based slow-down zone in the robot software.
    estop: 'Software e-stop (zero /cmd_vel at 20 Hz)',
    estopRateHz: 20,
    hardwareStop: 'Stop button cuts motor power',
  },

  power: {
    // Battery per optimus_ws/README.md (Robot Hardware table).
    nominalV: 25.6,
    capacityAh: 15,
    bmsCurrentA: 20,
    // No battery monitor is connected yet: /amr/battery_state is simulated
    // by amr_system_manager (see utils/battery.js).
    batteryTelemetry: 'simulated',
  },

  software: {
    ros: 'ROS 2 Humble (robot) / Jazzy (simulation)',
  },
});

/** Expected ROS nodes and what they do; presence is checked live via rosapi. */
export const ROBOT_NODES = Object.freeze([
  { name: '/controller_manager', role: 'ros2_control + ZLAC8015D hardware plugin', topic: '/joint_states', group: 'CORE' },
  { name: '/optimus_drivetrain_node', role: 'Diff-drive kinematics + wheel odometry', topic: '/cmd_vel → /odom', group: 'CORE' },
  { name: '/robot_state_publisher', role: 'URDF transforms', topic: '/tf_static', group: 'CORE' },
  { name: '/rplidar_node', role: 'RPLIDAR A1M8 driver', topic: '/scan_raw', group: 'CORE' },
  { name: '/front_scan_filter', role: 'Scan FOV crop + ray cap', topic: '/scan', group: 'CORE' },
  { name: '/amr_system_manager', role: 'Modes, health, e-stop', topic: '/amr/system_state', group: 'CORE' },
  { name: '/amr_location_manager', role: 'Saved locations + goals', topic: '/amr/navigation_status', group: 'CORE' },
  { name: '/rosbridge_websocket', role: 'Dashboard WebSocket bridge', topic: ':9090', group: 'CORE' },
  { name: '/slam_toolbox', role: 'SLAM mapping', topic: '/map', group: 'MAPPING' },
  { name: '/map_server', role: 'Saved map server', topic: '/map', group: 'LOCALIZATION' },
  { name: '/amcl', role: 'Particle-filter localization', topic: '/amcl_pose', group: 'LOCALIZATION' },
  { name: '/planner_server', role: 'NavFn global planner', topic: '/plan', group: 'NAVIGATION' },
  { name: '/controller_server', role: 'DWB path follower', topic: '/cmd_vel_nav', group: 'NAVIGATION' },
  { name: '/bt_navigator', role: 'Behavior-tree navigator', topic: '/navigate_to_pose', group: 'NAVIGATION' },
  { name: '/velocity_smoother', role: 'Velocity smoothing → /cmd_vel', topic: '/cmd_vel', group: 'NAVIGATION' },
  { name: '/optimus_voice', role: 'Voice command node (optional)', topic: '/optimus/voice/*', group: 'OPTIONAL' },
]);

/** Wheel speed (RPM) from body velocities using the real drivetrain geometry. */
export function wheelRpm(linearMps, angularRadps) {
  const { wheelRadiusM, wheelSeparationM } = ROBOT_SPEC.drivetrain;
  const v = Number(linearMps) || 0;
  const w = Number(angularRadps) || 0;
  const toRpm = (mps) => Math.round((mps / (2 * Math.PI * wheelRadiusM)) * 60);
  return {
    left: toRpm(v - (w * wheelSeparationM) / 2),
    right: toRpm(v + (w * wheelSeparationM) / 2),
  };
}
