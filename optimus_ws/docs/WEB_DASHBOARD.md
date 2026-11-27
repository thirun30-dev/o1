# Web Dashboard User & Developer Guide

## Overview

The Optimus Web Dashboard is a responsive single-page application built with
**React 18**, **Vite**, **Tailwind CSS 4** and **roslibjs**. It runs in the
operator's browser and talks to ROS 2 through rosbridge (WebSocket, port 9090).
It uses a dark "control room" theme designed for warehouse tablets and laptops.

---

## 1. Running the dashboard

```bash
cd web_dashboard
npm install          # first time
npm run dev          # http://<host>:3000
npm run build        # production build in dist/
```

Connection target, in order of priority:

1. URL: `?ros_url=ws://<robot-ip>:9090` or `?ros_host=<robot-ip>&ros_port=9090`
2. The URL saved from the connection settings (open them from the ONLINE/OFFLINE pill in the header)
3. `VITE_ROSBRIDGE_URL` in `.env`
4. `ws://<page host>:9090`

When the dashboard runs on a laptop and the robot on a Raspberry Pi, open
`http://localhost:3000/?ros_host=<pi-ip>`. Serving from `localhost` also lets
the browser microphone work for voice commands.

### Using the Vercel (https://) deployment on the same Wi-Fi

Browsers block `ws://` from an `https://` page, so the hosted dashboard
connects with `wss://` and rosbridge must run with TLS:

1. On the Pi, once: `~/optimus_ws/tools/optimus install` and
   `optimus url https://<your-app>.vercel.app`
2. Start the robot: **`optimus`**. It creates or renews the certificate,
   prints the links below and runs bringup with `rosbridge_ssl:=true`.
   Extra launch args pass through, e.g. `optimus use_voice:=true`.
   The systemd service (`tools/optimus-bringup.service`) runs the same command.
3. On each phone or laptop, once: open `https://<pi-ip>:9090` and accept the
   certificate warning.
4. Open `https://<your-app>.vercel.app/?ros_host=<pi-ip>`. If Chrome asks to
   access devices on your local network, choose **Allow**.

`ws://` URLs are upgraded to `wss://` automatically on https pages.

---

## 2. Pages

| Page | What it shows |
|---|---|
| **Cockpit** | Autonomy modes (READY / MAPPING / AMCL / NAV2), pose and velocity, map with scan, costmap, path and saved locations, and a tabbed panel: **Drive** (joystick, D-pad, WASD), **Camera** (live front / obstacle / depth camera view, see [CAMERA.md](CAMERA.md)), **POIs**, **Maps**, **Health** |
| **POI Dispatcher** | Build a route of saved locations with dwell times; run once or in a loop |
| **Diagnostics** | Robot specification (motors, RS485, LiDAR, drivetrain), live 360° LiDAR clearance by sector, wheel RPM from odometry, the live ROS graph (which robot nodes are running) and a **self-test** |
| **System Logs** | Session statistics (distance, goals, peak speed, e-stops) and a live event log of mode changes, goals, e-stops and health changes |

Header: connection status and settings, **VOICE** (Ctrl + ,), **SOP GUIDE**
(step-by-step operating procedure), mode badge and **E-STOP**. On phones the
bottom bar holds Cockpit, Dispatch, Logs, Voice and E-STOP.

All numbers come from the robot. Values the robot cannot measure are labelled:
the battery is **simulated** until a battery monitor is fitted, and motor
current/temperature are not shown because the driver does not report them.

---

## 3. Key capabilities

### Mapping, localization and navigation
- Start/stop SLAM Toolbox mapping, save maps, serialize and resume pose graphs.
- Select a saved map and start map server + AMCL; set the initial pose on the map.
- Start Nav2, click (and drag for heading) on the map to send goals, cancel goals.

Button operations use typed ROS services (`/amr/set_mode`, `/amr/map_operation`,
`/amr/set_initial_pose`, `/amr/cancel_navigation`). Map-click goals are sent as
`{"action": "navigate_to_pose"}` on `/amr/command`, which the location manager
turns into exactly one Nav2 goal. Status comes from `/amr/system_state`,
`/amr/system_health`, `/amr/maps`, `/amr/map_operation_result` and
`/amr/navigation_status`.

### Warehouse locations
- Save the current pose (or a clicked point) with a name and type.
- **GO** on any location sends the robot there.

### Voice commands - Ctrl + ,
- **Ctrl + ,** opens the voice panel and listens for one command (no wake word
  needed). Press again or **Esc** to close and release the microphone.
- **Hands-free** mode listens continuously and acts on "Optimus, ...".
- Typed commands and quick buttons (Where are you?, Go home, Start patrol,
  List places, Battery, STOP) always work.
- Needs the voice node: `bringup.launch.py use_voice:=true`. The panel warns if
  it is not running.
- Browser speech recognition needs Chrome or Edge on `https://` or `localhost`.

### Safe teleoperation
- Keyboard: **WASD** / arrow keys, **Space** to stop. Releasing the last key,
  switching tabs or leaving the window stops the robot.
- Joystick and D-pad; speed sliders limited to the robot's 0.5 m/s and
  1.0 rad/s; inching mode (0.08 m/s) for docking.
- The drivetrain also stops if `/cmd_vel` goes quiet for 0.5 s.

### Emergency stop
- E-STOP in the header (and bottom bar on phones) latches `EMERGENCY_STOP`,
  cancels Nav2 goals and holds zero velocity until reset.
- This is a software stop. The robot's hardware stop button cuts motor power.

---

## 4. Directory structure

```text
web_dashboard/
├── index.html
├── package.json
├── vite.config.js
└── src/
    ├── main.jsx
    ├── App.jsx                       # Page layout, Ctrl + , shortcut, mobile bar
    ├── index.css                     # Dark control-room design system
    ├── config/
    │   └── robotSpec.js              # Robot specification (single source for UI numbers)
    ├── components/
    │   ├── Header.jsx                # Navigation, connection, voice, SOP, E-STOP
    │   ├── ScrollingTicker.jsx       # Live telemetry strip
    │   ├── RobotStatus.jsx           # Pose, velocity, battery, Nav2 status
    │   ├── ModeSelector.jsx          # Autonomy mode deck
    │   ├── MapViewer.jsx             # Canvas map: grid, scan, costmap, paths, robot
    │   ├── Teleop.jsx                # Joystick / D-pad / keyboard teleoperation
    │   ├── LocationManager.jsx       # Saved locations
    │   ├── MapManager.jsx            # Save / load / resume maps
    │   ├── SensorStatus.jsx          # Health panel
    │   ├── WaypointScrollBar.jsx     # Quick waypoint bar
    │   ├── VoiceCommandModal.jsx     # Voice panel (Ctrl + ,)
    │   ├── WarehouseSopModal.jsx     # Operating procedure guide
    │   └── templates/
    │       ├── MissionDispatcherTemplate.jsx
    │       ├── DiagnosticsTemplate.jsx
    │       └── AnalyticsTemplate.jsx
    ├── hooks/
    │   └── useRos.js                 # React state for all ROS data and commands
    ├── ros/
    │   └── rosConnection.js          # roslibjs connection, topics, services
    └── utils/
        └── battery.js
```

When the robot changes, update `src/config/robotSpec.js`; each value there
names the robot config file it mirrors.
