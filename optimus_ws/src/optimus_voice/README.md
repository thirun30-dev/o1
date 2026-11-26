# optimus_voice — "Optimus, ..." voice control for the Optimus AMR

Speak to the robot through the laptop mic or the web dashboard. Commands go to Nav2,
and Optimus answers in a deep, robotic commander voice.

```
Laptop mic ─► Vosk (offline STT) ─┐
                                  ├─► wake word "Optimus" ─► intent parser ─► Nav2 actions
Dashboard (Chrome Web Speech) ────┘        /optimus/voice/text_command        (navigate_to_pose,
                                                                              follow_waypoints,
Replies ◄─ /optimus/voice/response ◄─ Piper/espeak + sox FX ◄──────────────    spin, drive_on_heading)
```

## Commands (all start with "Optimus")

| Say | What happens |
|---|---|
| go to the loading bay / navigate to point one / just "packing" | NavigateToPose to that waypoint (fuzzy matched, aliases ok) |
| start auto navigation / start patrol / patrol in a loop | FollowWaypoints through the `patrol:` list (loop repeats) |
| go home / return to the dock | Navigate to `home` |
| move forward 2 meters / go back 50 centimeters | DriveOnHeading (Nav2 collision checking stays on) |
| turn left / turn right 45 degrees / turn around | Spin |
| stop / halt / cancel / emergency stop | Cancel goal and publish zero `cmd_vel` |
| save this location as charging corner | Writes current map pose into waypoints.yaml |
| where are you / battery status / what places can you go to / who are you | Spoken reports |

Say just "Optimus", and it answers "I am listening" and waits 6 s for the command.

> **Note:** Voice "stop" is a convenience and does not replace your hardware E-stop.

## Install (Ubuntu, robot computer)

```bash
sudo apt install sox espeak-ng pulseaudio-utils ros-$ROS_DISTRO-rosbridge-server
pip install vosk sounddevice          # add --break-system-packages on 24.04

# Speech model (offline, ~36 MB, Indian English)
cd ~ && wget https://alphacephei.com/vosk/models/vosk-model-small-en-in-0.4.zip && unzip vosk-model-small-en-in-0.4.zip

# Optional, much better voice: Piper neural TTS + a deep male voice
pip install piper-tts
mkdir -p ~/piper && cd ~/piper
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/ryan/high/en_US-ryan-high.onnx{,.json}
```

Without Piper, it falls back to espeak-ng automatically, using the same effect chain.

## Build & run

The package is part of `~/optimus_ws/src` and is built with the rest of the workspace.

```bash
cp ~/optimus_ws/src/optimus_voice/config/waypoints.yaml ~/optimus_waypoints.yaml   # editable copy
cd ~/optimus_ws && colcon build --packages-select optimus_voice && source install/setup.bash

# Bring up your robot + Nav2 (localization or SLAM) first, then:
ros2 launch optimus_voice voice.launch.py waypoints_file:=$HOME/optimus_waypoints.yaml rosbridge:=true

# Test the voice alone
ros2 run optimus_voice say "Optimus online. Awaiting orders."

# Test without a mic
ros2 topic pub --once /optimus/voice/text_command std_msgs/String "{data: 'optimus go to the loading bay'}"
```

Pick a mic with `python3 -m sounddevice` and set `mic_device`.

The voice node uses its **own** list of places (`waypoints_file`, default
`config/waypoints.yaml`), separate from the dashboard's saved locations. The
shipped coordinates are placeholders - drive to each place and say
"Optimus, save this location as <name>", or edit the file, before relying on
voice navigation on the real robot.

## Website

The Optimus web dashboard has voice control built in: press **Ctrl + ,** (or
the **VOICE** button) to open the panel and speak one command, or switch on
hands-free mode for "Optimus, ...". Typed commands and quick buttons work too.
It publishes on `/optimus/voice/text_command` and shows replies from
`/optimus/voice/response`. Start the node with the robot:

```bash
ros2 launch optimus_bot bringup.launch.py use_voice:=true
```

(`bringup.launch.py` starts this package's `voice.launch.py` with
`enable_mic:=false`, since speech is recognised in the browser.)

`web/VoiceCommandPanel.jsx` is a standalone version of the panel for other
React apps: copy it into the app, run `npm install roslib`, then:

```jsx
const ros = new ROSLIB.Ros({ url: `ws://${window.location.hostname}:9090` });
<VoiceCommandPanel ros={ros} />
```

Browser speech recognition works only in Chrome or Edge, on `localhost` or `https`. When you open the
dashboard from your phone over plain `http://<ip>`, the mic is blocked, but typed commands and the
quick buttons still work. The browser's Web Speech API uses Google's cloud for recognition. The laptop mic
path uses Vosk and runs fully offline.

## Tuning the voice

The effects are in `SOX_FX` in `commander_voice.py`:
- `pitch -300`: deeper (try -200 to -450)
- `echo ... 9 ...`: the metallic "robot body" (5 to 15 ms)
- `overdrive 3`: grit
- `reverb 22`: space

Edit the replies in `lines.py`.

## Parameters you may need to change

In `config/voice_params.yaml`:
- `base_frame`: set it to `base_footprint` if Nav2 uses that frame.
- `cmd_vel_topic`: change it if your controller listens somewhere else. On Jazzy with `enable_stamped_cmd_vel`, the stop
  command must be a TwistStamped, so either point it at the unstamped input of your twist mux or rely on goal cancel.
- `battery_topic`: defaults to `/amr/battery_state` (published by `amr_system_manager`, simulated until a battery monitor is fitted). Point it at your ESP32's `sensor_msgs/BatteryState` topic once that exists.
