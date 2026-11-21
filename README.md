# create3_demos

Small ROS 2 (Python) demos for learning ROS 2 with an **iRobot Create 3**. The package is built with `ament_python` and tested from a Kubuntu laptop that talks to the robot over a Wi-Fi router.

## What's in here

| Node | What it does |
| --- | --- |
| `bump_and_go` | Drives forward. When the bumper or a cliff sensor triggers, it turns **away** from the side that was hit, shows a color on the light ring, and beeps. Sends a stop command on Ctrl+C. Tested on the robot. |
| `action_mission` | Drives a square by sending goals to the robot's built-in action servers (`/drive_distance`, `/rotate_angle`, and optionally `/undock` and `/dock`). Reliable over Wi-Fi. Tested on the robot. |
| `drive_square` | Drives a small square using `/odom` feedback. **Experimental**, see below. |

Speeds, turn times and options are ROS 2 parameters (`config/*.yaml`), so you can change them without editing code.

```
create3_demos/
├── config/
│   ├── action_mission.yaml          # parameters for action_mission
│   ├── bump_and_go.yaml             # parameters for bump_and_go
│   └── drive_square.yaml            # parameters for drive_square
├── launch/
│   ├── action_mission.launch.py     # starts action_mission with its YAML file
│   ├── bump_and_go.launch.py        # starts bump_and_go with its YAML file
│   └── drive_square.launch.py       # starts drive_square with its YAML file
├── create3_demos/
│   ├── action_mission.py            # the action_mission node
│   ├── bump_and_go.py               # the bump_and_go node
│   └── drive_square.py              # the drive_square node
├── resource/create3_demos           # ament package marker
├── package.xml
├── setup.py
├── setup.cfg
├── LICENSE
└── README.md
```

`bump_and_go` subscribes to `/hazard_detection` and publishes `geometry_msgs/Twist` messages to `/cmd_vel` at 10 Hz. It only turns (it never backs up), because the robot's `safety_override` was `"none"`, which limits reversing to about an inch. On a hit it also publishes to `/cmd_lightring` and `/cmd_audio`:

| Hit | Robot turns | Light ring | Beep |
| --- | --- | --- | --- |
| `bump_left`, `bump_front_left` | right | blue | 440 Hz |
| `bump_right`, `bump_front_right` | left | green | 660 Hz |
| `bump_front_center` | alternates left and right | magenta | 550 Hz |
| a cliff sensor | alternates left and right | yellow | two falling notes |

When a turn ends, the light ring goes back to the robot's own patterns. Hits that happen during a turn are ignored.

## Requirements

- **Robot:** iRobot Create 3, joined to the same 2.4 GHz Wi-Fi network as your computer. It was tested on firmware `G.5.3`.
- **Computer:** Ubuntu or Kubuntu 26.04 (tested on Kubuntu 26.04).
- **ROS 2:** Lyrical Luth (`ros-lyrical-desktop`, `ros-dev-tools`).
- **Middleware:** Cyclone DDS (`ros-lyrical-rmw-cyclonedds-cpp`). The robot is set to Cyclone DDS, and both sides must use the same one.
- **Messages:** iRobot's message definitions (`ros-lyrical-irobot-create-msgs`).
- **Network:** a Wi-Fi router (2.4 GHz) that the robot and your computer both join.

## 1. Install ROS 2 Lyrical

Run these in a terminal. They follow the official [Ubuntu install guide for Lyrical](https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html), so check it if anything has changed.

```bash
# UTF-8 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Enable the universe repository
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

# Add the ROS 2 apt source
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# Install ROS 2 and the build tools (colcon, etc.)
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-lyrical-desktop ros-dev-tools

# Load ROS in every new terminal
echo "source /opt/ros/lyrical/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Test it with two terminals:

```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

If the listener prints what the talker says, ROS 2 is working.

## 2. Install the Create 3 packages

```bash
sudo apt install -y ros-lyrical-rmw-cyclonedds-cpp ros-lyrical-irobot-create-msgs
```

Without `irobot-create-msgs`, commands like `ros2 topic echo /ir_intensity` fail with `The message type 'irobot_create_msgs/msg/IrIntensityVector' is invalid`.

## 3. Connect the robot and your computer to the router

1. Join the same 2.4 GHz Wi-Fi network on both devices. The Create 3 can only use 2.4 GHz.
2. To put the robot on the router, hold **both side buttons** until the light ring turns white to enter AP mode. Join the robot's own network (`create-XXXX`, printed on its sticker), open `http://192.168.10.1`, go to **Connect**, and enter your router's network name and password.
3. Find the robot's address in your router's DHCP client list (the hostname looks like `robot05`), then check it from your computer:

```bash
ping <robot-ip>
```

4. Open `http://<robot-ip>` in a browser and go to **Application > Configuration**. Note these three settings, because your computer must match them:
   - **ROS 2 Domain ID** (printed on the robot's sticker, 5 in the original setup)
   - **RMW implementation**: `rmw_cyclonedds_cpp`
   - **ROS 2 namespace**: empty

Don't put your router's Wi-Fi password or admin login in this repo.

## 4. Configure ROS 2 on your computer

Create `~/cyclonedds.xml` with the robot's address (replace `192.168.30.106` with yours):

```xml
<CycloneDDS>
  <Domain>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <Peers>
        <Peer address="192.168.30.106"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

Add these to `~/.bashrc` (use your robot's Domain ID):

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=5
export CYCLONEDDS_URI=file://$HOME/cyclonedds.xml
```

Open a new terminal and check that the robot is visible:

```bash
printenv | grep -E "ROS|RMW|CYCLONE"
ros2 topic list --spin-time 10
ros2 topic echo /battery_state --once
```

You should see topics such as `/cmd_vel`, `/odom`, `/hazard_detection` and `/battery_state`. If you only see `/parameter_events` and `/rosout`, see Troubleshooting below.

The robot's address comes from the router's DHCP, so it can change. If discovery stops working, check the DHCP list and update the `Peer address` in `cyclonedds.xml`.

## 5. Build and run this package

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Rabi246/create3_demos.git
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

To load the workspace in every new terminal, add it to `~/.bashrc`:

```bash
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
```

Put the robot **on the floor in a clear area**, stay close, and keep a finger on Ctrl+C. Do not run it on a desk, because it can drive off the edge.

```bash
ros2 launch create3_demos bump_and_go.launch.py
```

Or run the node directly and override a parameter:

```bash
ros2 run create3_demos bump_and_go --ros-args -p forward_speed:=0.08
```

Press the front bumper to test it. The node logs something like `hazard at bump_left (left), turning right`, the light ring changes color, and the robot beeps and turns away.

### Parameters

| Parameter | Default | Meaning |
| --- | --- | --- |
| `forward_speed` | `0.15` | Forward speed in m/s |
| `turn_speed` | `0.8` | Turning speed in rad/s |
| `turn_time` | `2.0` | How long each turn lasts, in seconds (about 90 degrees at the defaults) |
| `lights` | `true` | Color the light ring on a hit |
| `sound` | `true` | Beep on a hit |

Example: `ros2 run create3_demos bump_and_go --ros-args -p sound:=false -p forward_speed:=0.08`

### drive_square (experimental)

```bash
ros2 launch create3_demos drive_square.launch.py
```

Drives forward `side_length` metres (default `0.2`), turns left 90 degrees using `/odom`, and repeats for `sides` sides (default `4`). It stops if a bumper or cliff sensor triggers. Parameters are in `config/drive_square.yaml`: `side_length`, `forward_speed`, `turn_speed`, `sides`, `min_forward_speed`, `min_turn_speed`, `odom_timeout` and `pause_time`.

To watch it, run `rviz2`, set the **Fixed Frame** to `odom`, add a **TF** display, and add an **Odometry** display on `/odom` with **Reliability Policy** set to **Best Effort** and **Keep** set to `100`.

Known issue: over the lab's slow 2.4 GHz router, `/odom` arrives late and with gaps. In early runs, some corners spun far past 90 degrees (a full extra turn in one case), so the square wasn't reliable. The current version adds the following protections, tested only in a simulation with delayed and dropped odometry, not yet confirmed on the robot:

- It adds up small heading changes instead of comparing against the start heading, so the wrap-around at 180 degrees can't fool it.
- It slows down near each target.
- It stands still if odometry stops arriving.
- It pauses briefly after each side and corner.

For a reliable square over a laggy link, use `action_mission` instead (next section). It uses the robot's built-in actions, which run on the robot itself.

### action_mission

```bash
ros2 launch create3_demos action_mission.launch.py
```

Instead of publishing speeds to `/cmd_vel` and watching `/odom`, this node sends the robot **goals** through ROS 2 actions and waits for the robot to report back. The robot measures the distance and angle itself, so Wi-Fi lag doesn't make the moves sloppy.

The default mission is a square:

1. `/drive_distance` forward `side_length` metres
2. `/rotate_angle` 90 degrees to the left
3. Repeat for `sides` sides (the last corner returns the robot to its starting heading)

Optionally, it can start with `/undock` and finish with `/dock`. If any step fails (for example, the robot rejects a goal or a bump aborts a move), the mission stops and logs the reason. Ctrl+C cancels the goal that is running.

The robot can start from anywhere on the floor. Put it in a clear area of about 1 m by 1 m, and keep Ctrl+C ready.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `side_length` | `0.3` | Metres per side |
| `sides` | `4` | Number of sides |
| `drive_speed` | `0.1` | Maximum forward speed in m/s |
| `turn_speed` | `0.5` | Maximum turning speed in rad/s |
| `undock_first` | `false` | Start by undocking (the robot must be on its dock) |
| `dock_at_end` | `false` | Finish by docking (the robot must be able to see its dock) |
| `pause_time` | `0.5` | Seconds to pause between steps |
| `server_timeout` | `10.0` | Seconds to wait for the robot's action servers |

Example: `ros2 run create3_demos action_mission --ros-args -p side_length:=0.2`

On the robot, the square ran all 4 sides and 4 corners in order, and each step took about the same time. The `undock_first` and `dock_at_end` options have not been tested yet.

## Useful terminal commands

```bash
# Drive forward or spin until Ctrl+C
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}"

# Stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"

# Precise moves
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5, max_translation_speed: 0.15}"
ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 1.57, max_rotation_speed: 0.5}"

# Dock and undock
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
```

## Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| `ros2 topic list` shows only `/parameter_events` and `/rosout` | Middleware mismatch, discovery wait too short, or the robot application isn't running | Use Cyclone DDS on both sides, match the Domain ID, run `ros2 topic list --spin-time 10`, restart the robot application |
| `parser error` mentioning `CycloneDDS` | A typo in `cyclonedds.xml` (tag names are case sensitive) | Check the file against the example above |
| `The message type '...' is invalid` | `irobot_create_msgs` isn't installed | `sudo apt install ros-lyrical-irobot-create-msgs`, then open a new terminal |
| `ping: http://...: Name or service not known` | `ping` takes an address, not a URL | `ping 192.168.30.1` |
| `Package 'create3_demos' not found` | The workspace isn't sourced | `source ~/ros2_ws/install/setup.bash` |
| `No executable found` | `entry_points` is missing in `setup.py` | Check that `bump_and_go` is listed under `console_scripts`, then rebuild |
| Robot's network (`create-XXXX`) isn't in the Wi-Fi list | The robot is joined to a router and isn't in AP mode | Hold both side buttons until the ring turns white, wait 30 seconds, refresh |
| Solid red light ring | An error or low battery | Put it on its dock and restart the application from the robot's web page |

## Notes

- `bump_and_go` (including the light ring and beeps) and `action_mission` (the square, without undock and dock) have been run on a real robot. `drive_square` is experimental, as described above.
- This was developed on a shared lab robot, so check with whoever manages yours before changing its Wi-Fi, Domain ID, RMW setting or firmware.
- Updating the firmware (the robot's **Update** tab) wasn't done. The robot was on `G.5.3` and worked with ROS 2 Lyrical for topics and messages.

## License

MIT. See [LICENSE](LICENSE).
