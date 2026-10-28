# Commands

Frequently used commands for building, running, testing and debugging the project.
All commands are run from the workspace root:

```bash
cd "/home/samsapulee/Desktop/School/Mobile Robotics/Mobile-Robotics-Projesi"
```

> The folder path contains a space (`Mobile Robotics`): always quote it.

## 0. Clone and setup

```bash
git clone https://github.com/AbdullahTahaUstunsoy/Mobile-Robotics-Projesi.git ~/Mobile-Robotics-Projesi
cd ~/Mobile-Robotics-Projesi
sudo apt install -y python3-colcon-common-extensions ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge \
  ros-lyrical-robot-state-publisher ros-lyrical-xacro
rosdep update && rosdep install --from-paths src --ignore-src -r -y   # missing dependencies
```

Details and private repo access: `README.md` → *Setup*.

## 1. Build

```bash
source /opt/ros/lyrical/setup.bash              # ROS 2 (if not in .bashrc)
colcon build --symlink-install                  # all packages
colcon build --symlink-install --packages-select behavior  # a single package
source install/setup.bash                       # once in every new terminal
```

- With `--symlink-install`, changes to Python, launch, xacro, world and yaml files take effect **without rebuilding**.
  Rebuild after adding a new file, a new node (`setup.py` entry point) or changing `CMakeLists.txt` / `package.xml`.
- `Package 'bringup' not found` → `source install/setup.bash` was not run in that terminal.
- Clean build: `rm -rf build install log && colcon build --symlink-install`

## 2. Simulation

```bash
ros2 launch bringup sim.launch.py       # Gazebo + robot + bridge + color_detector + obstacle_detector + safety + RViz
```

| Argument | Default | Description |
|----------|---------|-------------|
| `headless` | `false` | Gazebo without GUI (server only) |
| `rviz` | `true` | Open RViz (`src/description/rviz/greenhouse_bot.rviz`) |
| `mission` | `false` | Run the autonomous greenhouse mission |
| `x`, `y`, `yaw` | `-0.5`, `0.65`, `0.0` | Robot start pose (aisle 1, middle row on the right) |
| `world` | `greenhouse.sdf` | World file |

To open only the world: `gz sim -r src/bringup/worlds/greenhouse.sdf`

## 3. Driving the robot

```bash
ros2 run behavior wasd_teleop          # hold w/a/s/d, q/e arc, PgUp/PgDn speed, space stop
ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist "{linear: {x: 0.3}}"  # through the safety filter
ros2 topic pub --once /cmd_vel_raw geometry_msgs/msg/Twist "{}"                 # stop
```

Published topics: `/cmd_vel_raw`, `/cmd_vel`, `/odom`, `/tf`, `/tf_static`, `/joint_states`, `/clock`, `/robot_description`,
`/camera/image_raw`, `/camera/camera_info`, `/color_detector/{red_apple,green_apple,red_checkpoint,green_checkpoint,debug_image}`,
`/scan`, `/obstacle_detector/{state,front_distance,markers}`, `/bumper`, `/safety/stop`, `/imu`.

```bash
ros2 topic list
ros2 topic echo /odom --once
```

## 3b. Camera and color detection

```bash
ros2 run rqt_image_view rqt_image_view /color_detector/debug_image   # detections + mask
ros2 run rqt_image_view rqt_image_view /camera/image_raw             # raw image
ros2 topic echo /color_detector/red_apple                            # x: horizontal offset, z: area ratio
ros2 topic hz /camera/image_raw                                      # ~15 Hz
ros2 run behavior color_detector --ros-args -p green_sat_min:=140   # tune a threshold (thresholds are read at startup)
python3 -m pytest -q src/behavior/test                               # unit tests
```

## 3c. Lidar and obstacle detection

```bash
ros2 topic hz /scan                                                  # ~10 Hz
ros2 topic echo /obstacle_detector/state                             # CLEAR / WARN / STOP
ros2 topic echo /obstacle_detector/front_distance                    # nearest point ahead (m)
ros2 run behavior obstacle_detector --ros-args -p stop_distance:=0.3 # tune a parameter (or edit config/obstacle_detector.yaml)
```

## 3d. Bumper and safety filter

```bash
ros2 topic echo /safety/stop                                         # True while forward motion is blocked
ros2 topic echo /bumper --field contacts                             # only while the bumper touches something
gz topic -e -t /bumper                                               # Gazebo side, before the bridge
ros2 run behavior safety --ros-args -p obstacle_stop:=false          # bumper only (or edit config/safety.yaml)
# The world has a low orange box (low_obstacle) ~1 m behind the start pose: turn around and drive into it.
# Another low box (below the lidar plane) anywhere, e.g. 0.45 m in front of the start pose:
ros2 run ros_gz_sim create -world greenhouse -name low_box -x 0.3 -y 0.65 -z 0.05 -string \
  '<sdf version="1.9"><model name="low_box"><static>true</static><link name="l"><collision name="c"><geometry><box><size>0.1 0.4 0.1</size></box></geometry></collision><visual name="v"><geometry><box><size>0.1 0.4 0.1</size></box></geometry></visual></link></model></sdf>'
gz service -s /world/greenhouse/remove --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "low_box", type: MODEL'                             # remove it again
```

## 3e. Autonomous mission

```bash
ros2 launch bringup sim.launch.py mission:=true                      # simulation + mission
ros2 run behavior greenhouse_mission                                 # mission only (fresh simulation, no teleop running)
ros2 run behavior greenhouse_mission --ros-args -p safety_timeout:=10.0
ros2 topic echo /imu --field orientation                             # IMU heading source
python3 -m pytest -q src/behavior/test/test_mission.py               # full route on a simulated robot
```

The route is in `src/behavior/config/mission.yaml` (Gazebo world coordinates); see `README.md` → *Autonomous mission*.

## 4. Figures for the report

```bash
ros2 run rqt_graph rqt_graph                          # node/topic graph
ros2 run tf2_tools view_frames                        # TF tree → frames_*.pdf (gitignored)
ros2 run tf2_ros tf2_echo odom base_footprint         # transform between two frames
```

## 5. Checks

```bash
# Xacro / URDF check
xacro src/description/urdf/greenhouse_bot.urdf.xacro > /tmp/bot.urdf && check_urdf /tmp/bot.urdf
gz sdf -k src/bringup/worlds/greenhouse.sdf   # is the world file valid
```

### Headless simulation test

Use a different `ROS_DOMAIN_ID` so it does not mix with an open simulation:

```bash
export ROS_DOMAIN_ID=42
ros2 launch bringup sim.launch.py headless:=true rviz:=false
gz model -m greenhouse_bot -p        # true pose in Gazebo (ground truth): x y z / roll pitch yaw
gz topic -l                           # Gazebo-side topics (before the bridge)
```

## 6. Troubleshooting

```bash
# Gazebo closed but the server is still running (old world opens, simulation is slow)
pgrep -af gz
pkill -f gz-sim-main
```

- `libEGL warning: ... dri2 screen` warnings are normal in headless runs.
- Robot does not move: check that `/cmd_vel` has a publisher with `ros2 topic info /cmd_vel` (the `safety` node), that `/safety/stop` is `False`, and that it reaches Gazebo with `gz topic -e -t /cmd_vel`.
- Robot does not drive forward: `/safety/stop` is `True` (bumper touched or lidar `STOP`): reverse or turn away.
- Nothing on `/bumper`: it is only published during a contact; check the `Contact` plugin in `greenhouse.sdf` and `gz topic -l | grep bumper`.
- Teleop does not react to keys: the terminal running `wasd_teleop` must be focused.
- Nothing on `/scan`: check that `lidar.xacro` is included in `greenhouse_bot.urdf.xacro` and that it shows in `gz topic -l`.
- Mission fails with `odom is not at its origin`: the robot already moved; restart `sim.launch.py`.
- Mission fails with `another node is publishing cmd_vel_raw`: close `wasd_teleop` (or any other driver).
- Mission fails with `no odom / imu / safety data`: Gazebo did not start (check `create` keeps printing `Requesting list of world names`); a VPN such as Cloudflare WARP can block Gazebo transport.
- Nothing on `/color_detector/*`: check that `/camera/image_raw` is publishing (`ros2 topic hz`), then look at the mask inset in `debug_image`.

## 7. Git

```bash
git status
git pull --rebase
git add -A && git commit -m "Short description"
git push
git log --oneline -10
```

- Repo: https://github.com/AbdullahTahaUstunsoy/Mobile-Robotics-Projesi
- `build/`, `install/`, `log/`, `frames*.pdf` are gitignored.
