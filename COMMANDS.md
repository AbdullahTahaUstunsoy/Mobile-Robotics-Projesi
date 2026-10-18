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
ros2 launch bringup sim.launch.py       # Gazebo + robot + bridge + color_detector + RViz
```

| Argument | Default | Description |
|----------|---------|-------------|
| `headless` | `false` | Gazebo without GUI (server only) |
| `rviz` | `true` | Open RViz (`src/description/rviz/greenhouse_bot.rviz`) |
| `x`, `y`, `yaw` | `-0.5`, `0.65`, `0.0` | Robot start pose (aisle 1, middle row on the right) |
| `world` | `greenhouse.sdf` | World file |

To open only the world: `gz sim -r src/bringup/worlds/greenhouse.sdf`

## 3. Driving the robot

```bash
ros2 run behavior wasd_teleop          # hold w/a/s/d, q/e arc, PgUp/PgDn speed, space stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"                 # stop
```

Published topics: `/cmd_vel`, `/odom`, `/tf`, `/tf_static`, `/joint_states`, `/clock`, `/robot_description`,
`/camera/image_raw`, `/camera/camera_info`, `/color_detector/{red_apple,green_apple,red_checkpoint,green_checkpoint,debug_image}`.

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
- Robot does not move: check that `/cmd_vel` has a publisher with `ros2 topic info /cmd_vel`, and that it reaches Gazebo with `gz topic -e -t /cmd_vel`.
- Teleop does not react to keys: the terminal running `wasd_teleop` must be focused.
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
