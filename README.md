# Mobile-Robotics-Projesi

A greenhouse inspection robot simulated with **ROS 2 Lyrical** and **Gazebo Jetty** (BIMU4101 midterm project).

A 4-wheel skid-steer robot drives through the aisles of a greenhouse. It can be driven with the keyboard (WASD) or run a fully autonomous mission. Along the way it detects ripe (red) and unripe (green) apples and the RED/GREEN checkpoint panels on the wall with its camera. A safety filter based on the 2D LiDAR and the front bumper keeps it from driving into obstacles.

## Contents

- [Sensors](#sensors)
- [Packages](#packages)
- [World](#world-srcbringupworldsgreenhousesdf)
- [Installation](#installation)
- [Build](#build)
- [Run](#run)
- [RGB camera and color detection](#rgb-camera-and-color-detection)
- [LiDAR and obstacle detection](#lidar-and-obstacle-detection)
- [Bumper and safety filter](#bumper-and-safety-filter)
- [IMU and autonomous navigation](#imu-and-autonomous-navigation)
- [Tests and CI](#tests-and-ci)

## Sensors

The robot has four sensor types. Each one is declared in its own xacro file and is bridged from Gazebo to ROS 2 by `ros_gz_bridge` (`src/bringup/config/bridge.yaml`).

| Sensor | Model file | ROS 2 topic | Message type | Processed by |
| --- | --- | --- | --- | --- |
| RGB camera | `urdf/camera.xacro` | `/camera/image_raw` | `sensor_msgs/Image` | `color_detector` |
| 2D LiDAR | `urdf/lidar.xacro` | `/scan` | `sensor_msgs/LaserScan` | `obstacle_detector` |
| Bumper (contact) | `urdf/bumper.xacro` | `/bumper` | `ros_gz_interfaces/Contacts` | `safety` |
| IMU | `urdf/imu.xacro` | `/imu` | `sensor_msgs/Imu` | `greenhouse_mission` |

Odometry (`/odom`) and joint states come from the drive plugin and are not counted as sensors.

## Packages

| Package | Contents |
| --- | --- |
| `bringup` | `launch/sim.launch.py` (single launch file), `worlds/greenhouse.sdf`, `config/bridge.yaml`, `config/obstacle_detector.yaml`, `config/safety.yaml` |
| `description` | Robot model `urdf/greenhouse_bot.urdf.xacro`: 4-wheel skid-steer base with the Gazebo DiffDrive plugin (`urdf/gazebo.xacro`), sensors in `urdf/camera.xacro`, `urdf/lidar.xacro`, `urdf/bumper.xacro`, `urdf/imu.xacro`; RViz config `rviz/greenhouse_bot.rviz` |
| `behavior` | Python nodes: `wasd_teleop` (keyboard driving), `color_detector` (camera), `obstacle_detector` (LiDAR), `safety` (bumper + LiDAR filter), `greenhouse_mission` (autonomous mission, uses the IMU and odometry); mission route in `config/mission.yaml`; unit tests in `test/` |

In `behavior`, every node is split into two files: `<name>.py` holds the logic (pure Python, no ROS, unit tested) and `<name>_node.py` is the ROS 2 wrapper.

## World: `src/bringup/worlds/greenhouse.sdf`

- 3 planter rows (y = 1.4, 0.0, -1.4) with 2 aisles between them
- 9 fruit on the middle row: 5 red (ripe), 4 green (unripe)
- RED and GREEN checkpoint panels (with a white X) on the east wall
- An 11 × 5.4 m area enclosed by four walls
- A low orange box (0.1 m tall, at x = -1.5, y = 0.65, behind the start pose) for bumper tests: it is below the LiDAR scan plane, so only the bumper detects it
- No external models; runs as a single file

## Installation

### Requirements

- Ubuntu 26.04
- **ROS 2 Lyrical** (`/opt/ros/lyrical`) with **Gazebo Jetty**; with Lyrical, Gazebo is installed from the ROS repository through `ros-lyrical-ros-gz`
- `colcon` and `rosdep`

### 1. Install the dependencies

~~~bash
source /opt/ros/lyrical/setup.bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep \
  ros-lyrical-ros-gz ros-lyrical-robot-state-publisher ros-lyrical-xacro \
  ros-lyrical-rviz2 ros-lyrical-cv-bridge \
  python3-opencv python3-numpy python3-yaml python3-pytest
~~~

### 2. Clone the repository

~~~bash
git clone https://github.com/AbdullahTahaUstunsoy/Mobile-Robotics-Projesi.git ~/Mobile-Robotics-Projesi
cd ~/Mobile-Robotics-Projesi
~~~

The repository is itself the colcon workspace: the packages are in `src/`. The `build/`, `install/` and `log/` folders come empty from the repo; they fill up during the build and their contents are not committed.

### 3. Check for missing dependencies (recommended)

`rosdep` reads the `package.xml` files and installs anything still missing:

~~~bash
sudo rosdep init        # first time only; skip it if it says "already exists"
rosdep update
rosdep install --from-paths src --ignore-src -r -y
~~~

## Build

~~~bash
cd ~/Mobile-Robotics-Projesi
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

Run `source /opt/ros/lyrical/setup.bash && source install/setup.bash` in every new terminal.
If you rename a package or see stale-package errors, do a clean build: `rm -rf build install log`, then build again.

## Run

One launch file starts everything: Gazebo with the world, the robot, the ROS–Gazebo bridge, the color and obstacle detectors, the safety filter and RViz.

### Option A: drive with the keyboard

Terminal 1:

~~~bash
ros2 launch bringup sim.launch.py
~~~

Terminal 2 (this terminal must be focused while driving):

~~~bash
source /opt/ros/lyrical/setup.bash && source install/setup.bash
ros2 run behavior wasd_teleop
~~~

| Key | Motion |
| --- | --- |
| `w` / `s` | forward / backward |
| `a` / `d` | turn left / right in place |
| `q` / `e` | forward + left / right |
| `PgUp` / `PgDn` | speed up / down by 10% |
| `space` | stop |
| `Ctrl+C` | quit (stops the robot) |

Hold the key down; the robot stops when you release it.

### Option B: autonomous mission

~~~bash
ros2 launch bringup sim.launch.py mission:=true
~~~

The robot runs the 13-stage route in `src/behavior/config/mission.yaml` on its own (see [IMU and autonomous navigation](#imu-and-autonomous-navigation)). Keep the default start pose and do not run `wasd_teleop` at the same time: the mission stops if another node publishes to `/cmd_vel_raw`.

### Launch arguments

| Argument | Default | Description |
| --- | --- | --- |
| `mission` | `false` | Run the autonomous greenhouse mission |
| `rviz` | `true` | Open RViz |
| `headless` | `false` | Run the Gazebo server only (no Gazebo GUI) |
| `x`, `y`, `yaw` | `-0.5`, `0.65`, `0.0` | Robot start pose (aisle 1, middle row on the right) |
| `world` | `greenhouse.sdf` | Gazebo world file |

RViz shows the robot model, TF, odometry, the raw camera image, the color detection image, the LiDAR scan and the detected obstacles.

To open only the world: `gz sim -r src/bringup/worlds/greenhouse.sdf`

More commands for debugging and for the report figures (topic graph, TF tree, ...) are in [`COMMANDS.md`](COMMANDS.md).

## RGB camera and color detection

The camera sits on a mast at the right edge of the chassis, ~0.40 m above the ground, facing **right** (640×480, 15 Hz, 1.3 rad horizontal field of view).
Driving along an aisle it sees the apples on the row on the robot's right; a wall checkpoint is seen when the robot turns its right side to the wall.
Its direction can be changed with `camera_yaw` in `urdf/camera.xacro` (`0` = looks forward).

The `color_detector` node starts automatically with `sim.launch.py`. HSV masks separate the **color** (red / green)
and the contour shape separates the **kind**: a round blob = apple, a 4-corner convex blob (a rectangle, or a trapezoid when seen at an angle) = checkpoint panel.
Foliage (S ≈ 110) and brown planters (S ≈ 95) are rejected by the saturation threshold.

| Topic | Type | Description |
| --- | --- | --- |
| `/camera/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | Raw camera image |
| `/color_detector/red_apple` | `geometry_msgs/PointStamped` | Red (ripe) apple |
| `/color_detector/green_apple` | `geometry_msgs/PointStamped` | Green (unripe) apple |
| `/color_detector/red_checkpoint` | `geometry_msgs/PointStamped` | RED panel on a wall |
| `/color_detector/green_checkpoint` | `geometry_msgs/PointStamped` | GREEN panel on a wall |
| `/color_detector/debug_image` | `sensor_msgs/Image` | Image with the detections drawn + mask in the top-right corner |

`PointStamped` messages are published only while the object is seen (the largest blob for each label):
`point.x` = horizontal offset from the image center (−1 left … +1 right), `point.y` = vertical (−1 top … +1 bottom),
`point.z` = fraction of the image covered by the object (grows as the robot gets closer).

~~~bash
ros2 run rqt_image_view rqt_image_view /color_detector/debug_image
ros2 topic echo /color_detector/red_apple
~~~

## LiDAR and obstacle detection

A 360° 2D LiDAR (`urdf/lidar.xacro`) sits at the front of the top plate, scan plane ~0.225 m above the ground
(below the planter tops, so rows, walls and boxes are all seen). 360 rays, 0.12–8 m, 10 Hz, frame `lidar_link`.

The `obstacle_detector` node starts automatically with `sim.launch.py`. It drops the points on the robot's own body,
splits the scan into obstacles (clusters of neighbouring points) and checks the **corridor** straight ahead
(the strip the robot will drive through, ±0.25 m): the nearest point inside it decides the state
`STOP` (< 0.25 m), `WARN` (< 1.0 m) or `CLEAR`. Parameters are in `src/bringup/config/obstacle_detector.yaml`.

| Topic | Type | Description |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | Raw LiDAR scan |
| `/obstacle_detector/state` | `std_msgs/String` | `CLEAR`, `WARN` or `STOP` for the corridor ahead |
| `/obstacle_detector/front_distance` | `std_msgs/Float32` | Nearest point in the corridor (m), `inf` if none |
| `/obstacle_detector/markers` | `visualization_msgs/MarkerArray` | RViz: corridor (green / orange / red by state), obstacles (red in the corridor, cyan outside) and their distances |

~~~bash
ros2 topic hz /scan                              # ~10 Hz
ros2 topic echo /obstacle_detector/state
ros2 topic echo /obstacle_detector/front_distance
~~~

## Bumper and safety filter

A thin bumper plate (`urdf/bumper.xacro`) covers the chassis front and the front wheels, ~0.055–0.135 m above the
ground: below the LiDAR scan plane, so it catches low obstacles the LiDAR cannot see. A Gazebo contact sensor
reports every touch (it needs the `Contact` system plugin at the top of `greenhouse.sdf`).

Both the teleop node and the mission publish **`/cmd_vel_raw`**. The `safety` node, started by `sim.launch.py`,
passes it on to `/cmd_vel` and removes forward motion (`linear.x > 0`) while

- `BUMPER`: the bumper touched something less than `bumper_hold_time` (1 s) ago, or
- `OBSTACLE`: `obstacle_detector` reports `STOP` (turn off with `obstacle_stop: false`).

Reversing and turning in place are always allowed, so the robot can back away.
To try the bumper, turn around at the start pose and drive into the low orange box ~1 m behind it. Parameters are in
`src/bringup/config/safety.yaml`. Anything that publishes straight to `/cmd_vel` bypasses the filter.

| Topic | Type | Description |
| --- | --- | --- |
| `/bumper` | `ros_gz_interfaces/Contacts` | Bumper contacts, published only while something touches it |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | Unfiltered commands (teleop or mission) |
| `/cmd_vel` | `geometry_msgs/Twist` | Safe command sent to the robot |
| `/safety/stop` | `std_msgs/Bool` | `True` while forward motion is blocked |

~~~bash
ros2 topic echo /safety/stop
ros2 topic echo /bumper --field contacts
~~~

## IMU and autonomous navigation

An IMU (`urdf/imu.xacro`, frame `imu_link`, 30 Hz) is bridged to ROS 2 as `/imu`.

The `greenhouse_mission` node (started with `mission:=true`) runs a single state machine (`mission.py`, `mission_node.py`) over the 13 stages in `src/behavior/config/mission.yaml`:

- drive through the first aisle and watch for apples,
- turn toward the checkpoint panels and scan them,
- cross to the second aisle and drive back through it,
- return to the start position and park.

Navigation features:

- IMU-based heading tracking (quaternion-to-yaw conversion)
- Proportional control for accurate turning
- Waypoint navigation with odometry
- Aisle following with lateral drift correction
- Automatic speed reduction near target positions
- Runs through the `safety` filter: while `/safety/stop` is `True` the mission waits; if the stop lasts longer than 30 s the mission fails

| Topic | Type | Direction |
| --- | --- | --- |
| `/imu` | `sensor_msgs/Imu` | subscribed: heading |
| `/odom` | `nav_msgs/Odometry` | subscribed: position |
| `/safety/stop` | `std_msgs/Bool` | subscribed: pause while blocked |
| `/color_detector/*` | `geometry_msgs/PointStamped` | subscribed: detections logged during the mission |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | published: drive command (filtered by `safety`) |

Results in Gazebo:

- All 13 mission stages completed.
- Both the red and the green checkpoint were detected.
- Final position error: approximately 0.05 m; final heading error: approximately 3 degrees.

## Tests and CI

The decision logic of every node is plain Python without ROS, so it is unit tested without Gazebo: 40 tests in `src/behavior/test/` (color detection 7, obstacle detection 7, safety 8, navigation 8, mission 10).

~~~bash
python3 -m pytest -v src/behavior/test
~~~

Every pull request and every push to `main` runs two GitHub Actions checks (`.github/workflows/ci.yml`):

- `lint`: `ruff check` and `ruff format --check` (rules in `ruff.toml`)
- `build-and-test`: `colcon build` and the unit tests in a clean `ros:lyrical-ros-base` container

`main` is protected: changes go through a pull request, both checks must pass and one teammate must approve. Run the same checks locally before opening a pull request:

~~~bash
pipx run ruff check src/ && pipx run ruff format --check src/ && python3 -m pytest -v src/behavior/test
~~~

The team workflow (branches, pull requests, reviews) is described in [`CONTRIBUTING.md`](CONTRIBUTING.md).
