# Mobile-Robotics-Projesi

A Gazebo Sim greenhouse world for ripe fruit (apple) detection and a 4-wheel robot driven with the keyboard (WASD). The robot currently has an RGB camera (red/green apple and RED/GREEN checkpoint detection), a 2D lidar (obstacle detection) and a front bumper (contact sensor) behind a safety filter; the other sensors are being added.

## Packages

| Package | Contents |
| --- | --- |
| `bringup` | `sim.launch.py`, `worlds/greenhouse.sdf`, `config/bridge.yaml`, `config/obstacle_detector.yaml`, `config/safety.yaml` |
| `description` | Robot model (`urdf/greenhouse_bot.urdf.xacro`): 4-wheel skid-steer base, Gazebo DiffDrive plugin, `urdf/camera.xacro`, `urdf/lidar.xacro`, `urdf/bumper.xacro`; RViz config `rviz/greenhouse_bot.rviz` |
| `behavior` | `wasd_teleop`: teleop node that publishes `/cmd_vel_raw` from the keyboard; `color_detector`: apple and checkpoint detection in the camera image; `obstacle_detector`: obstacle detection in the lidar scan; `safety`: filters `/cmd_vel_raw` into `/cmd_vel` using the bumper and the lidar |

## World: `src/bringup/worlds/greenhouse.sdf`

- 3 planter rows (y = 1.4, 0.0, -1.4) with 2 aisles between them
- 9 fruit on the middle row: 5 red (ripe), 4 green (unripe)
- An 11 x 5.4 m area enclosed by four walls
- A low orange box (0.1 m tall, at x = -1.5, y = 0.65, behind the start pose) for bumper tests: below the lidar scan plane, only the bumper detects it
- No external models; runs as a single file

## Setup

### Requirements

- Ubuntu + **ROS 2 Lyrical** (`/opt/ros/lyrical`) and Gazebo Sim
- `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`, `colcon`

Missing packages on a system with ROS 2 installed:

~~~bash
source /opt/ros/lyrical/setup.bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-rosdep \
  ros-lyrical-ros-gz-sim ros-lyrical-ros-gz-bridge \
  ros-lyrical-robot-state-publisher ros-lyrical-xacro
~~~

### Cloning the repo

The repo is **private**: the owner first has to invite you on GitHub under *Settings → Collaborators*, and you have to accept the invitation.

~~~bash
git clone https://github.com/AbdullahTahaUstunsoy/Mobile-Robotics-Projesi.git ~/Mobile-Robotics-Projesi
cd ~/Mobile-Robotics-Projesi
~~~

The `build/`, `install/` and `log/` folders come empty from the repo; they fill up during the build and their contents are not pushed to git.

### Checking dependencies (optional)

~~~bash
sudo rosdep init        # first time only; skip it if it says "already exists"
rosdep update
rosdep install --from-paths src --ignore-src -r -y
~~~

## Build

~~~bash
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

Run `source /opt/ros/lyrical/setup.bash && source install/setup.bash` in every new terminal.

## Run

Terminal 1 — Gazebo, robot, bridge, color and obstacle detectors, safety filter and RViz:

~~~bash
ros2 launch bringup sim.launch.py
~~~

Terminal 2 — keyboard driving (this terminal must be focused):

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

RViz shows the robot model, TF, odometry, the raw camera image, the color detection image, the lidar scan and the detected obstacles.

Launch arguments: `headless:=true` (no Gazebo GUI), `rviz:=false` (do not open RViz), `x`, `y`, `yaw` (start pose, default aisle 1: `-0.5`, `0.65`, `0.0`), `world` (a different world file).

To open only the world: `gz sim -r src/bringup/worlds/greenhouse.sdf`

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
`point.z` = fraction of the image covered by the object (grows as the robot gets closer). The navigation side can, for example, treat the object as "not seen" if no message arrives for ~0.5 s.

Suggested checkpoint panels: a thin box against the wall (e.g. 0.02 × 0.4 × 0.3 m, center at z ≈ 0.45 m),
colors `0.9 0.05 0.05` (red) and `0.05 0.8 0.1` (green). Detection was tested with these values.

~~~bash
ros2 run rqt_image_view rqt_image_view /color_detector/debug_image
ros2 topic echo /color_detector/red_apple
colcon test --packages-select behavior && colcon test-result --verbose   # unit tests
~~~

## Lidar and obstacle detection

A 360° 2D lidar (`urdf/lidar.xacro`) sits at the front of the top plate, scan plane ~0.23 m above the ground
(below the planter tops, so rows, walls and low obstacles are all seen). 360 rays, 0.12–8 m, 10 Hz, frame `lidar_link`.

The `obstacle_detector` node starts automatically with `sim.launch.py`. It drops the points on the robot's own body,
splits the scan into obstacles (clusters of neighbouring points) and checks the **corridor** straight ahead
(the strip the robot will drive through, ±0.25 m): the nearest point inside it decides the state
`STOP` (< 0.25 m), `WARN` (< 1.0 m) or `CLEAR`. Parameters are in `src/bringup/config/obstacle_detector.yaml`.

| Topic | Type | Description |
| --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | Raw lidar scan |
| `/obstacle_detector/state` | `std_msgs/String` | `CLEAR`, `WARN` or `STOP` for the corridor ahead |
| `/obstacle_detector/front_distance` | `std_msgs/Float32` | Nearest point in the corridor (m), `inf` if none |
| `/obstacle_detector/markers` | `visualization_msgs/MarkerArray` | RViz: corridor (green / orange / red by state), obstacles (red in the corridor, cyan outside) and their distances |

~~~bash
ros2 topic echo /obstacle_detector/state
ros2 topic echo /obstacle_detector/front_distance
python3 -m pytest -q src/behavior/test   # unit tests
~~~

## Bumper and safety filter

A thin bumper plate (`urdf/bumper.xacro`) covers the chassis front and the front wheels, ~0.055–0.135 m above the
ground: below the lidar scan plane, so it catches low obstacles the lidar cannot see. A Gazebo contact sensor
reports every touch (it needs the `Contact` system plugin at the top of `greenhouse.sdf`).

Teleop (and later navigation) publishes **`/cmd_vel_raw`**; the `safety` node, started by `sim.launch.py`,
passes it on to `/cmd_vel` and removes forward motion (`linear.x > 0`) while

- `BUMPER`: the bumper touched something less than `bumper_hold_time` (1 s) ago, or
- `OBSTACLE`: `obstacle_detector` reports `STOP` (turn off with `obstacle_stop: false`).

Reversing and turning in place are always allowed, so the robot can back away.
To try the bumper, turn around at the start pose and drive into the low orange box ~1 m behind it. Parameters are in
`src/bringup/config/safety.yaml`. Anything that publishes straight to `/cmd_vel` bypasses the filter.

| Topic | Type | Description |
| --- | --- | --- |
| `/bumper` | `ros_gz_interfaces/Contacts` | Bumper contacts, published only while something touches it |
| `/cmd_vel_raw` | `geometry_msgs/Twist` | Unfiltered commands (teleop, navigation) |
| `/cmd_vel` | `geometry_msgs/Twist` | Safe command sent to the robot |
| `/safety/stop` | `std_msgs/Bool` | `True` while forward motion is blocked |

~~~bash
ros2 topic echo /safety/stop
ros2 topic echo /bumper --field contacts
~~~
