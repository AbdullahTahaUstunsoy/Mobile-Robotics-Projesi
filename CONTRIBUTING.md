# PR Rules for Devs

Rules for everyone working on this repo. Please read before your first commit.

## Golden rules

1. **Never push directly to `main`.** `main` must always build and run.
2. **One branch per task.** Keep branches small and merge them quickly.
3. **Every change goes through a Pull Request** and is reviewed by at least one teammate.
4. **Pull `main` often** so you don't drift away from the others.

## Branch naming

| Prefix      | Use for                       | Example                      |
|-------------|-------------------------------|------------------------------|
| `feature/`  | New functionality             | `feature/lidar`              |
| `fix/`      | Bug fixes                     | `fix/camera-frame-id`        |
| `docs/`     | Documentation only            | `docs/pr-rules-for-devs`     |

## Daily workflow

```bash
# 1. Start from an up-to-date main
git checkout main
git pull

# 2. Create your branch
git checkout -b feature/<task-name>

# 3. Work, then commit
git status
git add <file1> <file2>
git commit -m "Short, clear message"

# 4. Push (-u only on the first push)
git push -u origin feature/<task-name>
git push

# 5. Get the latest main into your branch when needed
git pull origin main
```

Then open a Pull Request on GitHub ("Compare & pull request" button).

## Useful commands

```bash
git branch              # list local branches (* = current)
git branch -a           # list local + remote branches
git checkout <branch>   # switch to an existing branch
git log --oneline -10   # last 10 commits
```

## Commits

- Write messages in English, in the imperative: `Add LiDAR to URDF`, not `added lidar`.
- One logical change per commit.
- Do not commit `build/`, `install/`, `log/` or `__pycache__/` (already in `.gitignore`).
- Use `git add <files>` instead of `git add .` so you don't commit things by accident.

## Check-in rules

Go through this list before every push and before asking for review. A PR that fails any item is not merged.

**Before you push**

- [ ] Your branch is up to date with `main` (`git pull origin main`), and conflicts are resolved.
- [ ] Clean build passes with no errors:
      `rm -rf build install log && colcon build --symlink-install`
- [ ] Unit tests pass: `python3 -m pytest -q src/behavior/test`
- [ ] The simulation still launches and the robot drives with `wasd_teleop`:
      `ros2 launch bringup sim.launch.py`
- [ ] Your new topics actually publish (`ros2 topic list`, `ros2 topic echo <topic> --once`).
- [ ] `git status` shows only the files you meant to change: no `build/`, `install/`, `log/`,
      `__pycache__/`, recordings or personal config files.
- [ ] No debug leftovers: commented-out code, temporary `print`s, hard-coded paths from your machine.

**Code**

- [ ] Each sensor is in its own xacro file; each node is in its own Python file.
- [ ] New nodes are registered in `setup.py` (`console_scripts`), and new dependencies in `package.xml`.
- [ ] Tunable values (speeds, distances, thresholds) are ROS parameters, not magic numbers.
- [ ] Nodes that use simulation time set `use_sim_time: True`.
- [ ] Nobody else's code was changed without telling them.

**Docs**

- [ ] New or changed topics are added to the [Topic contract](#topic-contract) table.
- [ ] New commands are added to `COMMANDS.md`, and `README.md` is updated if the package contents changed.

**Merging**

- [ ] At least one teammate approved the PR.
- [ ] The reviewer pulled the branch and ran the simulation, not just read the diff.
- [ ] Merge only to a working `main`. If `main` breaks after a merge, fixing it comes before any other work.

## Pull Requests

- Title: short summary of the change.
- Description: what changed, how to test it, and `Closes #<issue>` if it fixes an issue.
- Copy the [Check-in rules](#check-in-rules) list into the description and tick the items.
- The author merges after approval, then deletes the branch.

## Where things go

```
src/
├── description/                        # what the robot IS (ament_cmake)
│   ├── urdf/
│   │   ├── greenhouse_bot.urdf.xacro   # main model: chassis, wheels, one include per sensor
│   │   ├── gazebo.xacro                # robot-wide gz plugins (DiffDrive, joint states)
│   │   ├── inertial_macros.xacro       # shared inertia macros
│   │   ├── camera.xacro                # RGB camera (Dev 2)
│   │   └── <sensor>.xacro              # lidar, bumper (Dev 1), imu (Dev 3)
│   ├── meshes/                         # 3D models (.stl/.dae), if any
│   ├── rviz/greenhouse_bot.rviz        # RViz config
│   ├── CMakeLists.txt                  # installs urdf/, meshes/, rviz/
│   └── package.xml
├── bringup/                            # how the system STARTS (ament_cmake)
│   ├── launch/sim.launch.py            # Gazebo + robot + bridge + nodes + RViz
│   ├── config/bridge.yaml              # gz <-> ROS topic bridge; node parameter YAMLs go here too
│   ├── worlds/greenhouse.sdf           # walls, rows, fruit, checkpoint panels, world plugins
│   ├── CMakeLists.txt                  # installs launch/, config/, worlds/
│   └── package.xml
└── behavior/                           # what the robot DOES: the ROS package (ament_python)
    ├── behavior/                       # the Python module: `from behavior.<name> import ...`
    │   ├── __init__.py
    │   ├── color_detection.py          # ROS-free logic (pure Python/OpenCV), unit tested
    │   ├── color_detector_node.py      # ROS node that wraps color_detection
    │   └── wasd_teleop_node.py         # ROS node: keyboard -> /cmd_vel
    ├── test/test_color_detection.py    # pytest tests for the ROS-free logic
    ├── resource/behavior               # empty marker file, lets ROS find the package
    ├── setup.py                        # every node is registered under console_scripts
    ├── setup.cfg
    └── package.xml
```

`behavior/behavior/` is not a duplicate: the outer folder is the ROS package, the inner one is the
Python module you import. Every ROS 2 Python package has this layout, so do not rename or flatten it.

Naming: `<name>.py` holds the logic with no `rclpy` imports, `<name>_node.py` is the ROS node.
The executable name in `setup.py` drops the `_node` suffix (`ros2 run behavior color_detector`).

| You are adding...                                    | Put it in                                                    |
|------------------------------------------------------|--------------------------------------------------------------|
| A new sensor (link, joint, `<sensor>` tag)           | `description/urdf/<sensor>.xacro` + one `<xacro:include>` in `greenhouse_bot.urdf.xacro` |
| A gz system plugin for the robot                     | `description/urdf/gazebo.xacro`                              |
| A gz system plugin for the world (e.g. contact, imu) | `<plugin>` at the top of `bringup/worlds/greenhouse.sdf`     |
| Objects in the world (panels, obstacles)             | `bringup/worlds/greenhouse.sdf`                              |
| A gz ↔ ROS topic bridge                              | `bringup/config/bridge.yaml`, under `# --- Sensors ---`      |
| Node parameters (speeds, thresholds)                 | `bringup/config/<node>.yaml`                                 |
| Starting a node with the sim                         | `bringup/launch/sim.launch.py`                               |
| A new ROS node                                       | `behavior/behavior/<name>_node.py` + entry in `setup.py`     |
| Algorithm code (detection, safety logic, path math)  | `behavior/behavior/<name>.py`, no `rclpy` imports            |
| Tests                                                | `behavior/test/test_<name>.py`                               |
| A new dependency (e.g. `sensor_msgs`, `tf2_ros`)     | `package.xml` of the package that uses it                    |
| RViz display changes                                 | `description/rviz/greenhouse_bot.rviz`                       |
| Screenshots, videos, diagrams for the report         | `docs/media/`                                                |
| New commands / how-to                                | `COMMANDS.md`; package overview in `README.md`               |

Planned files per developer:

| Dev   | Files                                                                                                         |
|-------|---------------------------------------------------------------------------------------------------------------|
| Dev 1 | `urdf/lidar.xacro`, `urdf/bumper.xacro`, checkpoint panels in `greenhouse.sdf`, `safety.py`, `safety_node.py` |
| Dev 2 | `urdf/camera.xacro`, `color_detection.py`, `color_detector_node.py`                                           |
| Dev 3 | `urdf/imu.xacro`, `navigation.py`, `navigation_node.py`                                                       |

Do not put files in `build/`, `install/` or `log/`: colcon generates them and they are not committed.

## Avoiding merge conflicts

These files are shared by everyone, so keep your changes to them minimal:

- `src/description/urdf/greenhouse_bot.urdf.xacro`
- `src/description/urdf/gazebo.xacro`
- `src/bringup/config/bridge.yaml`
- `src/bringup/launch/sim.launch.py`
- `src/behavior/setup.py`
- `src/bringup/worlds/greenhouse.sdf`

Put each sensor in its own xacro file (`lidar.xacro`, `imu.xacro`, `bumper.xacro`, like `camera.xacro`)
and only add a one-line `<xacro:include>` to the main URDF. Put each ROS node in its own Python file.

## Topic contract

Agree on topic names and message types before coding, and update this list when they change:

| Topic                     | Type                      | Owner   |
|---------------------------|---------------------------|---------|
| `/scan`                   | `sensor_msgs/LaserScan`   | Dev 1   |
| `/bumper`                 | `ros_gz_interfaces/Contacts` | Dev 1 |
| `/safety/stop`            | `std_msgs/Bool`           | Dev 1   |
| `/camera/image_raw`       | `sensor_msgs/Image`       | Dev 2   |
| `/color_detector/<label>` (`red_apple`, `green_apple`, `red_checkpoint`, `green_checkpoint`) | `geometry_msgs/PointStamped` | Dev 2 |
| `/color_detector/debug_image` | `sensor_msgs/Image`   | Dev 2   |
| `/imu`                    | `sensor_msgs/Imu`         | Dev 3   |
| `/odom`                   | `nav_msgs/Odometry`       | Dev 3   |
