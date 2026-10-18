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
- [ ] Unit tests pass: `python3 -m pytest -q src/midterm_behavior/test`
- [ ] The simulation still launches and the robot drives with `wasd_teleop`:
      `ros2 launch midterm_bringup sim.launch.py`
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
├── midterm_description/        # what the robot IS
│   ├── urdf/
│   │   ├── greenhouse_bot.urdf.xacro   # main model: chassis, wheels, sensor includes
│   │   ├── gazebo.xacro                # robot-wide gz plugins (DiffDrive, joint states)
│   │   ├── inertial_macros.xacro       # shared inertia macros
│   │   └── <sensor>.xacro              # one file per sensor: camera, lidar, imu, bumper
│   ├── meshes/                 # 3D models (.stl/.dae), if any
│   └── rviz/                   # RViz configs
├── midterm_bringup/            # how the system STARTS
│   ├── launch/                 # *.launch.py files
│   ├── config/                 # YAML: bridge.yaml, node parameter files
│   └── worlds/                 # Gazebo .sdf worlds (walls, rows, fruit, checkpoint panels)
└── midterm_behavior/           # what the robot DOES (all Python nodes)
    ├── midterm_behavior/
    │   ├── <name>.py           # ROS-free logic (pure Python/OpenCV), unit tested
    │   └── <name>_node.py      # thin ROS node that wraps the logic and handles topics
    ├── test/test_<name>.py     # pytest tests for the ROS-free logic
    └── setup.py                # register every node under console_scripts
```

| You are adding...                          | Put it in                                         |
|--------------------------------------------|---------------------------------------------------|
| A new sensor (link, joint, `<sensor>` tag) | `midterm_description/urdf/<sensor>.xacro` + one `<xacro:include>` in `greenhouse_bot.urdf.xacro` |
| A gz system plugin for the robot           | `midterm_description/urdf/gazebo.xacro`           |
| A gz system plugin for the world (e.g. contact, imu) | `<plugin>` at the top of `midterm_bringup/worlds/greenhouse.sdf` |
| Objects in the world (panels, obstacles)   | `midterm_bringup/worlds/greenhouse.sdf`           |
| A gz ↔ ROS topic bridge                    | `midterm_bringup/config/bridge.yaml`, under `# --- Sensors ---` |
| Node parameters (speeds, thresholds)       | `midterm_bringup/config/<node>.yaml`              |
| Starting a node with the sim               | `midterm_bringup/launch/sim.launch.py`            |
| A new ROS node                             | `midterm_behavior/midterm_behavior/<name>_node.py` + entry in `setup.py` |
| Algorithm code (detection, safety logic, path math) | `midterm_behavior/midterm_behavior/<name>.py`, no `rclpy` imports |
| Tests                                      | `midterm_behavior/test/test_<name>.py`            |
| RViz display changes                       | `midterm_description/rviz/greenhouse_bot.rviz`    |
| Screenshots, videos, diagrams for the report | `docs/media/`                                   |
| New commands / how-to                      | `COMMANDS.md`; package overview in `README.md`    |

Planned files per developer:

| Dev   | Files                                                                                     |
|-------|-------------------------------------------------------------------------------------------|
| Dev 1 | `urdf/lidar.xacro`, `urdf/bumper.xacro`, checkpoint panels in `greenhouse.sdf`, `safety.py`, `safety_node.py` |
| Dev 2 | `urdf/camera.xacro`, `color_detection.py`, `color_detector_node.py`                       |
| Dev 3 | `urdf/imu.xacro`, `navigation.py`, `navigation_node.py`                                   |

Do not put files in `build/`, `install/` or `log/`: colcon generates them and they are not committed.

## Avoiding merge conflicts

These files are shared by everyone, so keep your changes to them minimal:

- `src/midterm_description/urdf/greenhouse_bot.urdf.xacro`
- `src/midterm_description/urdf/gazebo.xacro`
- `src/midterm_bringup/config/bridge.yaml`
- `src/midterm_bringup/launch/sim.launch.py`
- `src/midterm_behavior/setup.py`
- `src/midterm_bringup/worlds/greenhouse.sdf`

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
