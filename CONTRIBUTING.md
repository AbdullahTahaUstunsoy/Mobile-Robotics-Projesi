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

## Pull Requests

- Title: short summary of the change.
- Description: what changed, how to test it, and `Closes #<issue>` if it fixes an issue.
- Make sure `colcon build` passes and the simulation still launches before asking for review.
- The author merges after approval, then deletes the branch.

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
