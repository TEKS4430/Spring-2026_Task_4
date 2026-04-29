# Spring-2026_Task_4

## Part A

# Assignment 2 — Arm Control
### TEKS4430 Engineering Autonomous Cyber-Physical Systems

> **Webots R2025a** runs on your host OS.  
> **ROS2 Jazzy** runs inside Docker.  
> Your code lives in `src/` — edit on host, build and run in container.

---

## Quick start

### 1. First-time setup

```bash
# Copy environment config for your OS
cp .env.example .env
# Linux:        uncomment NETWORK_MODE=host and WEBOTS_HOST=localhost
# Mac/Windows:  leave as-is (bridge mode is the default)

# Build the Docker image (takes ~5 min first time)
docker compose build
```

### 2. Open Webots on your host

1. Download Webots R2025a from https://cyberbotics.com if not installed
2. Open Webots → **File → Open World** → navigate to `worlds/arm_assignment.wbt`
3. Press **Play ▶** — the simulation must be running before launching the driver

> ⚠️ **Always press Play in Webots before running the driver.**
> If you launch first, the driver will print:
> `"The specified robot is not in the list of robots with <extern> controllers, retrying..."`
> This is normal — it retries for 50 seconds. Just press Play and it will connect automatically.

### 3. Enter the container

```bash
docker compose run --rm ros2 bash
```

Inside the container you have these aliases:

| Alias | Command |
|-------|---------|
| `cb`  | `colcon build --symlink-install` |
| `cbs` | `colcon build --packages-select <pkg>` |
| `src` | `source install/setup.bash` |
| `rt`  | `ros2 topic list` |
| `rn`  | `ros2 node list` |

### 4. Build the package

```bash
cb && src
```

---

## Running the tasks

> ⚠️ **The driver must be running in Terminal 1 for all tasks.**
> Sensor topics only exist while the driver is alive. If `angle=0.0` never changes,
> check that the driver is running and Webots is playing.

### Task 1 — Forward Kinematics

```bash
# Terminal 1 — start the Webots driver (keep this open the whole time)
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — run the FK explorer
ros2 run arm_control_pkg task1_fk
```

Edit `joint_index` and `target_angle` at the bottom of `task1_fk_explorer.py`,
then rebuild and run again for each experiment. You do **not** need to stop Terminal 1.

```bash
# After editing:
cb && src
ros2 run arm_control_pkg task1_fk
```

**Expected output:**
```
[fk_explorer]: Start EE:   x=0.4610  y=0.0000  z=0.1470
[fk_explorer]: Command sent: joint 1 → 1.000 rad
[fk_explorer]: Current EE: x=0.2491  y=0.3879  z=0.1470
```

---

### Task 2 — PID Controller

Run one experiment at a time (P only → PD → PID). For each run use three terminals:

```bash
# Terminal 1 — Webots driver (keep open across all runs)
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — your PID controller
ros2 run arm_control_pkg task2_pid

# Terminal 3 — data logger (start after Terminal 2, stop with Ctrl+C together)
ros2 run arm_control_pkg data_logger --ros-args -p label:=P_only
```

Change `label` for each run: `P_only`, `PD`, `PID`.

After all three runs, generate the comparison plot:

```bash
ros2 run arm_control_pkg plotter
# Plot saved to: src/arm_control_pkg/logs/task2_plot.png
```

**Expected output (every 2 seconds):**
```
[pid_joint_controller]: angle=0.3241  error=0.6759  correction=0.0000
```

---

### Task 3 — Pick and Place

```bash
# Terminal 1 — Webots driver
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — state machine
ros2 run arm_control_pkg task3_pick
```

Run **twice** by editing `DWELL_TIME` at the top of `task3_pick_place.py`:
- **Run 1:** `DWELL_TIME = 0.0` → observe state skipping
- **Run 2:** `DWELL_TIME = 1.5` → observe smooth sequence

Rebuild between runs:
```bash
cb && src
ros2 run arm_control_pkg task3_pick
```

---

## Verifying the setup

After `ros2 launch arm_control_pkg arm.launch.py`, confirm the driver loaded:

```bash
# In a second terminal — check all arm topics are live
rt | grep youBot
```

Expected topics:
```
/youBot/arm_joint_1/position    /youBot/arm_joint_1/sensor
/youBot/arm_joint_2/position    /youBot/arm_joint_2/sensor
/youBot/arm_joint_3/position    /youBot/arm_joint_3/sensor
/youBot/arm_joint_4/position    /youBot/arm_joint_4/sensor
/youBot/arm_joint_5/position    /youBot/arm_joint_5/sensor
/youBot/end_effector_position
/youBot/gripper_left/position
/youBot/gripper_right/position
```

Test a live command — the arm should visibly move in Webots:
```bash
ros2 topic pub --once /youBot/arm_joint_1/position std_msgs/Float64 "data: 1.0"
```

---

## Submission

Push your completed code from your **host machine** (not inside the container):

```bash
git add src/
git commit -m "Assignment 2 complete"
git push origin main
```

Submit to Moodle:
- GitHub repo link
- PDF report with plots and answers to all task questions

---

## Troubleshooting

**`ros2 topic list` shows no `/youBot/...` topics**  
→ The driver is not running. Start it with `ros2 launch arm_control_pkg arm.launch.py` in Terminal 1.  
→ Also check that Webots is open, the world is loaded, and simulation is playing (▶).

**`angle=0.0000` never changes in task2**  
→ The driver is not running in a separate terminal. You need Terminal 1 (driver) and Terminal 2 (task) open simultaneously.

**`retrying for another 50 seconds...` in launch output**  
→ Normal. Webots simulation was not playing when the driver started. Press Play ▶ in Webots and the driver will connect automatically within seconds.

**Segmentation fault when stopping the driver (Ctrl+C)**  
→ Known bug in `webots_ros2_driver` — not caused by your code. The driver still shuts down correctly. Safe to ignore.

**`colcon: command not found`**  
→ Run `source /opt/ros/jazzy/setup.bash` then retry.

**Container can't connect to Webots (Mac/Windows)**  
→ Check `.env` has `WEBOTS_HOST=host.docker.internal`.

**Mac Apple Silicon — build fails**  
→ Docker Desktop → Settings → General → enable **"Use Rosetta for x86/amd64 emulation"**.

**Changes not taking effect after editing a task file**  
→ With `--symlink-install`, Python files don't need a full rebuild. Run `src` to re-source.  
→ If still stale, run `cb && src`.

**`rcl_shutdown already called` error on Ctrl+C**  
→ Cosmetic — ROS2 cleans up the context before the `finally` block runs. Safe to ignore.

---

*ROS2 Jazzy · Webots R2025a · Docker*


## Part B

Report how you have used AI


## Returning instructions

- Create a video that reports Parts A and B, so demonstrates that you have got the environment up and running, implemented teleoperation, and reports how you have used AI in the assignment
