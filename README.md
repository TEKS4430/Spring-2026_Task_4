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
> If you launch the driver first, it will print:  
> `"The specified robot is not in the list of robots with <extern> controllers, retrying..."`  
> This is normal — just press Play and it connects automatically within seconds.
 
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
> Sensor topics only exist while the driver is alive.  
> If `angle=0.000` never changes, the driver is not running.
 
---
 
### Task 1 — Forward Kinematics
 
```bash
# Terminal 1 — start the driver (keep this open the whole session)
ros2 launch arm_control_pkg arm.launch.py
 
# Terminal 2 — run the FK explorer
ros2 run arm_control_pkg task1_fk
```
 
Edit `joint_index` and `target_angle` at the bottom of `task1_fk_explorer.py`,
rebuild, and run again for each experiment:
 
```bash
# After editing the file on your host:
cb && src
ros2 run arm_control_pkg task1_fk
```
 
**Expected output:**
```
[fk_explorer]: Start EE:   x=0.4610  y=0.0000  z=0.1470
[fk_explorer]: Command sent: joint 1 → 1.000 rad
[fk_explorer]: Current EE: x=0.2491  y=0.3879  z=0.1470
```
 
Wait until the EE values stop changing before recording them in your table.
 
---
 
### Task 2 — PID Controller (Sine Wave Tracking)
 
Joint 2 follows a continuously moving sine wave target. Your controller
must keep the arm tracking it. Watch the arm move in Webots as you add
each PID term — the tracking improves visibly with each step.
 
Run one experiment at a time: **P only → PD → PID**.
 
For each run open three terminals:
 
```bash
# Terminal 1 — Webots driver (keep open across all runs)
ros2 launch arm_control_pkg arm.launch.py
 
# Terminal 2 — your PID controller
ros2 run arm_control_pkg task2_pid
 
# Terminal 3 — data logger
ros2 run arm_control_pkg data_logger --ros-args -p label:=P_only
```
 
Let it run for **~15 seconds** (about 3 full sine cycles), then stop
Terminal 2 and Terminal 3 with Ctrl+C. Change `label` for each run:
 
| Run | Label |
|-----|-------|
| P only | `P_only` |
| P + D  | `PD` |
| P + I + D | `PID` |
 
After all three runs, generate the plot **inside the container**:
 
```bash
ros2 run arm_control_pkg plotter
```
 
Then **open the plot on your host machine** — no GUI needed in Docker:
```
src/arm_control_pkg/logs/task2_plot.png
```
The `src/` folder is shared between the container and your host, so the
file appears there immediately after the plotter runs.
 
**Expected log output (every 2 seconds):**
```
[pid_joint_controller]: target=0.650  angle=0.412  error=0.238  correction=0.000
```
The `error` value should shrink as you add terms. With a well-tuned PID,
error stays close to zero even as the target keeps moving.
 
---
 
### Task 3 — Pick and Place State Machine
 
```bash
# Terminal 1 — Webots driver
ros2 launch arm_control_pkg arm.launch.py
 
# Terminal 2 — state machine
ros2 run arm_control_pkg task3_pick
```
 
Run **twice** by editing `DWELL_TIME` at the top of `task3_pick_place.py`:
 
| Run | Setting | What to observe |
|-----|---------|-----------------|
| 1 | `DWELL_TIME = 0.0` | Arm skips states mid-motion |
| 2 | `DWELL_TIME = 1.5` | Arm completes each state smoothly |
 
Rebuild between runs:
```bash
cb && src
ros2 run arm_control_pkg task3_pick
```
 
---
 
## Verifying the full setup
 
After `ros2 launch arm_control_pkg arm.launch.py`, check all arm topics
are live in a second terminal:
 
```bash
rt | grep youBot
```
 
Expected:
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
 
Test a live command — the arm should visibly rotate in Webots:
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
→ The driver is not running. Start it with `ros2 launch arm_control_pkg arm.launch.py`.  
→ Also check Webots is open, the world is loaded, and simulation is playing ▶.
 
**`angle=0.000` never changes in task2**  
→ The driver is not running in a separate terminal. You need Terminal 1 (driver)
and Terminal 2 (task) open simultaneously.
 
**`retrying for another 50 seconds...` in launch output**  
→ Normal. Webots was not playing when the driver started.
Press Play ▶ in Webots — the driver connects automatically.
 
**Segmentation fault when stopping the driver (Ctrl+C)**  
→ Known bug in `webots_ros2_driver`, not caused by your code.
The driver shuts down correctly. Safe to ignore.
 
**`rcl_shutdown already called` error on Ctrl+C**  
→ Cosmetic only. ROS2 cleans up before the `finally` block runs. Safe to ignore.
 
**`colcon: command not found`**  
→ Run `source /opt/ros/jazzy/setup.bash` then retry.
 
**Container can't connect to Webots (Mac/Windows)**  
→ Check `.env` has `WEBOTS_HOST=host.docker.internal`.
 
**Mac Apple Silicon — build fails**  
→ Docker Desktop → Settings → General →
enable **"Use Rosetta for x86/amd64 emulation"**.
 
**Changes to task files not taking effect**  
→ Run `cb && src` after every edit.
With `--symlink-install` Python files don't need a full rebuild,
but re-sourcing is still required.
 
**Plot file not appearing on host**  
→ Check the logger actually ran and saved a CSV first:
`ls src/arm_control_pkg/logs/`  
→ If the folder is empty, the data_logger was not running while task2_pid ran.
 
---
