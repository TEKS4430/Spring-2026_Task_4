# Spring-2026 Task 4 — Arm Control

## Part A


### Overview

This assignment focuses on how autonomous cyber-physical systems execute actions in the physical world through control and actuation. Students implement a control pipeline for a robotic arm in a Webots and ROS 2 environment, exploring how discrete commands are translated into continuous motion. The task covers forward kinematics, PID control for trajectory tracking, and a state-machine-based pick-and-place behavior, highlighting the interaction between modeling, control, and physical constraints. Through this, students gain practical understanding of how perception and decision-making are realized as stable, controlled physical actions.

Setup is familiar from previous assignments:
> **Webots R2025a** runs on your host OS.  
> **ROS2 Jazzy** runs inside Docker.  
> Your code lives in `src/` — edit on host, build and run in container.

---

## Quick start

### 1. First-time setup

```bash
cp env.example .env
# Linux:        uncomment NETWORK_MODE=host and WEBOTS_HOST=localhost
# Mac/Windows:  uncomment NETWORK_MODE=bridge and WEBOTS_HOST=host.docker.internal

docker compose build   # ~5 min first time
```

### 2. Open Webots

1. Open Webots R2025a → **File → Open World** → `src/worlds/arm_assignment.wbt`
2. Press **Play ▶** before launching the driver

> ⚠️ If you see `"retrying for another 50 seconds..."` — just press Play in Webots.  
> The driver reconnects automatically.

### 3. Start the container and build

```bash
docker compose run --rm ros2 bash
cb && src
```

| Alias | What it does |
|-------|-------------|
| `cb`  | `colcon build --symlink-install` |
| `src` | `source install/setup.bash` |
| `rt`  | `ros2 topic list` |

### 4. Start the driver (always Terminal 1)

```bash
ros2 launch arm_control_pkg arm.launch.py
```

Confirm the driver loaded — you should see:
```
[YouBotArmDriver] Driver ready — all topics live
```

Then check topics in a second terminal:
```bash
rt | grep youBot
```

You should see `/youBot/arm_joint_1/sensor` through `arm_joint_5/sensor` and `/youBot/end_effector_position`. If you see nothing, the driver is not running.

---

## Task 1 — Forward Kinematics

**Learning goal:** Understand how joint angles map to end-effector position.

```bash
# Terminal 1 — driver (keep open for all tasks)
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — FK explorer
ros2 run arm_control_pkg task1_fk
```

Edit `joint_index` and `target_angle` at the bottom of `task1_fk_explorer.py`, then rebuild and run again for each experiment. Each run first sends **all joints to 0.0 rad**, waits 3 s, prints `Start EE`, and only then moves the chosen joint — so every row starts from the same pose.

```bash
# After editing on your host:
cb && src
ros2 run arm_control_pkg task1_fk
```

### What to record

Fill in this table in your report (run the node once per row):

| Joint | Start (rad) | End (rad) | EE start (x, y, z) | EE end (x, y, z) |
|-------|-------------|-----------|---------------------|------------------|
| 1 | 0.0 | 1.0 | | |
| 1 | 0.0 | 2.0 | | |
| 2 | 0.0 | 0.5 | | |
| 2 | 0.0 | 1.0 | | |
| 3 | 0.0 | -1.0 | | |
| 4 | 0.0 | 1.0 | | |

### Hints

- **Wait for values to stop changing** before recording — the arm takes 1–2 seconds to settle after a command.
- **What the axes mean:** `x` is forward (away from the robot body), `y` is sideways (left), `z` is up. All values are relative to the arm's base (where it is mounted on the robot).
- **Zero pose:** with all joints at 0.0 rad the arm points **straight up**.
- **Which joints do what:** Joint 1 rotates the base (changes x and y, z stays the same). Joints 2, 3 and 4 tilt the arm in its vertical plane: **negative angles lean it forward**, positive angles lean it backward over the robot. Joint 5 rolls the wrist and does not move the EE point.
- **Task 1b — answer in your report:** *Why can't the arm reach (0.0, 0.0, 0.05) regardless of joint angles?* Think about what the arm is physically attached to, and what that means about positions directly below or behind it.

---

## Task 2 — PID Controller (Sine Wave Tracking)

**Learning goal:** Implement and tune a PID controller. Observe how each term affects tracking.

Joint 2 follows a continuously moving sine wave target (centre 0.5 rad, amplitude 0.3 rad, period 20 s). Your controller outputs a **joint velocity** (rad/s) on `/youBot/arm_joint_2/velocity`. The joint only moves when your controller tells it to — with `correction = 0` the arm stays still while the target moves away.

Each run first moves joint 2 to 0.5 rad (3 s), then starts tracking, so all runs start from the same point.

### Workflow — one run at a time

```bash
# Before your first run (inside the container): clear old logs
rm -f /ros2_ws/src/arm_control_pkg/logs/*.csv

# Terminal 1 — driver (keep open across all runs)
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — your PID controller
ros2 run arm_control_pkg task2_pid

# Terminal 3 — data logger (saves plot automatically on Ctrl+C)
ros2 run arm_control_pkg data_logger --ros-args -p label:=P_only
```

Run for ~60 seconds (3 full cycles), then Ctrl+C both Terminal 2 and Terminal 3.

Repeat with labels `PD` and `PID`. When you stop the logger it automatically saves the comparison plot to:
```
src/arm_control_pkg/logs/task2_plot.png
```
Open that file directly on your host machine — no display needed in Docker.

### What to implement

Open `task2_pid_joint.py` and find the TODO section in `_control_loop`. Implement in order:

**Task 2a — P term only** (`Ki=0, Kd=0`):
```python
correction = Kp * error
```
The arm follows the sine wave but lags behind it. Try several `Kp` values: the lag shrinks as `Kp` grows, until the arm starts to oscillate.

**Task 2b — Add D term** (keep your Kp):
```python
derivative = (error - self.prev_error) / self.dt
correction = Kp * error + Kd * derivative
```
At moderate `Kp` the D term changes little. Its job is damping: push `Kp` close to where P-only oscillated and see the D term calm it down.

**Task 2c — Add I term** (keep your Kp and Kd):
```python
self.integral += error * self.dt
correction = Kp * error + Kd * derivative + Ki * self.integral
```
The remaining lag should mostly disappear.

### Hints

- **With `correction = 0.0`** (before you implement anything), the arm does not move at all and the error grows to the full amplitude of the wave. This is expected.
- **Starting gain values:** `Kp = 2.0` is a good start. Larger `Kp` reduces lag; if the arm starts to shake or oscillate, back off.
- **Keep Kd small** (0.05–0.3). The D term amplifies sensor noise — too large and the arm will vibrate rather than dampen.
- **Ki** in the range 0.5–2.0 works well. Too large causes slow, growing oscillations.
- **Each run must be independent** — stop Terminal 2 (controller) and Terminal 3 (logger) with Ctrl+C before starting the next run. Use a different `label` each time so the plot shows all three curves. If you redo a run, delete its old CSV first.

---

## Task 3 — Pick and Place State Machine

**Learning goal:** Implement a discrete state machine and observe the gap between discrete decisions and continuous arm dynamics.

The arm must complete this sequence:
```
HOME → APPROACH → GRASP → CLOSE_GRIP → LIFT
```

```bash
# Terminal 1 — driver
ros2 launch arm_control_pkg arm.launch.py

# Terminal 2 — state machine
ros2 run arm_control_pkg task3_pick
```

### What to implement

Open `task3_pick_place.py` and implement the two methods:

**`_send_joint_commands(state)`** — publish target angles to all 5 joints:
```python
for i in range(5):
    msg = Float64()
    msg.data = STATE_ANGLES[state][i]
    self.joint_pubs[i].publish(msg)
```

**`_check_transition(state)`** — return `True` only when safe to advance:
```python
# Condition 1: all joints close enough to target
all_close = all(
    abs(self.current_angles[i] - STATE_ANGLES[state][i]) < POSITION_THRESHOLD
    for i in range(5)
)
# Condition 2: arm has held that position for DWELL_TIME seconds
if all_close:
    if self.dwell_start is None:
        self.dwell_start = time.time()
    return time.time() - self.dwell_start >= DWELL_TIME
else:
    self.dwell_start = None
    return False
```

### Run it twice

Edit `DWELL_TIME` at the top of the file:

| Run | Setting | Expected behaviour |
|-----|---------|-------------------|
| 1 | `DWELL_TIME = 0.0` | Each state ends as soon as the joints are within the threshold, while the arm is still moving. GRASP and CLOSE_GRIP use the same angles, so CLOSE_GRIP ends instantly and the arm lifts before the gripper closes — the box stays on the table. |
| 2 | `DWELL_TIME = 1.5` | Each state settles before the next begins; the gripper closes and the box is lifted. |

Rebuild between runs: `cb && src`

### Adjusting STATE_ANGLES for your setup

The provided `STATE_ANGLES` reach the box in the provided world, with the gripper angled about 30° below horizontal (the box is too high and too far for a straight-down grasp). Small differences in the simulation may still make the gripper miss slightly — this is normal and part of the task. To adjust:

1. **Use Task 1's FK explorer** to find angles that position the EE near your box:
   - Run `task1_fk` with different joint angles
   - Look at the EE `x` and `z` values — aim to match the box position visually
   - Joints 2, 3 and 4 all tilt the arm forward when **negative** (0 = straight up)
   - Joint 2 (shoulder) moves the whole arm; joint 3 (elbow) bends the upper part
   - Joint 4 (wrist) sets the gripper angle — more negative points it further down

2. **Update `STATE_ANGLES`** in `task3_pick_place.py` with your found angles:
   - `APPROACH` should position the EE a few centimetres **above** the box
   - `GRASP` should position the EE at the box level
   - `LIFT` should raise the arm back up

3. **You can also move the box** in the Webots Scene Tree to a position that's easier to reach — click the TARGET_BOX node, change its `translation` field.

> The arm's maximum reach is approximately **0.46 m** from its shoulder joint, which sits ~0.19 m in front of the robot centre and ~0.25 m above the floor. The box must be within this range.

---



## Troubleshooting

**No `/youBot/...` topics after launch**  
→ Driver not loaded. Check Webots is open and simulation is playing ▶.

**`angle=0.000` never changes in task2**  
→ Driver must be running in a separate terminal. `task2_pid` alone does nothing — it needs the driver running in Terminal 1 simultaneously.

**Arm holds still in Task 2 after the first 3 s**  
→ `correction` is still `0.0`. Implement the P term first.

**Arm doesn't move in Task 3 / stuck on HOME forever**  
→ `_send_joint_commands` is not implemented yet (it's `pass`). Implement it first — without it, no commands are published and the arm never moves.

**Segmentation fault when stopping the driver (Ctrl+C)**  
→ Known bug in `webots_ros2_driver`. Safe to ignore — driver shuts down correctly.

**`rcl_shutdown already called` on Ctrl+C**  
→ Cosmetic only. Safe to ignore.

**Changes not taking effect**  
→ Run `cb && src` after every edit. Always rebuild before testing.

**Container can't connect to Webots (Mac/Windows)**  
→ Check `.env` has `WEBOTS_HOST=host.docker.internal`.

**Mac Apple Silicon — build fails**  
→ Docker Desktop → Settings → General → enable "Use Rosetta for x86/amd64 emulation".



## Part B: Use of AI tools and self-reflection

In part B, you will reflect on the new skills you have learned and analyze your own work and learning process. You will consider what technical skills you learned while completing part A, what tools you used, why you chose to use them, and what benefits or challenges were associated with their use.

Include insight on:
- What kind of challenges did you encounter while doing the assignment.
- What did you learn while doing the assignment.
- Which tools you used and for what purpose.
- Any prompts that were particularly helpful.
- Cases where the AI was wrong or misleading, and how you detected that.

[See more detailed instructions from Task 2](https://github.com/TEKS4430/Spring-2026_Task_2/tree/main#part-b-use-of-ai-tools-and-self-reflection)

---

## Returning instructions

Create a video covering Parts A and B:
- Demonstrate the environment running (Webots + ROS 2 launch).
- Show the FK explorer (Task 1), the PID tracking plot (Task 2) and the pick-and-place sequence with both DWELL_TIME settings (Task 3).
- Explain how you used AI in the assignment.

[See more detailed instructions from Task 2](https://github.com/TEKS4430/Spring-2026_Task_2/tree/main#returning-instructions)

