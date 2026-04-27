# MentorPi Workspace — Setup & Run

**ROS 2 Jazzy | Ubuntu 24.04 | RA305 Robotics Programming**

-----

## 1. Create the Workspace and Clone

```bash
mkdir -p ~/mentorpi_ws/src
```
```bash
cd ~/mentorpi_ws/src
```
Now clone the repository. 
```bash
git clone https://github.com/KSU-Chad/rover_ws.git .
```
> **Note:** The trailing `.` clones the repo contents directly into `src/` rather than creating a nested subdirectory.

-----

## 2. Build the Workspace

```bash
cd ~/mentorpi_ws
```
```bash
colcon build
```

Wait for the build to complete. Fix any errors before proceeding.

-----

## 3. Source the Install

Do this in **every new terminal** you open:

```bash
source install/setup.bash
```

> Pro tip: Add that line to your `~/.bashrc` so you don’t have to think about it.

-----

## 4. Launch — Three Terminal Windows

### Terminal 1 — Launch the Simulation

```bash
source install/setup.bash
ros2 launch my_rover launch_sim.launch.py use_sim_time:=true
```

Leave this running.

-----

### Terminal 2 — Open RViz

Replace `<user>` with your actual username (e.g., `pi`, `student`, etc.):

```bash
source install/setup.bash
rviz2 -d /home/<user>/mentorpi_ws/src/my_rover/rviz/view_bot.rviz
```

-----

### Terminal 3 — Teleop Keyboard Control

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard —ros-args -p stamped:=true
```

Click on Terminal 3 to make sure it has focus, then use the keyboard to drive the robot.

-----

## Quick Reference

|Terminal|Command                                                            |
|--------|-------------------------------------------------------------------|
|1       |`ros2 launch my_rover launch_sim.launch.py use_sim_time:=true`                        |
|2       |`rviz2 -d /home/<user>/mentorpi_ws/src/my_rover/rviz/view_bot.rviz`|
|3       |`ros2 run teleop_twist_keyboard teleop_twist_keyboard —ros-args -p stamped:=true`             |

-----

*Kansas State University Salina — Robotics and Autonomous Systems*