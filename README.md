# MentorPi Workspace — Setup & Run

**ROS 2 Jazzy | Ubuntu 24.04 | RA305 Robotics Programming**

---

## 1. One-Time System Setup

### Serial Port Access

The MentorPi communicates over USB serial via `/dev/rrc`. A udev rule creates this symlink automatically when the robot is plugged in.

Create the rule:

```bash
sudo nano /etc/udev/rules.d/99-rrc.rules
```

Add this line:

```
KERNEL=="ttyACM*", SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", SYMLINK+="rrc", MODE="0666", GROUP="dialout"
```

Reload and apply:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Add your user to the dialout group (requires logout/login to take effect):

```bash
sudo usermod -a -G dialout pi
```

Verify the symlink exists after plugging in the robot:

```bash
ls -la /dev/rrc
```

> **Note:** If you see `Permission denied` on `/dev/rrc` before the group change takes effect, run `sudo chmod 666 /dev/ttyACM0` as a temporary fix.

---

## 2. Create the Workspace and Clone

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

---

## 3. Environment Sourcing

The MentorPi requires a `typerc` environment file to be sourced before launching. This sets the machine type, lidar type, and other hardware variables that the driver nodes depend on. Without it, `odom_publisher` will crash on startup.

Source the following in **every new terminal** you open, in this order:

```bash
source ~/mentorpi_ws/src/.typerc
source ~/mentorpi_ws/install/setup.bash
```

---

## 4. Build the Workspace

```bash
cd ~/mentorpi_ws
```
```bash
colcon build --symlink-install
```
```bash
source ~/mentorpi_ws/install/setup.bash
```

Wait for the build to complete. Fix any errors before proceeding.

> **Note:** `--symlink-install` links install files directly to source files, so changes to Python nodes and launch files take effect immediately without rebuilding.

---

## 5. Run the Rover — Three Terminal Windows

**The robot motor controller must be plugged in to the RPi via USB before launching Terminal 1.**

### Terminal 1 — Launch the Robot

```bash
ros2 launch my_rover launch_robot.launch.py
```

This starts three things: the hardware driver (`ros_robot_controller`), the robot state publisher (TF tree), and the odometry publisher (motor commands + dead-reckoning odom). Leave this running.

---

### Terminal 2 — Open RViz

```bash
rviz2 -d ~/mentorpi_ws/src/my_rover/rviz/view_bot.rviz
```

---

### Terminal 3 — Teleop Keyboard Control

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/controller/cmd_vel
```

Click on Terminal 3 to make sure it has focus, then use the keyboard to drive the robot.

> **Note:** The topic remap is required — `odom_publisher` listens on `/controller/cmd_vel`, not `/cmd_vel`.

---

## Quick Reference

| Terminal | Command |
|----------|---------|
| 1 | `ros2 launch my_rover launch_robot.launch.py` |
| 2 | `rviz2 -d ~/mentorpi_ws/src/my_rover/rviz/view_bot.rviz` |
| 3 | `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/controller/cmd_vel` |

---

## Known Gotchas

- **Controller board must be plugged in** before launching — `/dev/rrc` won't exist otherwise
- **Source `.typerc` first** — `MACHINE_TYPE` must be set or `odom_publisher` crashes
- **Odometry is dead-reckoning only** — `odom_publisher` estimates position from commanded velocity, not encoder feedback, so drift accumulates over time
- **Vendor launch files use hardcoded paths** — don't call `odom_publisher.launch.py` or `robot_description.launch.py` directly, they contain `/home/localuser` paths that will fail

---

*Kansas State University Salina — Robotics and Autonomous Systems*
