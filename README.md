# k9_drive_pkg

ROS 2 Jazzy `ros2_control` hardware support for K9's RoboClaw 2x15A differential drive.

## What this package does

`K9RoboClawHardware` is a `hardware_interface::SystemInterface` loaded by
`controller_manager`. It presents two standard wheel joints to
`diff_drive_controller` and exclusively owns the RoboClaw USB serial connection.

It deliberately does **not** reimplement generic differential-drive odometry or subscribe to
`/cmd_vel` itself. `diff_drive_controller` handles those standard ROS functions; this package
handles K9/RoboClaw-specific units, limits, diagnostics and safety.

## Locked K9 drivetrain values

```text
RoboClaw                 2x15A
Serial                   /dev/roboclaw @ 115200, address 0x80
M1                       left wheel
M2                       right wheel
Positive                 forwards
Encoder                   200 RoboClaw counts / wheel revolution
Distance scale            0.002179 m / count
Effective wheel radius    0.0693597242 m
Wheel separation          0.2022 m
K9 operational top speed  642 QPPS ~= 1.398918 m/s
Accel / decel / emergency 128 / 256 / 512 QPPS/s
M1 velocity PID           P 10.644, I 2.206, D 0, QPPS 1987
M2 velocity PID           P 9.768, I 2.294, D 0, QPPS 1837
Main voltage limits       24.0 V / 29.2 V
Logic supply              RELiON 12.8 V LiFePO4
S3                        E-stop mode 2; mechanically-latching physical key
```

The 200-count and 0.002179 m/count values are the working 2021 controller's calibration and are
intentionally authoritative over the later ROS2 driver's 400-count setting.

## Safety

Four independent stop layers are retained/added:

1. `diff_drive_controller` stale-command timeout;
2. RoboClaw speed commands limited to ~0.1 s maximum uncommanded travel;
3. RoboClaw serial watchdog set to 0.2 s;
4. physical S3 emergency stop.

The hardware plugin also starts inhibited, hard-limits wheel QPPS, preserves the original K9
radius-dependent turn-speed reduction, uses the 128/256/512 acceleration policy, latches an
E-stop in software after the key is released, requires a zero-command re-arm, and stops on all
lifecycle/error/destruction paths.

## Build

```bash
cd ~/k9_ws/src
# copy k9_drive_pkg here
cd ~/k9_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select k9_drive_pkg --symlink-install
source install/setup.bash
```

## Start safely

The launch file is dry-run by default:

```bash
ros2 launch k9_drive_pkg drive.launch.py
```

Real hardware must be explicitly requested and still starts software-inhibited:

```bash
ros2 launch k9_drive_pkg drive.launch.py dry_run:=false start_inhibited:=true
```

Then, only when K9 is physically safe to move:

```bash
ros2 service call /k9/drive/set_inhibit std_srvs/srv/SetBool "{data: false}"
```

An explicit zero wheel/body command is required before non-zero motion will be accepted.

## Main ROS interfaces

```text
/diff_drive_controller/cmd_vel     geometry_msgs/TwistStamped  input
/diff_drive_controller/odom        nav_msgs/Odometry           encoder odometry
/diff_drive_controller/cmd_vel_out geometry_msgs/TwistStamped  limited command
/joint_states                      sensor_msgs/JointState

/k9/battery/motor                  sensor_msgs/BatteryState
/k9/battery/logic                  sensor_msgs/BatteryState
/k9/drive/estop                    std_msgs/Bool
/k9/drive/estop_latched            std_msgs/Bool
/k9/drive/enabled                  std_msgs/Bool
/diagnostics                       diagnostic_msgs/DiagnosticArray

/k9/drive/set_inhibit              std_srvs/SetBool
/k9/drive/clear_estop_latch        std_srvs/Trigger
```

## Nav2 Jazzy

`diff_drive_controller` uses `TwistStamped`. Nav2 Jazzy components default to unstamped `Twist`
in several places, so set `enable_stamped_cmd_vel: true` on every active Nav2 cmd_vel component
and remap the final command output to `/diff_drive_controller/cmd_vel`. See
`config/nav2_jazzy_integration.yaml.example`.

If Collision Monitor is used, it should remain the final Nav2 velocity safety/post-processing
component before `diff_drive_controller`.

## Integration into K9's full URDF

`urdf/k9_drive_bench.urdf.xacro` is intentionally a minimal commissioning robot. K9's real robot
description should include `urdf/k9_ros2_control.xacro` and invoke:

```xml
<xacro:k9_roboclaw_ros2_control
  name="K9RoboClawSystem"
  dry_run="false"
  start_inhibited="true"
  device="/dev/roboclaw"/>
```

The physical wheel/link geometry remains owned by the main K9 robot description; the
`ros2_control` macro supplies the hardware interfaces and known drivetrain parameters.

## Commissioning

Follow `docs/COMMISSIONING.md` in order. It begins with pure dry-run, then wheels-off-ground
hardware tests, encoder scale/direction, E-stop and watchdog testing, physical distance checks,
turning, and finally Nav2 integration.

## Tests

`test/test_drive_math.cpp` covers the most safety/accuracy-sensitive pure maths, including:

- 200-count distance calibration;
- 32-bit forward and reverse rollover;
- original top speed;
- original radius-dependent turn limiter;
- 128/256/512 acceleration selection;
- bounded command travel.

## Provenance

K9's physical calibration and movement constraints come from Richard Hopkins' working 2021
Python controller (Unlicense). The packet-serial design and later safety concepts were informed
by `hopkira/roboclaw_driver`, whose RoboClaw implementation carries Apache-2.0 provenance from
WimbleRobotics/Sigyn. This generated package is therefore distributed as Apache-2.0.
