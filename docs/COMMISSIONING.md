# K9 Drive Commissioning

The supplied launch file defaults to `dry_run:=true` and `start_inhibited:=true` deliberately.
Do not skip stages simply because the source-of-truth Python controller has driven K9 before;
this package changes the software architecture and unit boundary.

## Stage 0 — install and build

Copy `k9_drive_pkg` into `~/k9_ws/src`, then on the Pi:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/k9_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select k9_drive_pkg --symlink-install
source install/setup.bash
```

If Jazzy on the Pi is source-built rather than `/opt/ros/jazzy`, source K9's normal Jazzy setup
instead.

## Stage 1 — pure dry-run

```bash
ros2 launch k9_drive_pkg drive.launch.py
```

Confirm controllers:

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 topic echo /k9/drive/enabled
ros2 topic echo /diagnostics
```

The drive should be active but inhibited. Release only the software inhibit:

```bash
ros2 service call /k9/drive/set_inhibit std_srvs/srv/SetBool "{data: false}"
```

Send an explicit zero first to satisfy the re-arm guard:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

Then send a deliberately small command:

```bash
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.10}, angular: {z: 0.0}}}"
```

Watch `/diff_drive_controller/odom` and `/joint_states`. Ctrl-C the publisher. Verify the
controller timeout returns the simulated wheels to zero.

## Stage 2 — real RoboClaw, wheels clear of the floor

Physically raise K9 so both drive wheels can rotate freely. Keep clear of the wheels. Confirm
`/dev/roboclaw` resolves to the intended controller and no old Python/ROS driver has the port open.

Launch explicitly with real hardware, still inhibited:

```bash
ros2 launch k9_drive_pkg drive.launch.py dry_run:=false start_inhibited:=true
```

Check before releasing inhibit:

```bash
ros2 topic echo /k9/battery/motor
ros2 topic echo /k9/battery/logic
ros2 topic echo /k9/drive/estop
ros2 topic echo /diagnostics
```

Expected wiring/calibration:

- M1 = left;
- M2 = right;
- positive = forward;
- motor battery roughly appropriate for the 24 V system;
- logic voltage appropriate for the RELiON 12.8 V LiFePO4 supply.

Release software inhibit and send the zero command exactly as in Stage 1. Then command only
`+0.05 m/s`. Both wheels must rotate forwards and both joint positions must increase. If either
condition is false, stop and correct the mapping before continuing.

Repeat at `-0.05 m/s`: both wheels should rotate backwards and joint positions should decrease.

## Stage 3 — one-wheel-revolution scale check

With controllers inactive or K9 otherwise safely prevented from driving, rotate each wheel by
exactly one physical revolution and observe the continuous joint position/diagnostics. The scale
must correspond to approximately:

```text
200 RoboClaw counts = 2*pi radians = 0.4358 m of wheel circumference
```

This is the strongest commissioning check that the old 200-count calibration survived the new
ROS unit boundary unchanged.

## Stage 4 — E-stop

Wheels remain off the floor.

1. Arm the drive and command a low forward speed.
2. Operate the latching E-stop key.
3. Motion must stop from the RoboClaw/S3 hardware path, independently of ROS.
4. Startup must report `S3=0x01 (E-Stop)`; any other S3 readback is a commissioning failure.
4. `/k9/drive/estop` and `/k9/drive/estop_latched` should become true.
5. Release/reset the physical key. Raw E-stop should clear but the software latch must remain.
6. Clear it explicitly:

```bash
ros2 service call /k9/drive/clear_estop_latch std_srvs/srv/Trigger "{}"
```

7. A previously non-zero Nav2 command must not resume motion. A new explicit zero command is
   required before re-arming.

## Stage 5 — watchdogs

Perform only with wheels off the floor.

While commanding a low speed, terminate the velocity publisher. The ROS controller timeout
should stop K9. Separately, while moving at low speed, test loss of the controlling process so
that the RoboClaw does not receive refreshed packets; the 0.1 s bounded-distance command and 0.2 s
serial watchdog provide independent limits.

Do not test USB unplugging on the floor.

## Stage 6 — floor distance/odometry

At low speed on a straight, high-grip surface:

1. Mark K9's starting wheel-centre position.
2. Drive a measured distance, initially 0.5–1.0 m.
3. Compare physical travel with `/diff_drive_controller/odom`.
4. Repeat several times forward and backward.

The theoretical scale is already fixed by the known-working controller. Any repeatable physical
bias at this stage should be investigated for wheel effective radius/load/slip before changing the
calibration.

## Stage 7 — turning

Test increasingly tighter radii. `/diagnostics` reports `original_turn_modifier`; it should tend
from 1.0 on straight motion towards 0.1 on an in-place turn. Confirm the hardware QPPS remains
within the radius-dependent ceiling and the turn direction is correct.

## Stage 8 — Nav2 integration

Merge `config/nav2_jazzy_integration.yaml.example` into K9's Nav2 parameters. Jazzy must be
configured to use `TwistStamped` throughout the active velocity chain. The final safety/post-
processing node should publish/remap to `/diff_drive_controller/cmd_vel`.

If using Nav2 Collision Monitor, keep it as the last Nav2 post-processing link before the drive
controller.

## Stage 9 — robot_localization

Once wheel odometry is characterised and IMU fusion is introduced:

- set `enable_odom_tf: false` on `diff_drive_controller`;
- feed wheel odometry into `robot_localization`;
- let the EKF publish the sole `odom -> base_link` transform.
