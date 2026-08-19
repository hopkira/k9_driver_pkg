# K9 RoboClaw Drive Design

## Architecture

```text
Nav2 / teleoperation / K9 behaviour
            |
            | geometry_msgs/TwistStamped
            v
    diff_drive_controller
            |
            | left/right wheel velocity [rad/s]
            v
     K9RoboClawHardware
            |
            | RoboClaw packet serial, exclusive owner
            v
       RoboClaw 2x15A
         M1       M2
        left     right
```

The hardware plugin owns only hardware-specific behaviour. Generic differential-drive
kinematics, odometry, `odom`, optional `odom -> base_link` TF, task-space limits, and the
ROS command timeout belong to `diff_drive_controller`.

## Source-of-truth calibration

The working 2021 K9 Python controller is authoritative:

| Parameter | Value |
|---|---:|
| M1 | left wheel |
| M2 | right wheel |
| positive command | forwards |
| RoboClaw counts/revolution | 200 |
| metres/count | 0.002179 m |
| effective circumference | 0.4358 m |
| effective radius | 0.0693597242 m |
| wheel separation | 0.2022 m |
| operational top speed | 642 QPPS = 1.398918 m/s |
| acceleration | 128 QPPS/s |
| deceleration | 256 QPPS/s |
| emergency deceleration | 512 QPPS/s |
| M1 PID | P=10.644 I=2.206 D=0 QPPS=1987 |
| M2 PID | P=9.768 I=2.294 D=0 QPPS=1837 |

The PID QPPS values are controller tuning/calibration values, not K9's allowed travel speed.
The hardware plugin independently enforces 642 QPPS as the operational ceiling.

### Distance fidelity

`diff_drive_controller` converts body velocity into wheel angular velocity using the effective
radius above. The hardware converts wheel angular velocity into RoboClaw counts using
200 counts/revolution. Therefore both command and feedback use the same known-working scale:

```text
200 counts = 2*pi wheel radians = 0.4358 m
1 count = 0.002179 m
```

No 400-count conversion is used.

## Original K9 turn-speed constraint

The original controller reduced top speed as turning radius tightened:

```text
turn_modifier = 1 - 0.9 / (abs(radius_m) + 1)
```

The hardware plugin preserves that rule by inferring the centreline radius from the two wheel
velocity commands. It scales both wheel commands together, preserving curvature. An
on-the-spot turn therefore has a 0.1 modifier; a 1 m-radius turn has a 0.55 modifier; the
modifier tends to 1 for a straight path.

## Encoder handling

The RoboClaw encoder registers are reset to zero on a genuine hardware configuration, matching
the proven Python implementation. The software does not depend on never wrapping, however.
Every read calculates the delta modulo 2^32 and adds it to a signed 64-bit accumulator:

```text
raw uint32 RoboClaw count
        |
        v
(current - previous) modulo 2^32 -> int32 signed delta
        |
        v
signed int64 accumulated counts
        |
        v
continuous wheel position [rad]
```

Thus startup has a clean origin and inevitable encoder rollover is harmless.

## Safety layers

The protections are deliberately independent:

1. `diff_drive_controller.cmd_vel_timeout = 0.25 s` stops stale ROS commands.
2. Every RoboClaw command is distance-bounded to at most 0.1 seconds of requested travel.
3. RoboClaw's serial watchdog is configured for 0.2 seconds.
4. S3 is configured as RoboClaw v4.1.34 E-stop mode `0x01` and wired to K9's mechanically-latching key.

Additional safeguards:

- hard 642-QPPS operational ceiling in the hardware plugin;
- original radius-dependent turn-speed ceiling;
- normal/deceleration/emergency ratios 128/256/512;
- stop on deactivate, cleanup, shutdown, error, destruction, invalid command, and serial failure;
- non-finite commands are rejected;
- E-stop state is latched in software after the physical key is released;
- a zero wheel command is required after activation/re-enable/E-stop clear before motion re-arms;
- real and dry-run hardware start software-inhibited by default;
- no ROS timer callback accesses the serial port; telemetry publishing uses cached values;
- one process owns `/dev/roboclaw`.

### Why S3 uses mode 0x01

K9's replacement emergency switch is mechanically latching. Using RoboClaw's firmware-latching
mode as well would require a controller reset/power cycle after every event. Mode 2 keeps the
RoboClaw v4.1.34 defines `0x01` as the non-latching E-stop (`0x81` is firmware-latching). The mechanically-latching key keeps the physical hardware stop active for as long as the key asserts S3, while the driver adds an
explicit software latch and re-arm sequence after release.

## Batteries and diagnostics

The plugin publishes:

- `/k9/battery/motor` — RoboClaw main supply, K9 24 V motor battery;
- `/k9/battery/logic` — RELiON 12.8 V LiFePO4 logic supply;
- `/diagnostics` — controller state, status bits, currents, temperature, encoder accumulators,
  measured/commanded QPPS, selected acceleration, and applied turn modifier;
- `/k9/drive/estop` — raw S3/RoboClaw E-stop state;
- `/k9/drive/estop_latched` — software-latched emergency state;
- `/k9/drive/enabled` — true only when the drive is active and fully armed.

The driver does not invent state-of-charge from voltage. Battery percentage/capacity/current
fields are NaN unless genuinely measured by the relevant source.

## Odometry ownership

For initial bench/drive commissioning `diff_drive_controller` publishes `odom -> base_link`.
When K9 later fuses wheel odometry and IMU using `robot_localization`, set
`enable_odom_tf: false` in `drive_controller.yaml` and let the EKF own `odom -> base_link`.
Do not allow two publishers to own the same TF.

### E-stop lifecycle semantics

The physical RoboClaw E-stop is a motion interlock, not a reason to take the ros2_control hardware component
offline. `on_activate()` therefore succeeds with S3 asserted while `write()` forces zero whenever
`raw_estop` or `estop_latched` is true. Communication faults and non-E-stop RoboClaw hardware faults still
cause lifecycle activation to fail. This keeps diagnostics, encoder feedback and battery telemetry available
during an emergency-stop condition.
