# Validation notes

The package retains K9's proven drivetrain calibration and safety design while moving the generic differential-drive layer to `ros2_control`.

## Fixed K9 drivetrain calibration

- M1 = left wheel; M2 = right wheel.
- Positive motor command = forward.
- 200 RoboClaw encoder counts per physical wheel revolution.
- 0.002179 m per encoder count.
- Effective wheel circumference 0.4358 m and radius 0.06935972 m.
- Wheel separation 0.2022 m.
- Operational wheel-speed ceiling 642 qpps.
- Acceleration/deceleration/emergency-deceleration 128/256/512 counts/s^2.

## Encoder handling

The RoboClaw encoder registers are reset on real hardware configuration, matching the proven 2021 K9 controller. Runtime wrap is independently handled with modular 32-bit deltas accumulated into continuous 64-bit software counts before conversion to ROS joint radians.

## RoboClaw 2x15A v4.1.34 protocol details verified on K9

K9's controller identifies as `USB Roboclaw 2x15a v4.1.34`.

- Commands 74/75 use the three-byte S3/S4/S5 mode form.
- S3 mode `0x01` is used for the non-latching RoboClaw E-stop on firmware v4.1.34.
- Startup writes `S3/S4/S5 = 0x01/0x00/0x00`, reads the modes back, and refuses configuration unless S3 reads back exactly `0x01`.
- Command 90 (`GETERROR`) returns FOUR status data bytes followed by CRC on K9's controller. This matches K9's later proven RoboClaw C++ driver (`CmdReadStatus`, which uses `getULongCont2`).
- E-stop is bit 0 of that 32-bit status word: `0x00000001`.

The earlier 16-bit command-90 assumption was disproved by the actual controller response: reading only two data bytes caused the upper two status bytes to be interpreted as the CRC.

## Local validation performed

The RoboClaw transport source compiles independently with C++17 and strict warnings. YAML/Xacro/launch syntax has been checked. A final ROS 2 Jazzy `colcon build` must be performed on the target ROS host.
