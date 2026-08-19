# Bundle validation performed during generation

The generation environment did not contain a ROS 2 Jazzy installation, so a full `colcon build`
could not be performed here. The following checks were completed successfully before packaging:

- XML parsing: `package.xml`, plugin XML and both Xacro/XML files;
- YAML parsing: controller and Nav2 integration parameter files;
- Python syntax compilation: `launch/drive.launch.py`;
- strict standalone C++ compilation (`-std=c++17 -Wall -Wextra -Wpedantic -Werror`) of
  `roboclaw_transport.cpp`;
- standalone drive-math smoke tests for the 200-count calibration, forward and reverse uint32
  encoder rollover, 642-QPPS operating limit, bounded-distance calculation, and 128/256/512
  acceleration selection;
- manual API comparison with current ROS 2 Jazzy `ros2_control` and `diff_drive_controller`
  documentation;
- RoboClaw status-bit review against the 32-bit command-90 status layout used by the later
  `hopkira/roboclaw_driver` implementation.

The first Pi-side step should therefore still be:

```bash
cd ~/k9_ws
source /opt/ros/jazzy/setup.bash   # or K9's normal Jazzy setup on the Pi
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select k9_drive_pkg --symlink-install
```

Then follow `COMMISSIONING.md` rather than enabling real movement immediately.


## RoboClaw firmware protocol correction

K9's controller identifies as `USB Roboclaw 2x15a v4.1.34`. The driver therefore uses the
RoboClaw Revision 5.6 / firmware 4.1.x packet protocol: command 74 has three pin-mode bytes,
mode `2` is non-latching E-Stop, command 75 returns three pin-mode bytes, and command 90
returns a 16-bit status word with E-Stop at bit mask `0x0004`. Startup reads the pin modes back
and refuses configuration if S3 is not actually mode 2.
