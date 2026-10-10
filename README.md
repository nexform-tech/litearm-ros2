# litearm-ros2

The ROS 2 driver stack for the LiteArm arm: a standalone maintenance node that owns the
arm's USB link and the message package it speaks, for ROS 2 Humble users who need the
[litearm-cpp](https://github.com/nexform-tech/litearm-cpp) SDK's administrative command set
without a `controller_manager` in the loop.

The driver opens the arm's USB CDC port itself and serialises every SDK call through one
mutually exclusive callback group, so two commands never interleave on the wire. Motion
control is not its job: trajectories, servo and teleoperation belong to the ros2_control
plugin in [litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control).

**Do not run the driver and the ros2_control stack at the same time.** Both open the same
serial port and the SDK takes an exclusive `flock` on it, so the second process fails to
open the port. Stop one before starting the other.

## Packages

| Package | What it is |
| --- | --- |
| `litearm_driver` | The lifecycle node: 55 services in ten groups, three topics and 14 parameters. Configure creates its services, deactivate destroys them. |
| `litearm_msgs` | The message and service definitions the driver speaks: `LitearmStatus`, `JointParam`, and one service per SDK call. |

Start with [docs/quickstart.md](docs/quickstart.md) for the commands that bring the node
up; the full command set, with the refusals and the reason behind each, is in
[docs/command-set.md](docs/command-set.md), with a Chinese reference in
[docs/command-set.zh-CN.md](docs/command-set.zh-CN.md).

## Highlights

- **No daemon, no shared memory.** The node talks to the firmware directly over USB CDC.
- **Gates run before the frame.** A refused argument puts nothing on the wire, so a bad
  payload mass or speed percent cannot reach the motors.
- **Motion is opt-in.** Every service that can energise the motors is refused until
  `allow_motion` is set, and `allow_dfu` gates the firmware update entry point.
- **Offline tests.** The suite injects the SDK's `FakeTransport` and drives the whole
  lifecycle without an arm attached.

## Requirements

| Item | Requirement |
| --- | --- |
| ROS 2 | Humble |
| Build | `ament_cmake`, C++17 |
| SDK | `litearm-cpp`, either installed or as a sibling source tree |
| Firmware | `Litearm1.5.0` or newer — reported as `Litearm<major.minor.patch>-{7J\|1J}` |
| Connection | USB CDC serial, `VID:PID 1d50:606f` |

On Linux, grant yourself serial port access:

```bash
sudo usermod -aG dialout $USER      # takes effect after you log in again
```

## Build

Both packages need the `litearm-cpp` source tree as a sibling, unless an installed
`litearm` CMake package is already on `CMAKE_PREFIX_PATH`:

```text
~/litearm_ws/src/
├── litearm-cpp/              # the C++ SDK
└── litearm-ros2/             # this repository
```

```bash
source /opt/ros/humble/setup.bash
cd ~/litearm_ws
colcon build
source install/setup.bash
```

To build against a `litearm-cpp` checkout somewhere else, point CMake at it:

```bash
colcon build --cmake-args -DLITEARM_CPP_DIR=/path/to/litearm-cpp
```

## Run

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

The launch file puts the node in the `litearm` namespace, so its relative service names
become `/litearm/enable`, `/litearm/clear_faults` and so on. It configures and activates the
node unless you pass `autostart:=false`.

Every service that can move the arm is refused until the gate is open. The gate is read on
every call, so no restart is needed:

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
```

Support the arm before enabling anything that moves it, and keep the emergency stop within
reach.

## Tests

The suite is offline — it injects the SDK's `FakeTransport` and drives the whole lifecycle
and every service handler without an arm attached:

```bash
cd ~/litearm_ws
colcon test --packages-select litearm_msgs litearm_driver --event-handlers console_direct+
colcon test-result --verbose
```

## Related repositories

| Repository | What it holds |
| --- | --- |
| [litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control) | The ros2_control `SystemInterface` plugin, the control path for `joint_trajectory_controller` and MoveIt 2. |
| [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) | The C++ SDK both repositories build against. |

## License

Apache-2.0. See [LICENSE](LICENSE).
