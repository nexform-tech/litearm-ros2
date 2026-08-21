# litearm-ros2

A [ROS2](https://docs.ros.org/en/humble) bridge workspace for the **LiteArm**
robotic arm. It wraps [litearm-python](https://pypi.org/project/litearm-python)
so the arm appears as standard ROS2 topics, transforms and services: joint
states, TCP pose + TF, joint commands, and `movej`/`movel`/`fk`/`ik` services.

The workspace contains two standard packages:

| Package | Type | Role |
|---|---|---|
| `litearm_interfaces` | ament_cmake | Custom services: `Movej`, `Movel`, `Fk`, `Ik`, `GetState` |
| `litearm_ros2` | ament_python | Bridge node + ROS-agnostic core (`bridge.py`, `pose_utils.py`) |

```text
ROS2 topics / services ──→ litearm_node ──→ LiteArmBridge ──→ litearm.Arm ──→ litearm-server
```

> 📖 Full developer guide & API reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · 中文文档：[README.zh-CN.md](README.zh-CN.md)

---

## Table of contents

- [Overview & architecture](#overview--architecture)
- [Requirements](#requirements)
- [Install & build](#install--build)
- [Launch](#launch)
- [Node interface](#node-interface)
- [Usage examples](#usage-examples)
- [TF & visualization](#tf--visualization)
- [Troubleshooting](#troubleshooting)
- [Workspace layout](#workspace-layout)
- [Documentation & license](#documentation--license)

---

## Overview & architecture

The workspace is a **colcon** workspace with two packages. The interface package
`litearm_interfaces` holds the custom services (they must be an ament_cmake
package because ROS2 generates interfaces at the CMake phase). The bridge
package `litearm_ros2` provides a single node, `litearm_node`, that connects to
a running **litearm-server** over its Zenoh endpoint:

| Direction | ROS2 side | Arm side |
|---|---|---|
| Out (state) | `/joint_states`, `/litearm/tcp_pose`, `/tf` | `Arm.get_state()`, `Arm.get_tcp_pose()` |
| In (command) | `/litearm/cmd_joint`, `/litearm/stop` | `Arm.movej()`, `Arm.request_stop()` |
| In (service) | `/litearm/movej`, `/litearm/movel`, `/litearm/fk`, `/litearm/ik`, `/litearm/get_state`, `/litearm/request_stop`, `/litearm/clear_stop`, `/litearm/enable`, `/litearm/disable` | `Arm.*` RPCs |

The bridge logic lives in `litearm_ros2/src/litearm_ros2/bridge.py` +
`pose_utils.py`, which contain **no `rclpy` imports** — they work on plain
dicts/tuples, are shared with the ROS1 bridge, and can be unit-tested on any
machine without ROS.

## Requirements

| Item | Requirement |
|---|---|
| ROS2 | **Humble** (tested) with `colcon` |
| Python | 3.10+ with `rclpy` |
| Base SDK | `litearm-python` (pip; **not** rosdep-resolvable) |
| Runtime | A reachable **litearm-server** (Zenoh endpoint, e.g. `tcp/192.168.31.237:7447`) |

> `litearm-python` talks to litearm-server over Zenoh; the server owns the
> actual hardware or simulation. Make sure it is running before launch.

## Install & build

```bash
source /opt/ros/humble/setup.bash
cd <this repo>                        # the workspace root (both packages)

# install the base SDK into the python that runs the node:
pip install litearm-python            # or: pip install -e /path/to/litearm-python

colcon build
source install/setup.bash
```

- **Interpreter matters.** The node imports `litearm`, so `litearm-python` must
  be importable from the same python that provides `rclpy`. On a stock Humble
  install that is the system python: `sudo python3 -m pip install litearm-python`
  and upgrade `protobuf>=7.35.1` in the same interpreter if the node import
  fails with a protobuf error.
- **Build-time `empy`.** Colcon needs `empy` (`python3-empy` on Debian/Ubuntu).
  If the build fails with `No module named 'em'`, the `python3` on `PATH` is
  missing it (e.g. a conda base env). Point `PATH` at `/usr/bin` or install
  `python3-empy` into that env.

Run the mock unit tests on any machine (no ROS required):

```bash
cd litearm_ros2 && PYTHONPATH=src python3 -m pytest tests/ -q
```

## Launch

```bash
ros2 launch litearm_ros2 litearm.launch.py \
  endpoint:=tcp/192.168.31.237:7447   # address of the litearm-server
```

Or run the node directly:

```bash
ros2 run litearm_ros2 litearm_node --ros-args \
  -p endpoint:=tcp/192.168.31.237:7447 -p arm_id:=armA
```

The launch file accepts the following arguments (defaults come from
`config/litearm.yaml`, which is loaded as node parameters):

| Arg | Default | Meaning |
|---|---|---|
| `endpoint` | `tcp/192.168.31.237:7447` | Zenoh endpoint of the litearm-server |
| `arm_id` | `armA` | Arm id registered on the server |
| `loop_hz` | `50.0` | State publish rate in Hz |
| `cmd_speed` | `0.5` | Default joint move speed (0..1) |
| `joint_names` | `joint0 … joint6` | Joint names used in `/joint_states` |
| `base_frame` | `base_link` | Robot base frame name |
| `tcp_frame` | `tool0` | TCP (tool) frame name |

All parameters can also be overridden via `--ros-args -p` on the command line or
in `config/litearm.yaml` (see [Parameters](#parameters)).

## Node interface

### Publishers

| Topic | Type | Contents |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `q` / `dq` / `tau` published at `loop_hz` |
| `/litearm/tcp_pose` | `geometry_msgs/msg/PoseStamped` | TCP pose in `base_frame` |
| `/tf` | `tf2_ros` transform broadcaster | `base_frame → tcp_frame` transform |

### Subscribers

| Topic | Type | Action |
|---|---|---|
| `/litearm/cmd_joint` | `sensor_msgs/msg/JointState` | `movej(position, speed=cmd_speed)` |
| `/litearm/stop` | `std_msgs/msg/Empty` | Emergency stop: `request_stop()` |

### Services

Custom services (`litearm_interfaces/srv/*.srv`). Every response carries
`success` + `message`.

| Service | Request | Response |
|---|---|---|
| `/litearm/movej` | `float64[] q_target, float64 speed, float64 settle_s` | `bool success, string message` |
| `/litearm/movel` | `geometry_msgs/Pose pose, float64 speed, float64 settle_s` | `bool success, string message` |
| `/litearm/fk` | `float64[] q` | `geometry_msgs/Pose pose, bool success, string message` |
| `/litearm/ik` | `geometry_msgs/Pose pose, float64[] q_seed` | `float64[] q, bool success, string message` |
| `/litearm/get_state` | *(empty)* | `float64[] q/dq/tau, string state, bool fault, string message` |
| `/litearm/request_stop` | `std_srvs/srv/Trigger` | `bool success, string message` |
| `/litearm/clear_stop` | `std_srvs/srv/Trigger` | `bool success, string message` |
| `/litearm/enable` | `std_srvs/srv/Trigger` | `bool success, string message` |
| `/litearm/disable` | `std_srvs/srv/Trigger` | `bool success, string message` |

### Parameters

Declared node parameters, defaults below (launch args and `config/litearm.yaml`
both feed them):

| Param | Default | Meaning |
|---|---|---|
| `endpoint` | `tcp/192.168.31.237:7447` | Zenoh endpoint of the litearm-server |
| `arm_id` | `armA` | Arm id registered on the server |
| `loop_hz` | `50.0` | State publish rate in Hz |
| `cmd_speed` | `0.5` | Default joint move speed (0..1) used by `/litearm/cmd_joint` |
| `joint_names` | `joint0 … joint6` | Joint names used in `/joint_states` |
| `base_frame` | `base_link` | Robot base frame name |
| `tcp_frame` | `tool0` | TCP (tool) frame name |

## Usage examples

### Inspect the arm state

```bash
ros2 topic echo /joint_states
ros2 topic echo /litearm/tcp_pose
ros2 run tf2_ros tf2_echo base_link tool0     # live TF between the two frames
```

### Drive a joint-space motion (topic)

```bash
ros2 topic pub -1 /litearm/cmd_joint sensor_msgs/msg/JointState \
  '{name: [joint0, joint1, joint2, joint3, joint4, joint5, joint6], \
    position: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}'
```

`position` is passed to `movej()` with the node's `cmd_speed`.

### Drive a joint-space motion (service)

```bash
ros2 service call /litearm/movej litearm_interfaces/srv/Movej \
  '{q_target: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], speed: 0.3, settle_s: 0.5}'
```

`q_target` holds 7 joint positions, `speed` is a normalized velocity in `0..1`,
and `settle_s` is the dwell time after the motion settles.

### Move to a Cartesian pose

```bash
ros2 service call /litearm/movel litearm_interfaces/srv/Movel \
  '{pose: {position: {x: 0.25, y: 0.0, z: 0.35}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, speed: 0.3, settle_s: 0.5}'
```

The pose is a `geometry_msgs/Pose` in the `base_frame`; orientation is a
quaternion `(x, y, z, w)`.

### Forward / inverse kinematics

```bash
# FK: joints → TCP pose
ros2 service call /litearm/fk litearm_interfaces/srv/Fk \
  '{q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# IK: TCP pose → joints (q_seed is optional, helps pick a solution)
ros2 service call /litearm/ik litearm_interfaces/srv/Ik \
  '{pose: {position: {x: 0.25, y: 0.0, z: 0.35}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
    q_seed: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### Read the full state

```bash
ros2 service call /litearm/get_state litearm_interfaces/srv/GetState
# → q, dq, tau arrays, state string, fault bool, message
```

### Emergency stop and recovery

```bash
# immediate stop (topic):
ros2 topic pub -1 /litearm/stop std_msgs/msg/Empty '{}'
# or as a service:
ros2 service call /litearm/request_stop std_srvs/srv/Trigger '{}'

# clear the stop once it is safe to move again:
ros2 service call /litearm/clear_stop std_srvs/srv/Trigger '{}'
```

### Enable / disable

```bash
ros2 service call /litearm/enable std_srvs/srv/Trigger '{}'
ros2 service call /litearm/disable std_srvs/srv/Trigger '{}'
```

## TF & visualization

The node broadcasts a transform `base_link → tool0` at every state update, so
any TF consumer can track the arm. To visualize:

```bash
ros2 run rviz2 rviz2
# Add → TF, set the Fixed Frame to base_link
```

> This workspace does not ship a URDF or a MoveIt config — it exposes the live
> TF `base_link → tool0` and the `/litearm/tcp_pose` topic, which is enough to
> visualize the arm pose. If you have a URDF for your LiteArm, match its
> base/tool frame names via `base_frame` / `tcp_frame`.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `ModuleNotFoundError: No module named 'litearm'` | `litearm-python` is installed into a different python than `rclpy`. `sudo python3 -m pip install litearm-python`. |
| Import error mentioning `protobuf` | Upgrade the ROS python: `python3 -m pip install 'protobuf>=7.35.1'`. |
| Build fails with `No module named 'em'` | The `python3` on `PATH` lacks `empy` (e.g. conda base env). Use `/usr/bin/python3` on `PATH` or `pip install empy`. |
| No data on `/joint_states` | litearm-server unreachable. Check `endpoint` (`tcp/<host>:<port>`), that the server is up, and that `arm_id` matches a registered arm. |
| `movej` returns `success: True` but the arm does not move | Check the arm state via `/litearm/get_state`; the arm may be disabled or in a stop state — call `/litearm/enable` / `/litearm/clear_stop`. |
| Commands feel stuck / queued | `movej`/`movel` are **blocking** RPCs — the node (and a `/litearm/cmd_joint` publish) blocks until the motion finishes. Issue commands at a sensible rate. |
| Service type not found (`litearm_interfaces/srv/...`) | The interface package is not sourced. Rebuild with `colcon build` and `source install/setup.bash` from the workspace root. |

## Workspace layout

```text
litearm-ros2/
├── litearm_interfaces/        ament_cmake package: custom srv interfaces
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── srv/{Movej,Movel,Fk,Ik,GetState}.srv
├── litearm_ros2/              ament_python package: bridge node
│   ├── package.xml
│   ├── setup.py
│   ├── src/litearm_ros2/
│   │   ├── bridge.py          ROS-agnostic bridge logic (no rclpy imports)
│   │   ├── pose_utils.py      pure-Python litearm ⇄ ROS pose conversion
│   │   └── litearm_node.py    rclpy bridge node
│   ├── launch/litearm.launch.py
│   ├── config/litearm.yaml
│   └── tests/                 mock unit tests (no ROS required)
├── docs/
└── .markdownlint.json
```

## Documentation & license

- Developer guide & API reference (interface details, pose conventions, core
  internals): [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
- 中文使用说明：[README.zh-CN.md](README.zh-CN.md)
- License: see `package.xml` (`MIT`).
