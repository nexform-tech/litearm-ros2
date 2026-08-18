# litearm-ros2

[ROS2](https://docs.ros.org/en/humble) bridge workspace for the LiteArm robotic
arm. Wraps [litearm-python](../litearm-python) so the arm shows up as standard
ROS2 topics, transforms and services: joint states, TCP pose + TF, joint
commands, and `movej`/`movel`/`fk`/`ik` services.

The workspace contains two standard packages:

| Package | Type | Role |
|---|---|---|
| `litearm_interfaces` | ament_cmake | Custom services: `Movej`, `Movel`, `Fk`, `Ik`, `GetState` |
| `litearm_ros2` | ament_python | Bridge node + ROS-agnostic core (`bridge.py`, `pose_utils.py`) |

## Features

- 📡 **Publishers** — `/joint_states`, `/litearm/tcp_pose`, TF `base_link → tool0`
- 🎮 **Subscribers** — `/litearm/cmd_joint` (JointState → movej), `/litearm/stop` (E-stop)
- 🛠️ **Services** — `/litearm/movej`, `/litearm/movel`, `/litearm/fk`, `/litearm/ik`,
  `/litearm/get_state` + std_srvs Trigger (`request_stop`/`clear_stop`/`enable`/`disable`)
- 🧪 **ROS-agnostic core** — `bridge.py` + `pose_utils.py` have no `rclpy` imports
  and are unit-tested on any machine

> 📖 Full developer guide & API reference: [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · 中文文档：[README.zh-CN.md](README.zh-CN.md)

## Requirements

- ROS2 **Humble** (tested) with `colcon`
- Python 3.10+ with `rclpy`
- [litearm-python](https://pypi.org/project/litearm-python) (pip; **not** rosdep-resolvable)

## Build

```bash
source /opt/ros/humble/setup.bash
cd <this repo>                     # the workspace root
# install the base SDK into the python that runs the node:
pip install litearm-python         # or: pip install -e /path/to/litearm-python
colcon build
source install/setup.bash
```

> Colcon builds both packages from the workspace root. If the node import fails
> with a protobuf error, upgrade `protobuf>=7.35.1` in the ROS python environment.

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

Inspect the arm state:

```bash
ros2 topic echo /joint_states
ros2 topic echo /litearm/tcp_pose
```

Drive a motion:

```bash
# via the subscriber
ros2 topic pub -1 /litearm/cmd_joint sensor_msgs/msg/JointState \
  '{name: [joint0, joint1, joint2, joint3, joint4, joint5, joint6], \
    position: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}'
# or via the service
ros2 service call /litearm/movej litearm_interfaces/srv/Movej \
  '{q_target: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], speed: 0.3, settle_s: 0.5}'
```

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
└── tests/                     (per-package; see litearm_ros2/tests)
```
