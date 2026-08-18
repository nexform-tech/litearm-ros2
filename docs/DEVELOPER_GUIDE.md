# litearm-ros2 Developer Guide & API Reference

`litearm-ros2` is a ROS2 (Humble) bridge workspace for
[litearm-python](../litearm-python). It ships two standard packages:
`litearm_interfaces` (custom services) and `litearm_ros2` (the bridge node plus
a ROS-agnostic core).

```text
ROS2 topics/services ──→ litearm_node.py ──→ LiteArmBridge ──→ litearm.Arm ──→ litearm-server
```

---

## 1. Requirements & build

| Item | Requirement |
|---|---|
| ROS2 | Humble (tested) with `colcon` |
| Python | 3.10+ with `rclpy` |
| Base SDK | `litearm-python` (pip; install into the python that runs the node) |

```bash
source /opt/ros/humble/setup.bash
cd <this repo>                      # workspace root (both packages)
pip install litearm-python
colcon build
source install/setup.bash
```

Build-time requires `empy` (`python3-empy` on Debian/Ubuntu) — if the build fails
with `No module named 'em'`, the `python3` on `PATH` is missing it (e.g. a conda
base env). Point `PATH` at `/usr/bin` or install `python3-empy` into that env.

Run the mock unit tests on any machine (no ROS needed):

```bash
cd litearm_ros2 && PYTHONPATH=src python3 -m pytest tests/ -q
```

## 2. Package split

`litearm_interfaces` (ament_cmake) generates the custom services via
`rosidl_generate_interfaces`. `litearm_ros2` (ament_python) depends on it and
imports the generated classes as `litearm_interfaces.srv.*`.

The interface package exists because ROS2's rosidl code generation runs in the
CMake phase; an ament_python package cannot generate interfaces by itself. Colcon
builds both from the workspace root and `setup.bash` makes the interfaces visible
to the node.

## 3. The ROS-agnostic core

`litearm_ros2/src/litearm_ros2/bridge.py` (`LiteArmBridge`) adapts a
`litearm.Arm` to plain dicts/tuples; `pose_utils.py` converts between the
litearm pose format (`[position(3), rotation(3×3 row-major)]`) and ROS
`(x, y, z, qx, qy, qz, qw)`. No `rclpy` imports — the node
(`litearm_ros2/src/litearm_ros2/litearm_node.py`) is the only ROS layer.

Key `LiteArmBridge` methods (all return `(bool, message)` or plain data):

| Method | Result |
|---|---|
| `read_joint_state()` | `{name, position, velocity, effort}` or `None` |
| `read_tcp_pose()` | `(x, y, z, qx, qy, qz, qw)` or `None` |
| `get_state()` | `{q, dq, tau, state, fault}` or `None` |
| `movej(q, speed, settle_s)` / `movel(xyz_quat, speed, settle_s)` | `(bool, message)` |
| `fk(q)` / `ik(xyz_quat, q_seed)` | `(data, bool, message)` |
| `request_stop()` / `clear_stop()` / `enable()` / `disable()` | `(bool, message)` |
| `close()` | — |

## 4. Node interfaces

`litearm_node.py` (`LiteArmNode`, rclpy):

### Publishers

| Topic | Type | Contents |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `q` / `dq` / `tau` at `loop_hz` |
| `/litearm/tcp_pose` | `geometry_msgs/PoseStamped` | TCP pose in `base_link` |
| `/tf` | `tf2_ros TransformBroadcaster` | `base_link → tool0` |

### Subscribers

| Topic | Type | Action |
|---|---|---|
| `/litearm/cmd_joint` | `sensor_msgs/JointState` | `movej(position, speed=cmd_speed)` |
| `/litearm/stop` | `std_msgs/Empty` | emergency `request_stop()` |

**Services** (custom `litearm_interfaces/srv/*.srv`, all reply `success` + `message`)

| Service | Request | Response |
|---|---|---|
| `/litearm/movej` | `float64[] q_target, float64 speed, float64 settle_s` | `bool success, string message` |
| `/litearm/movel` | `geometry_msgs/Pose pose, float64 speed, float64 settle_s` | `bool success, string message` |
| `/litearm/fk` | `float64[] q` | `geometry_msgs/Pose pose, bool success, string message` |
| `/litearm/ik` | `geometry_msgs/Pose pose, float64[] q_seed` | `float64[] q, bool success, string message` |
| `/litearm/get_state` | *(empty)* | `float64[] q/dq/tau, string state, bool fault, string message` |
| `/litearm/request_stop`, `/litearm/clear_stop`, `/litearm/enable`, `/litearm/disable` | `std_srvs/Trigger` | `bool success, string message` |

**Parameters** (launch args + `config/litearm.yaml`): `endpoint`, `arm_id`,
`loop_hz`, `cmd_speed`, `joint_names`, `base_frame`, `tcp_frame`. Values passed
as constructor kwargs win over parameters; the launch file passes them as
parameters.

## 5. Pose convention

- litearm pose: `[position(3), rotation(3×3 row-major matrix)]`
- ROS pose: `(x, y, z, qx, qy, qz, qw)`

`xyz_quat_to_litearm_pose` / `litearm_pose_to_xyz_quat` convert both ways with
pure Python (no numpy, no ROS). Quaternions are normalized on output.

## 6. Notes & caveats

- `litearm-python` is **not rosdep-resolvable**; install it with pip into the
  same interpreter that runs the node, and upgrade `protobuf>=7.35.1` if import
  fails.
- `movej`/`movel` are **blocking** RPCs — a service call returns only when the
  motion completes. The `/litearm/cmd_joint` subscriber also blocks until the
  commanded motion finishes; issue commands at a sensible rate.
- The node must run from a python that has **both** `rclpy` and
  `litearm-python`. On a stock Humble install that is usually the system python
  after `pip install litearm-python` + protobuf upgrade into it.
