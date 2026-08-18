# litearm-ros2 开发指南与 API 参考

`litearm-ros2` 是 [litearm-python](../litearm-python) 的 ROS2（Humble）桥接工作区。
内含两个标准包：`litearm_interfaces`（自定义服务）与 `litearm_ros2`（桥接节点 +
与 ROS 无关的核心）。

```text
ROS2 topics/services ──→ litearm_node.py ──→ LiteArmBridge ──→ litearm.Arm ──→ litearm-server
```

---

## 1. 环境要求与构建

| 项目 | 要求 |
|---|---|
| ROS2 | Humble（已验证）+ `colcon` |
| Python | 3.10+，含 `rclpy` |
| 基础 SDK | `litearm-python`（pip 安装；装进运行节点的 python） |

```bash
source /opt/ros/humble/setup.bash
cd <本仓库>                      # 工作区根目录（两个包）
pip install litearm-python
colcon build
source install/setup.bash
```

构建期需要 `empy`（Debian/Ubuntu 上是 `python3-empy`）——若构建报
`No module named 'em'`，说明 `PATH` 上的 `python3` 缺 empy（例如 conda base
环境）。把 `PATH` 指向 `/usr/bin`，或往该环境装 `python3-empy`。

在任意机器上运行 mock 单测（无需 ROS）：

```bash
cd litearm_ros2 && PYTHONPATH=src python3 -m pytest tests/ -q
```

## 2. 包拆分

`litearm_interfaces`（ament_cmake）通过 `rosidl_generate_interfaces` 生成自定义
服务；`litearm_ros2`（ament_python）依赖它，运行时以 `litearm_interfaces.srv.*`
导入生成的类。

接口包单独存在，是因为 ROS2 的 rosidl 代码生成发生在 CMake 阶段；ament_python
包无法自行生成接口。colcon 从工作区根目录构建两者，`setup.bash` 让接口对节点可见。

## 3. 与 ROS 无关的核心

`litearm_ros2/src/litearm_ros2/bridge.py`（`LiteArmBridge`）把 `litearm.Arm`
适配成纯 dict/tuple；`pose_utils.py` 负责 litearm 位姿格式
（`[position(3), rotation(3×3 row-major)]`）与 ROS
`(x, y, z, qx, qy, qz, qw)` 互转。两者都不 import `rclpy`——节点
（`litearm_ros2/src/litearm_ros2/litearm_node.py`）是唯一的 ROS 层。

`LiteArmBridge` 主要方法（均返回 `(bool, message)` 或纯数据）：

| 方法 | 结果 |
|---|---|
| `read_joint_state()` | `{name, position, velocity, effort}` 或 `None` |
| `read_tcp_pose()` | `(x, y, z, qx, qy, qz, qw)` 或 `None` |
| `get_state()` | `{q, dq, tau, state, fault}` 或 `None` |
| `movej(q, speed, settle_s)` / `movel(xyz_quat, speed, settle_s)` | `(bool, message)` |
| `fk(q)` / `ik(xyz_quat, q_seed)` | `(data, bool, message)` |
| `request_stop()` / `clear_stop()` / `enable()` / `disable()` | `(bool, message)` |
| `close()` | — |

## 4. 节点接口

`litearm_node.py`（`LiteArmNode`，rclpy）：

### 发布

| 话题 | 类型 | 内容 |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | `q` / `dq` / `tau`，频率 `loop_hz` |
| `/litearm/tcp_pose` | `geometry_msgs/PoseStamped` | `base_link` 系下的 TCP 位姿 |
| `/tf` | `tf2_ros TransformBroadcaster` | `base_link → tool0` |

### 订阅

| 话题 | 类型 | 动作 |
|---|---|---|
| `/litearm/cmd_joint` | `sensor_msgs/JointState` | `movej(position, speed=cmd_speed)` |
| `/litearm/stop` | `std_msgs/Empty` | 急停 `request_stop()` |

**服务**（自定义 `litearm_interfaces/srv/*.srv`，响应均含 `success` + `message`）

| 服务 | 请求 | 响应 |
|---|---|---|
| `/litearm/movej` | `float64[] q_target, float64 speed, float64 settle_s` | `bool success, string message` |
| `/litearm/movel` | `geometry_msgs/Pose pose, float64 speed, float64 settle_s` | `bool success, string message` |
| `/litearm/fk` | `float64[] q` | `geometry_msgs/Pose pose, bool success, string message` |
| `/litearm/ik` | `geometry_msgs/Pose pose, float64[] q_seed` | `float64[] q, bool success, string message` |
| `/litearm/get_state` | *（空）* | `float64[] q/dq/tau, string state, bool fault, string message` |
| `/litearm/request_stop`、`/litearm/clear_stop`、`/litearm/enable`、`/litearm/disable` | `std_srvs/Trigger` | `bool success, string message` |

**参数**（launch 参数 + `config/litearm.yaml`）：`endpoint`、`arm_id`、
`loop_hz`、`cmd_speed`、`joint_names`、`base_frame`、`tcp_frame`。构造器入参优先
于参数；launch 文件以参数方式传入。

## 5. 位姿约定

- litearm 位姿：`[position(3), rotation(3×3 row-major matrix)]`
- ROS 位姿：`(x, y, z, qx, qy, qz, qw)`

`xyz_quat_to_litearm_pose` / `litearm_pose_to_xyz_quat` 用纯 Python（无 numpy、
无 ROS）双向转换。输出四元数会归一化。

## 6. 注意事项

- `litearm-python` **无法**用 rosdep 解析；请用 pip 装进运行节点的解释器，
  若 import 报错需升级 `protobuf>=7.35.1`。
- `movej`/`movel` 是**阻塞** RPC——服务调用在运动完成时才返回。`/litearm/cmd_joint`
  订阅回调同样会阻塞到运动结束；请以合理频率下发指令。
- 节点必须在**同时有** `rclpy` 与 `litearm-python` 的 python 中运行。标准 Humble
  安装下通常是系统 python：先 `pip install litearm-python` 并升级 protobuf。
