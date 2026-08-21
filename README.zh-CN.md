# litearm-ros2

面向 **LiteArm** 机械臂的 [ROS2](https://docs.ros.org/en/humble) 桥接工作区。
封装 [litearm-python](https://pypi.org/project/litearm-python)，让机械臂以标准
ROS2 话题、坐标变换和服务出现：关节状态、TCP 位姿 + TF、关节指令，以及
`movej`/`movel`/`fk`/`ik` 服务。

工作区包含两个标准包：

| 包 | 类型 | 作用 |
|---|---|---|
| `litearm_interfaces` | ament_cmake | 自定义服务：`Movej`、`Movel`、`Fk`、`Ik`、`GetState` |
| `litearm_ros2` | ament_python | 桥接节点 + 与 ROS 无关的核心（`bridge.py`、`pose_utils.py`） |

```text
ROS2 话题 / 服务 ──→ litearm_node ──→ LiteArmBridge ──→ litearm.Arm ──→ litearm-server
```

> 📖 完整开发指南与 API 参考：[docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · English: [README.md](README.md)

---

## 目录

- [概述与架构](#概述与架构)
- [环境要求](#环境要求)
- [安装与构建](#安装与构建)
- [启动](#启动)
- [节点接口](#节点接口)
- [使用示例](#使用示例)
- [TF 与可视化](#tf-与可视化)
- [常见问题排查](#常见问题排查)
- [工作区结构](#工作区结构)
- [文档与许可证](#文档与许可证)

---

## 概述与架构

本工作区是一个 **colcon** 工作区，含两个包。接口包 `litearm_interfaces` 存放
自定义服务（必须是 ament_cmake 包，因为 ROS2 在 CMake 阶段生成接口）；桥接包
`litearm_ros2` 提供单个节点 `litearm_node`，通过 Zenoh endpoint 连接运行中的
**litearm-server**：

| 方向 | ROS2 侧 | 机械臂侧 |
|---|---|---|
| 出（状态） | `/joint_states`、`/litearm/tcp_pose`、`/tf` | `Arm.get_state()`、`Arm.get_tcp_pose()` |
| 入（指令） | `/litearm/cmd_joint`、`/litearm/stop` | `Arm.movej()`、`Arm.request_stop()` |
| 入（服务） | `/litearm/movej`、`/litearm/movel`、`/litearm/fk`、`/litearm/ik`、`/litearm/get_state`、`/litearm/request_stop`、`/litearm/clear_stop`、`/litearm/enable`、`/litearm/disable` | `Arm.*` RPC |

桥接逻辑位于 `litearm_ros2/src/litearm_ros2/bridge.py` + `pose_utils.py`，
**不含 `rclpy` import** —— 只操作纯 dict/tuple，与 ROS1 桥接共享，且可在任意
机器上无 ROS 单测。

## 环境要求

| 项目 | 要求 |
|---|---|
| ROS2 | **Humble**（已验证）+ `colcon` |
| Python | 3.10+，含 `rclpy` |
| 基础 SDK | `litearm-python`（pip 安装；**无法**用 rosdep 解析） |
| 运行时 | 可达的 **litearm-server**（Zenoh endpoint，例如 `tcp/192.168.31.237:7447`） |

> `litearm-python` 通过 Zenoh 与 litearm-server 通信；服务器持有真实硬件或仿真。
> 启动前请确认其已运行。

## 安装与构建

```bash
source /opt/ros/humble/setup.bash
cd <本仓库>                        # 工作区根目录（两个包）

# 把基础 SDK 装进运行节点的 python：
pip install litearm-python         # 或：pip install -e /path/to/litearm-python

colcon build
source install/setup.bash
```

- **解释器很重要。** 节点要 `import litearm`，所以 `litearm-python` 必须能被提供
  `rclpy` 的同一个 python 导入。标准 Humble 下是系统 python：
  `sudo python3 -m pip install litearm-python`；若节点 import 报 protobuf 错误，
  在同一解释器中升级 `protobuf>=7.35.1`。
- **构建期需要 `empy`。** colcon 依赖 `empy`（Debian/Ubuntu 上是 `python3-empy`）。
  若构建报 `No module named 'em'`，说明 `PATH` 上的 `python3` 缺 empy（例如 conda
  base 环境）。把 `PATH` 指向 `/usr/bin`，或往该环境装 `python3-empy`。

在任意机器上运行 mock 单测（无需 ROS）：

```bash
cd litearm_ros2 && PYTHONPATH=src python3 -m pytest tests/ -q
```

## 启动

```bash
ros2 launch litearm_ros2 litearm.launch.py \
  endpoint:=tcp/192.168.31.237:7447   # litearm-server 的地址
```

或直接运行节点：

```bash
ros2 run litearm_ros2 litearm_node --ros-args \
  -p endpoint:=tcp/192.168.31.237:7447 -p arm_id:=armA
```

launch 文件支持的参数如下（默认值来自 `config/litearm.yaml`，以节点参数加载）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `endpoint` | `tcp/192.168.31.237:7447` | litearm-server 的 Zenoh endpoint |
| `arm_id` | `armA` | 服务器上注册的机械臂 id |
| `loop_hz` | `50.0` | 状态发布频率（Hz） |
| `cmd_speed` | `0.5` | 默认关节运动速度（0..1） |
| `joint_names` | `joint0 … joint6` | `/joint_states` 中的关节名 |
| `base_frame` | `base_link` | 机械臂基座坐标系 |
| `tcp_frame` | `tool0` | 工具中心点坐标系 |

所有参数也可通过命令行 `--ros-args -p` 覆盖，或在 `config/litearm.yaml` 中修改
（见[参数](#参数)）。

## 节点接口

### 发布

| 话题 | 类型 | 内容 |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | `q` / `dq` / `tau`，以 `loop_hz` 发布 |
| `/litearm/tcp_pose` | `geometry_msgs/msg/PoseStamped` | `base_frame` 系下的 TCP 位姿 |
| `/tf` | `tf2_ros` 变换广播 | `base_frame → tcp_frame` 变换 |

### 订阅

| 话题 | 类型 | 动作 |
|---|---|---|
| `/litearm/cmd_joint` | `sensor_msgs/msg/JointState` | `movej(position, speed=cmd_speed)` |
| `/litearm/stop` | `std_msgs/msg/Empty` | 急停：`request_stop()` |

### 服务

自定义服务（`litearm_interfaces/srv/*.srv`）。每个响应都含 `success` + `message`。

| 服务 | 请求 | 响应 |
|---|---|---|
| `/litearm/movej` | `float64[] q_target, float64 speed, float64 settle_s` | `bool success, string message` |
| `/litearm/movel` | `geometry_msgs/Pose pose, float64 speed, float64 settle_s` | `bool success, string message` |
| `/litearm/fk` | `float64[] q` | `geometry_msgs/Pose pose, bool success, string message` |
| `/litearm/ik` | `geometry_msgs/Pose pose, float64[] q_seed` | `float64[] q, bool success, string message` |
| `/litearm/get_state` | *（空）* | `float64[] q/dq/tau, string state, bool fault, string message` |
| `/litearm/request_stop` | `std_srvs/srv/Trigger` | `bool success, string message` |
| `/litearm/clear_stop` | `std_srvs/srv/Trigger` | `bool success, string message` |
| `/litearm/enable` | `std_srvs/srv/Trigger` | `bool success, string message` |
| `/litearm/disable` | `std_srvs/srv/Trigger` | `bool success, string message` |

### 参数

已声明的节点参数，默认值如下（launch 参数与 `config/litearm.yaml` 都会喂给它）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `endpoint` | `tcp/192.168.31.237:7447` | litearm-server 的 Zenoh endpoint |
| `arm_id` | `armA` | 服务器上注册的机械臂 id |
| `loop_hz` | `50.0` | 状态发布频率（Hz） |
| `cmd_speed` | `0.5` | `/litearm/cmd_joint` 使用的默认关节速度（0..1） |
| `joint_names` | `joint0 … joint6` | `/joint_states` 中的关节名 |
| `base_frame` | `base_link` | 机械臂基座坐标系 |
| `tcp_frame` | `tool0` | 工具中心点坐标系 |

## 使用示例

### 查看机械臂状态

```bash
ros2 topic echo /joint_states
ros2 topic echo /litearm/tcp_pose
ros2 run tf2_ros tf2_echo base_link tool0     # 实时查看两坐标系间的 TF
```

### 通过话题下发关节空间运动

```bash
ros2 topic pub -1 /litearm/cmd_joint sensor_msgs/msg/JointState \
  '{name: [joint0, joint1, joint2, joint3, joint4, joint5, joint6], \
    position: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}'
```

`position` 会以节点的 `cmd_speed` 传给 `movej()`。

### 通过服务下发关节空间运动

```bash
ros2 service call /litearm/movej litearm_interfaces/srv/Movej \
  '{q_target: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], speed: 0.3, settle_s: 0.5}'
```

`q_target` 是 7 个关节位置，`speed` 是归一化速度（`0..1`），`settle_s` 是运动
到位后的停留时间。

### 运动到笛卡尔位姿

```bash
ros2 service call /litearm/movel litearm_interfaces/srv/Movel \
  '{pose: {position: {x: 0.25, y: 0.0, z: 0.35}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, speed: 0.3, settle_s: 0.5}'
```

位姿为 `base_frame` 系下的 `geometry_msgs/Pose`，姿态用四元数 `(x, y, z, w)`。

### 正解 / 逆解

```bash
# FK：关节 → TCP 位姿
ros2 service call /litearm/fk litearm_interfaces/srv/Fk \
  '{q: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'

# IK：TCP 位姿 → 关节（q_seed 可选，用于帮助选出解）
ros2 service call /litearm/ik litearm_interfaces/srv/Ik \
  '{pose: {position: {x: 0.25, y: 0.0, z: 0.35}, \
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, \
    q_seed: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
```

### 读取完整状态

```bash
ros2 service call /litearm/get_state litearm_interfaces/srv/GetState
# → q, dq, tau 数组、state 字符串、fault 布尔、message
```

### 急停与恢复

```bash
# 立即停止（话题）：
ros2 topic pub -1 /litearm/stop std_msgs/msg/Empty '{}'
# 或通过服务：
ros2 service call /litearm/request_stop std_srvs/srv/Trigger '{}'

# 确认安全后清除急停：
ros2 service call /litearm/clear_stop std_srvs/srv/Trigger '{}'
```

### 使能 / 失能

```bash
ros2 service call /litearm/enable std_srvs/srv/Trigger '{}'
ros2 service call /litearm/disable std_srvs/srv/Trigger '{}'
```

## TF 与可视化

节点每次状态更新都会广播 `base_link → tool0` 变换，任何 TF 消费者都可以跟踪
机械臂。可视化：

```bash
ros2 run rviz2 rviz2
# Add → TF，将 Fixed Frame 设为 base_link
```

> 本工作区不含 URDF 或 MoveIt 配置——它提供实时 TF `base_link → tool0` 与
> `/litearm/tcp_pose` 话题，足以可视化机械臂位姿。若你另有 LiteArm 的 URDF，
> 请用 `base_frame` / `tcp_frame` 对齐其基座/工具坐标系名。

## 常见问题排查

| 现象 | 可能原因 / 解决 |
|---|---|
| `ModuleNotFoundError: No module named 'litearm'` | `litearm-python` 装进了与 `rclpy` 不同的 python。执行 `sudo python3 -m pip install litearm-python`。 |
| import 报 protobuf 相关错误 | 升级 ROS python：`python3 -m pip install 'protobuf>=7.35.1'`。 |
| 构建报 `No module named 'em'` | `PATH` 上的 `python3` 缺 `empy`（例如 conda base 环境）。改用 `/usr/bin/python3`，或 `pip install empy`。 |
| `/joint_states` 无数据 | litearm-server 不可达。检查 `endpoint`（`tcp/<主机>:<端口>`）、服务器是否在运行、`arm_id` 是否匹配已注册机械臂。 |
| `movej` 返回 `success: True` 但机械臂不动 | 用 `/litearm/get_state` 查看状态；机械臂可能被失能或处于急停态——调用 `/litearm/enable` / `/litearm/clear_stop`。 |
| 指令像被卡住 / 排队 | `movej`/`movel` 是**阻塞** RPC——节点（以及发布 `/litearm/cmd_joint`）会阻塞到运动结束。请以合理频率下发指令。 |
| 找不到服务类型（`litearm_interfaces/srv/...`） | 接口包未 source。在工作区根目录重新 `colcon build` 并 `source install/setup.bash`。 |

## 工作区结构

```text
litearm-ros2/
├── litearm_interfaces/        ament_cmake 包：自定义 srv 接口
│   ├── package.xml
│   ├── CMakeLists.txt
│   └── srv/{Movej,Movel,Fk,Ik,GetState}.srv
├── litearm_ros2/              ament_python 包：桥接节点
│   ├── package.xml
│   ├── setup.py
│   ├── src/litearm_ros2/
│   │   ├── bridge.py          ROS 无关桥接逻辑（无 rclpy import）
│   │   ├── pose_utils.py      纯 Python 的 litearm ⇄ ROS 位姿转换
│   │   └── litearm_node.py    rclpy 桥接节点
│   ├── launch/litearm.launch.py
│   ├── config/litearm.yaml
│   └── tests/                 mock 单元测试（无需 ROS）
├── docs/
└── .markdownlint.json
```

## 文档与许可证

- 开发指南与 API 参考（接口细节、位姿约定、核心内部实现）：
  [docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
- English: [README.md](README.md)
- 许可证：见 `package.xml`（`MIT`）。
