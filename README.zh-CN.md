# litearm-ros2

[ROS2](https://docs.ros.org/en/humble) 桥接工作区，用于 LiteArm 机械臂。封装
[litearm-python](../litearm-python)，让机械臂以标准 ROS2 话题、坐标变换和服务
出现：关节状态、TCP 位姿 + TF、关节指令，以及 `movej`/`movel`/`fk`/`ik` 服务。

工作区包含两个标准包：

| 包 | 类型 | 作用 |
|---|---|---|
| `litearm_interfaces` | ament_cmake | 自定义服务：`Movej`、`Movel`、`Fk`、`Ik`、`GetState` |
| `litearm_ros2` | ament_python | 桥接节点 + 与 ROS 无关的核心（`bridge.py`、`pose_utils.py`） |

## 特性

- 📡 **发布** —— `/joint_states`、`/litearm/tcp_pose`、TF `base_link → tool0`
- 🎮 **订阅** —— `/litearm/cmd_joint`（JointState → movej）、`/litearm/stop`（急停）
- 🛠️ **服务** —— `/litearm/movej`、`/litearm/movel`、`/litearm/fk`、`/litearm/ik`、
  `/litearm/get_state` + std_srvs Trigger（`request_stop`/`clear_stop`/`enable`/`disable`）
- 🧪 **与 ROS 无关的核心** —— `bridge.py` + `pose_utils.py` 不含 `rclpy` import，
  可在任意机器上单测

> 📖 完整开发指南与 API 参考：[docs/DEVELOPER_GUIDE.md](docs/DEVELOPER_GUIDE.md)
> · English: [README.md](README.md)

## 环境要求

- ROS2 **Humble**（已验证）+ `colcon`
- Python 3.10+（含 `rclpy`）
- [litearm-python](https://pypi.org/project/litearm-python)（pip 安装；**无法**用 rosdep 解析）

## 构建

```bash
source /opt/ros/humble/setup.bash
cd <本仓库>                     # 工作区根目录
# 把基础 SDK 装进运行节点的 python：
pip install litearm-python         # 或：pip install -e /path/to/litearm-python
colcon build
source install/setup.bash
```

> 在根目录用 colcon 一次构建两个包。若节点 import 报 protobuf 错误，请在 ROS
> python 环境中升级 `protobuf>=7.35.1`。

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

查看机械臂状态：

```bash
ros2 topic echo /joint_states
ros2 topic echo /litearm/tcp_pose
```

下发运动：

```bash
# 通过订阅话题
ros2 topic pub -1 /litearm/cmd_joint sensor_msgs/msg/JointState \
  '{name: [joint0, joint1, joint2, joint3, joint4, joint5, joint6], \
    position: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]}'
# 或通过服务
ros2 service call /litearm/movej litearm_interfaces/srv/Movej \
  '{q_target: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1], speed: 0.3, settle_s: 0.5}'
```

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
└── tests/                     （各包内；见 litearm_ros2/tests）
```
