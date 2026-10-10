# litearm-ros2

LiteArm 机械臂的 ROS 2 驱动栈：一个自己持有机械臂 USB 链路的独立维护节点，以及它所用的消息包。
面向希望在链路里没有 `controller_manager` 的前提下使用
[litearm-cpp](https://github.com/nexform-tech/litearm-cpp) SDK 管理类指令的 ROS 2 Humble 用户。

驱动自己打开机械臂的 USB CDC 串口，并把每一次 SDK 调用串行化在同一个互斥回调组里，因此两条指令
不会在链路上交错。运动控制不是它的职责：轨迹、伺服与遥操作属于
[litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control) 里的 ros2_control 插件。

**不要让驱动节点与控制栈同时运行。** 两者都会打开同一个串口，SDK 会对其加独占 `flock`，第二个
进程会打不开串口。启动一个之前先停掉另一个。

## 包组成

| 包 | 是什么 |
| --- | --- |
| `litearm_driver` | 生命周期节点：十个分组共 55 个服务、三个话题、14 个参数。configure 时创建服务，deactivate 时销毁。 |
| `litearm_msgs` | 驱动使用的消息与服务定义：`LitearmStatus`、`JointParam`，以及每个 SDK 调用对应的服务。 |

启动命令见 [docs/quickstart.zh-CN.md](docs/quickstart.zh-CN.md)（英文原文为
[docs/quickstart.md](docs/quickstart.md)）；完整指令集、每条拒绝规则及其原因见
[docs/command-set.zh-CN.md](docs/command-set.zh-CN.md)（英文原文为
[docs/command-set.md](docs/command-set.md)）。

## 特点

- **没有守护进程，也没有共享内存。** 节点直接通过 USB CDC 与固件通信。
- **闸门先于组帧。** 参数被拒时链路上什么都不会发出，因此不合法的负载质量或速度百分比到不了电机。
- **运动需要显式开启。** 每一个能给电机上电的服务，在 `allow_motion` 打开之前都会被拒；
  `allow_dfu` 是固件升级入口的闸门。
- **离线测试。** 测试套件注入 SDK 的 `FakeTransport`，不接机械臂就能跑完整个生命周期。

## 环境要求

| 项目 | 要求 |
| --- | --- |
| ROS 2 | Humble |
| 构建 | `ament_cmake`，C++17 |
| SDK | `litearm-cpp`，已安装或作为同级源码树 |
| 固件 | `Litearm1.5.0` 或更新 —— 版本串形如 `Litearm<主.次.修>-{7J\|1J}` |
| 连接 | USB CDC 串口，`VID:PID 1d50:606f` |

Linux 下需要授予串口权限：

```bash
sudo usermod -aG dialout $USER      # 重新登录后生效
```

## 构建

除非 `CMAKE_PREFIX_PATH` 上已经有装好的 `litearm` CMake 包，否则两个包都需要 `litearm-cpp` 源码树
作为同级目录：

```text
~/litearm_ws/src/
├── litearm-cpp/              # C++ SDK
└── litearm-ros2/             # 本仓库
```

```bash
source /opt/ros/humble/setup.bash
cd ~/litearm_ws
colcon build
source install/setup.bash
```

如果 `litearm-cpp` 签出在别处，用 CMake 参数指过去：

```bash
colcon build --cmake-args -DLITEARM_CPP_DIR=/path/to/litearm-cpp
```

## 运行

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

launch 文件把节点放进 `litearm` 命名空间，因此它那些相对服务名会变成 `/litearm/enable`、
`/litearm/clear_faults` 等等。除非传 `autostart:=false`，否则它会完成 configure 与 activate。

每一个能让臂动的服务，在闸门打开之前都会被拒。闸门每次调用重新读取，所以不需要重启：

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
```

会让臂动之前先支撑好机械臂，并让急停在手边。

## 测试

测试套件完全离线 —— 它注入 SDK 的 `FakeTransport`，不接机械臂就能跑完整个生命周期与每一个服务
处理器：

```bash
cd ~/litearm_ws
colcon test --packages-select litearm_msgs litearm_driver --event-handlers console_direct+
colcon test-result --verbose
```

## 相关仓库

| 仓库 | 内容 |
| --- | --- |
| [litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control) | ros2_control 的 `SystemInterface` 插件，`joint_trajectory_controller` 与 MoveIt 2 的控制通路。 |
| [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) | 两个仓库共同构建所依赖的 C++ SDK。 |

## 许可证

Apache-2.0，见 [LICENSE](LICENSE)。
