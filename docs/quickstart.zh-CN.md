# litearm driver 快速上手

把 `litearm_driver` 在真机上跑起来的命令，面向在没有 ros2_control 栈的情况下做开机验收、授权
与参数调试的人。每一节都是按顺序执行的一步；第 5 节是某一步失败时该看的地方。

ros2_control 栈不在本仓库里。如果你要的是控制器、MoveIt 与 RViz，而不是 SDK 的管理类调用，请用
[litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control)。两个入口**互斥**
——都要打开同一个 USB CDC 串口，SDK 会对它加独占 `flock`——所以一次只跑一个。

**不要在 ros2_control 栈运行时跑 driver。** 症状是 driver 的 configure 失败：没有第二个进程能
共用这个串口。

## 0. 先做准备

```bash
source /opt/ros/humble/setup.bash
source ~/litearm_ws/install/setup.bash

lsusb | grep 1d50          # 应看到 1d50:606f
ls /dev/ttyACM*            # 应看到 /dev/ttyACM0
```

## 1. 启动 driver

```bash
ros2 launch litearm_driver litearm_driver.launch.py
```

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `namespace` | `litearm` | 服务所在命名空间，服务直接位于其中。 |
| `autostart` | `true` | 起节点后完成 configure 与 activate。`false` 则只起节点，手动驱动 lifecycle。 |
| `config` | 包内 `config/litearm_driver.yaml` | 参数文件。 |
| `log_level` | `info` | 节点日志级别。 |

driver 不隔离 ROS 域，普通终端可以直接访问。

### 只读启动

`auto_enable` 在参数文件里，默认 `true`。想在不给电机上电的情况下先看臂：

```bash
cp install/litearm_driver/share/litearm_driver/config/litearm_driver.yaml /tmp/driver_no_enable.yaml
sed -i 's/auto_enable: true/auto_enable: false/' /tmp/driver_no_enable.yaml
ros2 launch litearm_driver litearm_driver.launch.py config:=/tmp/driver_no_enable.yaml
```

## 2. 不会让臂动的调用

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{}"
ros2 service call /litearm/get_diagnostics litearm_msgs/srv/GetDiagnostics "{}"
ros2 service call /litearm/get_tcp litearm_msgs/srv/GetTcp "{}"
ros2 service call /litearm/get_feedforward_mask litearm_msgs/srv/GetFeedforwardMask "{}"
ros2 service call /litearm/get_feedforward_catalog litearm_msgs/srv/GetFeedforwardCatalog "{}"
ros2 service call /litearm/inverse_kinematics litearm_msgs/srv/InverseKinematics \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], seed: [], timeout: 0.5}"
ros2 service call /litearm/log_start litearm_msgs/srv/LogStart "{ticks: 600}"
ros2 service call /litearm/log_dump litearm_msgs/srv/LogDump \
  "{filename: tick_log.bin, wait: true, timeout: 3.0}"
```

`get_feedforward_catalog` 直接答 SDK 的静态表，完全不需要链路；`log_dump` 写进节点的 `log_dir`
（默认 `$HOME/.ros/litearm`），并且拒绝把路径当作 `filename`。

## 3. 让机械臂动起来

每一个能给电机上电的服务，在 `allow_motion` 打开之前都会被拒。它与 `allow_dfu` 一样每次调用重新
读取，所以不需要重启：

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
ros2 service call /litearm/move_l litearm_msgs/srv/MoveL \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], speed: 0.2, wait: true}"
```

`speed` 是满速的比例 `0 < speed <= 1`；`30` 表示 30 倍，不是 30%。百分比限速用
`/litearm/set_speed_scaling`，状态消息里的 `speed_scaling` 只是主机侧对上一次写入的记录。

**会让臂动之前先支撑好机械臂，并让急停在手边。** `move_js`、`send_mit`、`send_mit_all` 都是单帧：
固件的 100 ms 看门狗会在没人持续重发时把臂交回持位。

## 4. 交还机械臂

```bash
# 在跑着 driver 的那个终端按 Ctrl-C，等进程真正退出。
fuser -v /dev/ttyACM0 2>/dev/null      # 必要时看谁还占着串口
```

两套部署不能重叠的原因是**串口**，不是某个 ROS 资源：driver 并不持有任何其他部署的共享内存锁。

## 5. 起不来时

| 现象 | 原因与处理 |
| --- | --- |
| `未找到 STM32 CDC (VID:PID 1d50:606f)` | 板子没插、没上电，或处在 DFU。先看 `lsusb`；DFU 下会枚举成 `0483:DF11`，需要刷固件才能回来。 |
| 刚才 ros2_control 栈跑过，configure 就失败 | 串口仍被占用。停掉另一个进程再试，没有第二个进程能共用它。 |
| `ENABLE was refused: ... ERR{0x10,0x06}` | 固件里有锁存故障（EMERGENCY 或 `joint_fault`）。若是授权原因，driver 的状态消息里能看到授权字段；用 `/litearm/reset` 清除。 |
| 运动服务返回关于 `allow_motion` 的拒绝 | 闸门没开。用 `ros2 param set /litearm/driver allow_motion true` 打开。 |
| `enter_dfu` 被拒 | `allow_dfu` 默认 `false`。用 `ros2 param set /litearm/driver allow_dfu true` 打开。 |

## 6. 接下来看什么

- [`docs/command-set.zh-CN.md`](command-set.zh-CN.md) —— 每个 driver 服务、它的拒绝规则与背后的
  链路语义（英文原文：[`docs/command-set.md`](command-set.md)）。
- [`litearm_driver/README.zh-CN.md`](../litearm_driver/README.zh-CN.md) —— driver 的参数、话题与
  字段级服务参考。
- [`litearm_msgs/README.md`](../litearm_msgs/README.md) —— 客户端编译所依赖的消息与服务定义。
