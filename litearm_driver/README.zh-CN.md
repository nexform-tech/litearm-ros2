# litearm_driver

独立运行的 ROS 2 生命周期节点：自己持有 LiteArm 机械臂的 USB 链路，把 SDK 的管理类指令以服务形式
暴露出来，用于在没有 controller_manager 参与的情况下维护、授权或整定机械臂。

[English](README.md) · 简体中文

节点经 [litearm-cpp](https://github.com/nexform-tech/litearm-cpp) SDK 与固件通信。服务在 activate
时创建、deactivate 时销毁，所有 SDK 调用由同一个回调组串行化。

**本节点与 ros2_control 控制栈互斥。** 两者都要打开同一个串口，而 SDK 会对其加独占 `flock`，
所以第二个进程打不开串口。启用其中一个前请先停掉另一个。

## 运行

```bash
ros2 launch litearm_driver litearm_driver.launch.py
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

launch 文件把节点放在 `litearm` 命名空间下，因此相对服务名会变成 `/litearm/enable`、
`/litearm/clear_faults` 等。下文所有服务名都相对于该命名空间书写。

整套启动（launch 参数、只读启动、不会让臂动的调用）见
[`docs/quickstart.zh-CN.md`](../docs/quickstart.zh-CN.md)。ros2_control 栈是另一个仓库：
[litearm-ros2-control](https://github.com/nexform-tech/litearm-ros2-control)。

## 指令集

节点提供 10 组共 55 个服务、3 个话题与 14 个参数。每个服务都返回 `success` 与 `message`；失败时
message 是 SDK 自己的原文，若是被门禁在发帧前拦下，则会说明拒绝的理由。

三条规则决定了整个接口的形状：

- **门禁先于发帧。** 被拒的参数不会在链路上留下任何字节。节点拒绝的是那些"固件会静默改成另一个值"
  的请求：超出 `[0, 20]` kg 的载荷质量、超出 `0..100` 的速度百分比、超过 100 ms 命令看门狗的零重力
  保活周期、非有限的前馈数值。
- **同一时刻只有一条命令。** 所有 SDK 调用都在同一个互斥回调组里执行，因此两条命令绝不可能在链路上
  交错。命令需要等固件 ACK，所以链路卡住时一次服务调用最多会被拖到 SDK 自己的 1.2 s 超时。
- **读取走缓存。** 只有按需读取的服务（`get_status`、`get_tcp`、`get_feedforward_*`、`get_joint_params`、模型读取、`kin_bench`、日志服务与
`poll_cart`）会与固件通信；`get_feedforward_catalog` 直接答静态表，连链路都不需要。`/litearm/status` 从不发帧，因此订阅它不会干扰在途命令。

### 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `""` | USB CDC 设备路径。留空表示按 VID:PID `1d50:606f` 自动发现。 |
| `joint_names` | 未设置 | 按固件轴序给出的轴名。未设置时由固件上报的轴数推导 `joint1`..`jointN`；长度不对会让 configure 失败。 |
| `auto_enable` | `true` | activate 时使能电机。ENABLE 被拒会让 activate 失败，而不是留下一个"激活但电机没力"的节点。 |
| `enable_attempts` | `12` | `enable` 服务的重试次数；只有固件标注"可重试"的错误码才消耗重试。 |
| `status_rate_hz` | `10.0` | `/litearm/status` 发布频率。 |
| `publish_joint_states` | `false` | 是否发布 `joint_states_topic`。只要有 `joint_state_broadcaster` 在跑就保持 false。 |
| `joint_states_topic` | `/joint_states` | 可选关节状态话题名。 |
| `publish_diagnostics` | `true` | 是否发布 `/diagnostics`。 |
| `diagnostics_rate_hz` | `1.0` | `/diagnostics` 发布频率。 |
| `zero_g_keepalive_period_s` | `0.04` | 零重力保活周期，取值 `[0.005, 0.10)`。每次 `zero_g` 调用都会重新读取。 |
| `allow_dfu` | `false` | `enter_dfu` 的开关，每次调用重新读取。 |
| `allow_motion` | `false` | 所有运动服务（`move_j`、`move_p`、`move_js`、`send_mit*`、`home`、`move_l`、`move_c`、`move_path`）的开关，每次调用重新读取。 |
| `log_dir` | `""` | `log_dump` 的落盘目录。留空表示 `$HOME/.ros/litearm`。 |
| `frame_id` | `""` | 状态与关节状态消息的 header frame id。 |

### 话题

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/litearm/status` | `litearm_msgs/msg/LitearmStatus` | 以 `status_rate_hz` 发布缓存与本地 SDK 状态，从不发帧。 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 一条状态：链路、使能、故障、掉线持位推断、授权、反馈新鲜度。 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 仅在 `publish_joint_states:=true` 时发布。默认关闭，避免与 `joint_state_broadcaster` 抢同一个话题。 |

`LitearmStatus` 按 `joint_names` 顺序携带 `temperature_mos`、`temperature_coil`、
`joint_error_code` 与 `joint_fault` 位图。当前帧没有覆盖到的轴，温度读 `NaN`、错误码读 `0`；
首帧到达前 `feedback_age` 为 `-1`。`speed_scaling` 与 `zero_g_*` 是主机侧视图，不是固件回读值。

### 服务索引

| 服务 | 类型 | 用途 |
| --- | --- | --- |
| `enable` | `std_srvs/srv/Trigger` | 使能全部轴。 |
| `disable` | `std_srvs/srv/Trigger` | 失能全部轴；机械臂不再被持位。 |
| `reset` | `std_srvs/srv/Trigger` | 清故障并重新锚定位姿。 |
| `clear_faults` | `std_srvs/srv/Trigger` | 只清 RAM 里的故障位。 |
| `emergency_stop` | `std_srvs/srv/Trigger` | 请求急停。 |
| `park` | `std_srvs/srv/Trigger` | 声明全刚度静态持位。 |
| `zero_g` | `std_srvs/srv/SetBool` | 进入或退出零重力（手动引导）。 |
| `get_status` | `litearm_msgs/srv/GetStatus` | 主动发 `GET_STATUS` 并返回一次快照。 |
| `reconnect` | `litearm_msgs/srv/Trigger` | 拔插线缆后重建链路。 |
| `home` | `litearm_msgs/srv/Trigger` | 以固件低速走回 URDF 零位。 |
| `get_tcp` | `litearm_msgs/srv/GetTcp` | 读取固件当前的工具位姿。 |
| `get_diagnostics` | `litearm_msgs/srv/GetDiagnostics` | 主机计数器、各 id 报文速率、链路与能力标志。 |
| `kin_bench` | `litearm_msgs/srv/KinBench` | 跑固件的运动学基准并解析回执。 |
| `move_j` | `litearm_msgs/srv/MoveJ` | 关节空间移动，固件逐轴 S 曲线。 |
| `move_j_sync` | `litearm_msgs/srv/MoveJSync` | 关节空间移动，各轴共用一条同步曲线。 |
| `move_p` | `litearm_msgs/srv/MoveP` | 位姿移动：固件自己解 IK 并走曲线。 |
| `move_js` | `litearm_msgs/srv/MoveJs` | 发一帧 MOVE_JS，即流式原语。 |
| `send_mit` | `litearm_msgs/srv/SendMit` | 单轴 MIT 透传。 |
| `send_mit_all` | `litearm_msgs/srv/SendMitAll` | 全臂 MIT 透传，一帧。 |
| `move_l` | `litearm_msgs/srv/MoveL` | 固件规划的直线笛卡尔运动。 |
| `move_c` | `litearm_msgs/srv/MoveC` | 经过途经点的圆弧笛卡尔运动。 |
| `move_path` | `litearm_msgs/srv/MovePath` | 多路点笛卡尔路径。 |
| `poll_cart` | `litearm_msgs/srv/PollCart` | 查询在途笛卡尔请求的结局。 |
| `inverse_kinematics` | `litearm_msgs/srv/InverseKinematics` | 把位姿解算成关节角，不产生运动。 |
| `get_feedforward_vector` | `litearm_msgs/srv/GetFeedforwardVector` | 回读一条前馈向量。 |
| `get_feedforward_mask` | `litearm_msgs/srv/GetFeedforwardMask` | 回读前馈使能掩码。 |
| `get_feedforward_catalog` | `litearm_msgs/srv/GetFeedforwardCatalog` | 列出 SDK 的条目表，不需要链路。 |
| `set_gravity_scale` | `litearm_msgs/srv/SetGravityScale` | 按轴缩放重力前馈。 |
| `set_inertia_scale` | `litearm_msgs/srv/SetInertiaScale` | 按轴缩放惯量前馈。 |
| `set_gravity_vector` | `litearm_msgs/srv/SetGravityVector` | 写模型使用的重力向量。 |
| `probe_model` | `litearm_msgs/srv/ProbeModel` | 探测固件是否提供动力学模型存储。 |
| `get_model_body` | `litearm_msgs/srv/GetModelBody` | 读取动力学模型的单个刚体。 |
| `set_model_body` | `litearm_msgs/srv/SetModelBody` | 把单个刚体暂存进 RAM。 |
| `get_model_jm` | `litearm_msgs/srv/GetModelJm` | 读取关节空间模型项。 |
| `set_model_jm` | `litearm_msgs/srv/SetModelJm` | 把关节空间模型项暂存进 RAM。 |
| `commit_model` | `litearm_msgs/srv/CommitModel` | 把暂存的模型固化到 flash。 |
| `revert_model` | `litearm_msgs/srv/Trigger` | 丢弃暂存的模型。 |
| `get_model_status` | `litearm_msgs/srv/GetModelStatus` | 覆写级别、暂存掩码与 dirty 标志。 |
| `log_start` | `litearm_msgs/srv/LogStart` | 开始录制固件 300 Hz 控制拍。 |
| `log_stop` | `litearm_msgs/srv/Trigger` | 停止录制。 |
| `log_dump` | `litearm_msgs/srv/LogDump` | 把录制读回并落盘成文件。 |
| `set_speed_scaling` | `litearm_msgs/srv/SetSpeedScaling` | 写全局速度倍率（百分比）。 |
| `set_motion_mode` | `litearm_msgs/srv/SetMotionMode` | 写固件运动模式。 |
| `set_payload` | `litearm_msgs/srv/SetPayload` | 声明末端载荷质量与质心。 |
| `set_feedforward_mask` | `litearm_msgs/srv/SetFeedforwardMask` | 写前馈使能掩码。 |
| `set_feedforward_preset` | `litearm_msgs/srv/SetFeedforwardPreset` | 选择前馈预设。 |
| `set_feedforward_scalar` | `litearm_msgs/srv/SetFeedforwardScalar` | 写一个前馈标量。 |
| `set_feedforward_vector` | `litearm_msgs/srv/SetFeedforwardVector` | 写一个前馈向量。 |
| `get_feedforward_scalar` | `litearm_msgs/srv/GetFeedforwardScalar` | 回读一个标量。 |
| `get_joint_params` | `litearm_msgs/srv/GetJointParams` | 读关节参数表。 |
| `set_joint_gains` | `litearm_msgs/srv/SetJointGains` | 写某个轴的 `kp`/`kd`/`tau_max`。 |
| `set_joint_limits` | `litearm_msgs/srv/SetJointLimits` | 写某个轴的软限位。 |
| `reset_factory_params` | `std_srvs/srv/Trigger` | 恢复出厂关节参数。 |
| `save_params` | `std_srvs/srv/Trigger` | 把运行时参数写入 Flash。 |
| `enter_dfu` | `std_srvs/srv/Trigger` | 让设备登记进入 ROM bootloader。 |

### 状态与安全

#### `enable`

`Arm::enable(enable_attempts)`。只有固件标注可重试的错误码会被重试；锁存故障或缺授权会立即失败。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。授权必须已激活，否则固件以 `ERR{0x10,0x08}` 拒绝 ENABLE。 |

```bash
ros2 service call /litearm/enable std_srvs/srv/Trigger "{}"
```

#### `disable`

`Arm::disable()`。它绕过零重力守卫，因为"降低能量"是安全方向。

**不要**在机械臂没有支撑时调用 `disable`：位置环不再持位，负载会掉下来。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/disable std_srvs/srv/Trigger "{}"
```

#### `reset`

`Arm::reset()`：清故障并重新锚定位姿。不是 MCU 重启。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/reset std_srvs/srv/Trigger "{}"
```

#### `clear_faults`

`Arm::clear_faults()`：只清 RAM 里的故障位。需要同时重新锚定位姿参考时用 `reset`。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/clear_faults std_srvs/srv/Trigger "{}"
```

#### `emergency_stop`

`Arm::emergency_stop()`。这是软件请求：权威仍是硬件急停；链路断开时固件自己的看门狗会持住机械臂。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/emergency_stop std_srvs/srv/Trigger "{}"
```

#### `park`

`Arm::park()`：声明全刚度静态持位，不依赖 100 ms 命令看门狗。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常。 |

```bash
ros2 service call /litearm/park std_srvs/srv/Trigger "{}"
```

#### `zero_g`

`data=true` 调用 `Arm::zero_g_start(zero_g_keepalive_period_s)`；`data=false` 调用
`Arm::zero_g_stop(true)`，因此保活线程若曾中断会被如实报出而不是吞掉。保活线程由 SDK 持有。

**不要在零重力激活期间发送其它运动命令**：固件会拒绝，而且那一刻机械臂处于人力引导状态。
请先退出零重力。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `data`（请求） | `bool` | `true` 进入零重力，`false` 退出。 |
| `success`（响应） | `bool` | 被门禁或 SDK 拒绝时为 `false`。 |
| `message`（响应） | `string` | 退出时若保活曾失败，会带上原因。 |

```bash
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: true}"
ros2 service call /litearm/zero_g std_srvs/srv/SetBool "{data: false}"
```

### 状态

#### `get_status`

主动发 `GET_STATUS`（`Arm::get_status_now(timeout)`）并返回一条 `LitearmStatus`。被动状态流静默、
又需要一个带超时上限的"还活着"证据时用它。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `timeout`（请求） | `float64` | 等待应答的秒数；`<= 0` 表示立刻返回缓存帧。 |
| `status`（响应） | `litearm_msgs/LitearmStatus` | 快照，字段与话题一致。 |
| `success`（响应） | `bool` | 未连接或固件未应答时为 `false`。 |

```bash
ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"
```

### 运动配置

#### `set_speed_scaling`

`Arm::set_speed(percent)`：固件的全局速度倍率。它是全局且持续的，与 SDK `movej` 的单条轨迹
`speed` 参数不是一回事。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `percent`（请求） | `uint8` | 0..100。这是整数百分比，`1` 表示百分之一速，不是全速。 |
| 门禁 | | 越界值在构造任何帧之前就被拒绝。 |

```bash
ros2 service call /litearm/set_speed_scaling litearm_msgs/srv/SetSpeedScaling "{percent: 40}"
```

#### `set_motion_mode`

`Arm::set_motion_mode(mode)`。当前固件只识别 mode 0，也就是 `park()`；其他值 SDK 会直接报错，
而不是 ACK 一个并未发生的模式切换。响应 message 里也写明了这一点。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `mode`（请求） | `int32` | 模式号。目前只接受 0。 |

```bash
ros2 service call /litearm/set_motion_mode litearm_msgs/srv/SetMotionMode "{mode: 0}"
```

#### `set_payload`

`Arm::set_payload(mass, com)`：声明固件需要补偿的载荷。固件会把质量静默钳进 `[0, 20]` kg，
因此节点在越界时直接拒绝，而不是为一个与请求不同的数字报告成功。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `mass`（请求） | `float64` | 千克，取值 `[0, 20]`。 |
| `com`（请求） | `float64[3]` | 质心，米，法兰坐标系，三个分量都必须有限。 |
| 门禁 | | 范围与有限性在这里检查；SDK 与固件也会在链路上执行同一条规则。 |

用 `get_feedforward_scalar`（`item: 4, sub: 0`）回读，拿到的是钳后的真值。

```bash
ros2 service call /litearm/set_payload litearm_msgs/srv/SetPayload \
  "{mass: 0.8, com: [0.0, 0.0, 0.05]}"
```

### 动力学与前馈

固件会静默钳制前馈数值，因此"写入成功"只代表钳后的值被存下，不代表你要的值被存下。
**写完请回读确认。**

#### `set_feedforward_mask`

`Arm::set_ff_mask(mask)`：前馈使能掩码。`FF_ALL` 之外的位由 SDK 拒绝，而不是被折成"全关"——
那等于静默关掉重力补偿。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `mask`（请求） | `uint32` | 只接受 `FF_ALL` 范围内的位。 |

```bash
ros2 service call /litearm/set_feedforward_mask litearm_msgs/srv/SetFeedforwardMask "{mask: 511}"
```

#### `set_feedforward_preset`

`Arm::ff_preset(preset)`。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `preset`（请求） | `int32` | 0 全关 / 1 出厂 / 2 全开。 |
| 门禁 | | 其它值在构造任何帧之前就被拒绝。 |

```bash
ros2 service call /litearm/set_feedforward_preset litearm_msgs/srv/SetFeedforwardPreset "{preset: 1}"
```

#### `set_feedforward_scalar`

`Arm::set_ff_scalar(item, sub, value)`。item 编号见 `litearm::Arm::ff_scalar_items()`；
`sub` 只对 item 5、6 有意义。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `item`（请求） | `int32` | 标量 item 号。 |
| `sub`（请求） | `int32` | 子索引；只对 item 5、6 有意义（0..2）。 |
| `value`（请求） | `float64` | 要写入的数值；固件会钳制。 |

```bash
ros2 service call /litearm/set_feedforward_scalar litearm_msgs/srv/SetFeedforwardScalar \
  "{item: 4, sub: 0, value: 0.8}"
```

#### `set_feedforward_vector`

`Arm::set_ff_vec(item, values)`。item 编号见 `litearm::Arm::ff_vec_items()`：item 1..5、7、8 为
7 个值；重力向量（item 6）为 3 个值。长度不对由 SDK 拒绝；空列表或非有限值在这里就被拒。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `item`（请求） | `int32` | 向量 item 号。 |
| `values`（请求） | `float64[]` | 非空且全部有限；长度必须与 item 匹配。 |

```bash
ros2 service call /litearm/set_feedforward_vector litearm_msgs/srv/SetFeedforwardVector \
  "{item: 7, values: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]}"
```

#### `get_feedforward_scalar`

`Arm::get_ff_scalar(item, sub)`：回读通路。item 9 是只读扩展，返回 `ff_mask`。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `item`（请求） | `int32` | 标量 item 号；9 读 `ff_mask`。 |
| `sub`（请求） | `int32` | 子索引；只对 item 5、6 有意义。 |
| `value`（响应） | `float64` | 固件里实际持有的值。 |

```bash
ros2 service call /litearm/get_feedforward_scalar litearm_msgs/srv/GetFeedforwardScalar \
  "{item: 9, sub: 0}"
```

### 关节参数

#### `get_joint_params`

`JointParams::get_joint_param(joint)` 或 `all_joint_params()`。读一个轴或整张表。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `joint`（请求） | `int32` | 轴索引；`-1` 表示全部轴。 |
| `params`（响应） | `litearm_msgs/JointParam[]` | 每个被请求的轴一条记录。 |
| 门禁 | | 超出固件上报轴数的索引会被拒绝。 |

每条 `JointParam` 携带 `index`、`kp`、`kd`、`tau_max`、`q_min`、`q_max`。

```bash
ros2 service call /litearm/get_joint_params litearm_msgs/srv/GetJointParams "{joint: -1}"
```

#### `set_joint_gains`

`JointParams::set_joint_param(joint, kp, kd, tau_max)`。这三个是固件自身位置环的增益；
ros2_control 组件自己不发任何增益。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `joint`（请求） | `int32` | 轴索引。 |
| `kp`、`kd`、`tau_max`（请求） | `float64` | 该轴的位置增益、阻尼增益与力矩上限。 |

```bash
ros2 service call /litearm/set_joint_gains litearm_msgs/srv/SetJointGains \
  "{joint: 0, kp: 12.0, kd: 1.5, tau_max: 9.0}"
```

#### `set_joint_limits`

`JointParams::set_joint_limits(joint, q_min, q_max)`。主机在连接时缓存限位，因此 SDK 的本地预检
要等下次 connect 才会用上新值，不是立即生效。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `joint`（请求） | `int32` | 轴索引。 |
| `q_min`、`q_max`（请求） | `float64` | 软限位，弧度。 |

```bash
ros2 service call /litearm/set_joint_limits litearm_msgs/srv/SetJointLimits \
  "{joint: 0, q_min: -1.0, q_max: 1.0}"
```

#### `reset_factory_params`

`JointParams::reset_factory()`。要求电机已失能。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常且电机已失能。 |

```bash
ros2 service call /litearm/reset_factory_params std_srvs/srv/Trigger "{}"
```

#### `save_params`

`Arm::save_params()`：把运行时参数写入 Flash。要求电机已失能。Flash 写入不可逆。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | 链路正常且电机已失能。 |

```bash
ros2 service call /litearm/save_params std_srvs/srv/Trigger "{}"
```

### 运动（`allow_motion`）

这一组服务的每一个都能给电机上电，因此除非 `allow_motion:=true`（与 `allow_dfu` 一样每次调用重新读取，
`ros2 param set` 即可生效），它们一律被拒。驱动独占串口，所以这些服务执行期间不会有别的进程在发命令。

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `move_j` | `litearm_msgs/srv/MoveJ` | 关节空间移动，固件逐轴 S 曲线。 |
| `move_j_sync` | `litearm_msgs/srv/MoveJSync` | 关节空间移动，各轴共用一条同步曲线。 |
| `move_p` | `litearm_msgs/srv/MoveP` | 位姿移动：固件自己解 IK 并走曲线。 |
| `move_js` | `litearm_msgs/srv/MoveJs` | 发一帧 MOVE_JS，即流式原语。 |
| `send_mit` | `litearm_msgs/srv/SendMit` | 单轴 MIT 透传。 |
| `send_mit_all` | `litearm_msgs/srv/SendMitAll` | 全臂 MIT 透传，一帧。 |
| `home` | `std_srvs/srv/Trigger` | 以固件低速走回 URDF 零位。 |
| `move_l` | `litearm_msgs/srv/MoveL` | 固件规划的直线笛卡尔运动。 |
| `move_c` | `litearm_msgs/srv/MoveC` | 经过途经点的圆弧笛卡尔运动。 |
| `move_path` | `litearm_msgs/srv/MovePath` | 多路点笛卡尔路径。 |

```bash
ros2 param set /litearm/driver allow_motion true
ros2 service call /litearm/move_j litearm_msgs/srv/MoveJ "{q: [0, 0, 0, 0, 0, 0, 0], speed: 0.2}"
ros2 service call /litearm/move_l litearm_msgs/srv/MoveL \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], speed: 0.2, wait: true}"
```

⚠ `move_js`、`send_mit`、`send_mit_all` 都是**单帧**：固件的 100 ms 命令看门狗会在没人持续重发时把臂
交回持位——持续重发是 `litearm_ros2_control` 做的事，这个驱动不做。`speed` 是满速的比例
（`0 < speed <= 1`）；百分比限速用 `set_speed_scaling`。

### 解算

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `inverse_kinematics` | `litearm_msgs/srv/InverseKinematics` | 把位姿解算成关节角，不产生运动（**不受** `allow_motion` 限制）。 |
| `poll_cart` | `litearm_msgs/srv/PollCart` | 查询 `wait=false` 的笛卡尔请求：`pending` 为真表示固件还没给出结论。 |

```bash
ros2 service call /litearm/inverse_kinematics litearm_msgs/srv/InverseKinematics \
  "{pose: [0.3, 0.0, 0.4, 3.14, 0.0, 0.0], seed: [], timeout: 0.5}"
```

### 前馈回读与重力变量

固件会静默钳位，所以写入是否生效要靠回读确认。`get_feedforward_catalog` 直接答 SDK 的静态表，
完全不需要链路。

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `get_feedforward_vector` | `litearm_msgs/srv/GetFeedforwardVector` | 回读一条前馈向量。 |
| `get_feedforward_mask` | `litearm_msgs/srv/GetFeedforwardMask` | 回读前馈使能掩码。 |
| `get_feedforward_catalog` | `litearm_msgs/srv/GetFeedforwardCatalog` | 列出 SDK 的条目表，不需要链路。 |
| `set_gravity_scale` | `litearm_msgs/srv/SetGravityScale` | 按轴缩放重力前馈（item 7，7 个值）。 |
| `set_inertia_scale` | `litearm_msgs/srv/SetInertiaScale` | 按轴缩放惯量前馈（item 8，7 个值）。 |
| `set_gravity_vector` | `litearm_msgs/srv/SetGravityVector` | 写模型使用的重力向量（item 6 的三个标量）。 |

```bash
ros2 service call /litearm/get_feedforward_catalog litearm_msgs/srv/GetFeedforwardCatalog "{}"
ros2 service call /litearm/get_feedforward_mask litearm_msgs/srv/GetFeedforwardMask "{}"
```

### 动力学模型存储

固件支持通过链路导入动力学模型（`probe_model` 回答这个固件是否支持）。写入是**暂存在 RAM**：
`commit_model` 固化到 flash 且要求电机已失能，`revert_model` 丢弃暂存。

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `probe_model` | `litearm_msgs/srv/ProbeModel` | 探测固件是否提供动力学模型存储。 |
| `get_model_body` / `set_model_body` | `litearm_msgs/srv/GetModelBody` / `SetModelBody` | 读 / 暂存单个刚体。 |
| `get_model_jm` / `set_model_jm` | `litearm_msgs/srv/GetModelJm` / `SetModelJm` | 读 / 暂存关节空间模型项。 |
| `commit_model` | `litearm_msgs/srv/CommitModel` | 固化暂存（`expected_mask` 必须与固件暂存一致）。 |
| `revert_model` | `std_srvs/srv/Trigger` | 丢弃暂存。 |
| `get_model_status` | `litearm_msgs/srv/GetModelStatus` | 覆写级别、暂存掩码与 dirty 标志。 |

```bash
ros2 service call /litearm/probe_model litearm_msgs/srv/ProbeModel "{}"
ros2 service call /litearm/get_model_status litearm_msgs/srv/GetModelStatus "{}"
```

### 诊断与链路

`get_diagnostics` 用来区分「链路安静」与「两端对帧格式的理解已经不一致」：`dropped` 是没人要的帧，
`bad_status_frames` 是 CRC 正确但本解码器读不懂的帧。

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `reconnect` | `std_srvs/srv/Trigger` | 拔插线缆后重建会话，并重读授权记录。 |
| `get_tcp` | `litearm_msgs/srv/GetTcp` | 读取固件当前的工具位姿。 |
| `get_diagnostics` | `litearm_msgs/srv/GetDiagnostics` | 主机计数器、各 id 报文速率、链路与能力标志。 |
| `kin_bench` | `litearm_msgs/srv/KinBench` | 跑固件的运动学基准；计数器里的 0 表示「没上报」。 |

```bash
ros2 service call /litearm/get_diagnostics litearm_msgs/srv/GetDiagnostics "{}"
ros2 service call /litearm/kin_bench litearm_msgs/srv/KinBench "{timeout: 8.0}"
```

### 控制拍日志

固件会记录自己的 300 Hz 控制拍（每拍 `tick`、每轴 `q_ref`/`dq`/`tau`）——控制环的问题靠它事后复盘。

| 服务 | 类型 | 作用 |
| --- | --- | --- |
| `log_start` | `litearm_msgs/srv/LogStart` | 开始录制指定拍数。 |
| `log_stop` | `std_srvs/srv/Trigger` | 停止录制。 |
| `log_dump` | `litearm_msgs/srv/LogDump` | 把录制读回并写进 `log_dir`（`filename` 只能是文件名）。 |

```bash
ros2 service call /litearm/log_start litearm_msgs/srv/LogStart "{ticks: 600}"
ros2 service call /litearm/log_dump litearm_msgs/srv/LogDump \
  "{filename: tick_log.bin, wait: true, timeout: 3.0}"
```

### 维护

#### `enter_dfu`

`Arm::enter_dfu()`：让设备登记进入 ROM bootloader。ACK 之后 CDC 设备会消失，本节点在重新用新链路
configure 之前不可用；设备会重新枚举成 `0483:DF11`，必须烧入固件才能回来。

**不要**为了"试一下"调用 `enter_dfu`。

| 项目 | 值 |
| --- | --- |
| 类型 | `std_srvs/srv/Trigger` |
| 请求 | 无 |
| 响应 | `success`、`message` |
| 前置条件 | `allow_dfu:=true`、电机已失能、ROM 向量表有效。 |

```bash
ros2 param set /litearm/driver allow_dfu true
ros2 service call /litearm/enter_dfu std_srvs/srv/Trigger "{}"
```

### 拒绝规则

每条门禁都在构造任何帧之前拒绝，因此被拒的调用不会在链路上留下任何字节。测试
`RefusedArgumentsSendNoFrame` 覆盖了这一点。

| 拒绝信息 | 触发条件 | 为什么在这里拒绝而不是转发 |
| --- | --- | --- |
| `percent must be 0..100` | `set_speed_scaling` 超出范围 | 固件收的是整数百分比：`1` 是百分之一速，不是全速。 |
| `preset must be 0 ... 2` | `set_feedforward_preset` 超出范围 | 只有三档预设。 |
| `period must be in [0.005, 0.10)` | `zero_g` 遇到越界的保活周期 | 固件命令看门狗在 0.10 s 触发，更慢的保活会静默掉出零重力。 |
| `mass must be in [0, 20] kg` | `set_payload` 超出范围 | 固件会静默钳制；存进去的值将与请求值不同。 |
| `requires the motors to be disabled` | 使能状态下调用 `save_params`、`reset_factory_params`、`commit_model` | 写 Flash 只有在没有力矩权限时才是安全的。 |
| `set the parameter ... to true` | `allow_dfu` 关着时调用 `enter_dfu`；`allow_motion` 关着时调用任何运动服务 | 一个不可逆，一个会给机械臂上电；默认都关。 |
| `not connected` | 任何在链路不可用时调用的服务 | 点出常见原因：USB 线、24 V 供电，或控制栈占着串口。 |
| `axis index must be 0..N-1` | 关节服务传入了固件未上报的轴号 | 否则一个笔误就会写到另一个轴上。 |

这些预检是尽力而为：`save_params` 靠缓存的状态帧判断电机是否使能。SDK 与固件会在链路上执行同一条
规则，所以缓存过期最多把一次干净的拒绝变成固件自己的错误消息，绝不会变成一次不安全的写入。

### 未暴露的能力

笛卡尔伺服与遥操作在这里没有对应服务：伺服需要命令通路上有控制器，而不是一个每次只交接一个
位姿的服务，那部分在 MoveIt Servo 与 `litearm_servo_control`。轨迹执行、固件笛卡尔规划、模型存储
与控制拍日志都已经暴露（能驱动机械臂的都在 `allow_motion` 之后）。持有权规则、错误语义与兼容性表
写在 [`docs/command-set.zh-CN.md`](../docs/command-set.zh-CN.md) 里。

## 测试

测试套件完全离线：它注入 SDK 的 `FakeTransport`，不接机械臂就能跑完整个生命周期与每个服务处理器：

```bash
colcon test --packages-select litearm_driver --event-handlers console_direct+
colcon test-result --verbose
```

## 许可证

Apache-2.0，见 [LICENSE](../LICENSE)。
