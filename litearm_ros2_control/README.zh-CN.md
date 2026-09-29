# litearm 的 ros2_control 适配层

让 [litearm-stm32](../../litearm-stm32) 固件驱动的 LiteArm 七轴机械臂，能被 ROS 2 的
`controller_manager` / `joint_trajectory_controller` / MoveIt2 直接控制。

运动学本体随工作区自带（`src/litearm`，上游快照自 `robot_description/` 仓库），
本工作区只做"适配"。

## 架构

```
┌─ ros2_control_node (C++ 实时环, 250Hz) ─────────────────────────┐
│  controller_manager                                             │
│   ├─ joint_state_broadcaster          → /joint_states           │
│   ├─ joint_trajectory_controller      ← FollowJointTrajectory   │
│   └─ LitearmSystem (hardware_interface::SystemInterface)        │
│        read()  ← seqlock 读状态块   write() → seqlock 写命令块  │
└─────────────────────────┬───────────────────────────────────────┘
                          │ POSIX 共享内存（seqlock 双缓冲，无锁）
┌─────────────────────────┴───────────────────────────────────────┐
│  litearm_hw_daemon (Python, 非实时, 250Hz)                      │
│   独占 USB CDC + litearm-stm32 固件协议                          │
│   读命令块 → MOVE_JS(q_ref, dq_ref) → 固件（PD + 内置前馈）      │
│   固件 100Hz 状态帧 → 写状态块                                   │
│   命令陈旧/故障/过温/急停 → 自动持续下发冻结参考（持位）          │
└─────────────────────────────────────────────────────────────────┘
                          │ USB CDC /dev/ttyACM0
┌─────────────────────────┴───────────────────────────────────────┐
│  litearm-stm32 固件 (STM32H723, 300Hz 控制环)                    │
│   · 独占 FDCAN1（1Mbps），直驱 7 台达妙电机                       │
│   · PD + 重力/摩擦/积分/kd_extra 前馈（模型由 URDF 生成、编译进去）│
│   · 安全包络：位置越限/超速/过温/跟随误差/反馈陈旧                 │
│   · 命令看门狗 100ms → fail-soft 持位；到位交接（升刚 + 重力前馈）│
└─────────────────────────────────────────────────────────────────┘
```

**为什么这样切分**

1. `read()` / `write()` 是纯 memcpy，实时环内没有 Python、没有 GIL、没有串口
   阻塞，也没有锁。
2. USB CDC 由单一进程独占。
3. **ROS 侧进程崩溃或重启期间，守护进程继续持位** —— 命令帧陈旧就转入 HOLDING
   并持续下发冻结参考，固件用正常刚度 + 重力前馈把臂持住，不会下坠。

### 与旧后端（pylitearm）的分工差异

换底层之前，PD、前馈、看门狗、安全包络、关节限位全在 PC 端的 Python 里算。
现在这些**全部下移到固件**：

| 职责 | 旧（pylitearm 后端） | 新（litearm-stm32） |
|---|---|---|
| 达妙 MIT 帧打包/收发 | 本层（SocketCAN） | **固件** |
| PD 增益 kp/kd | 逐帧由命令帧给 | **固件参数表** |
| 前馈 G/摩擦/积分/kd_extra | 本层（pinocchio 模型） | **固件 ff_mask** |
| 命令看门狗 / 失败软持位 | 本层 + pylitearm 看门狗 | **固件 100ms** |
| 安全包络（限位/超速/温度/跟随） | 本层逐周期裁决 | **固件 safety_check** |
| 关节限位 / tau_max / 零点 | pylitearm litearm.yaml | **固件参数表** |
| ROS 命令流节拍 / SHM 契约 | 本层 | 本层（不变） |

**唯一的实质功能回退**：默认通道没有 `M·q̈` 与 `C·q̇`。原因不是省略，是
**MOVE_JS 没有加速度源**——固件源码注释原文：「无加速度源(梯形限幅, 非 S 曲线):
M·ddq 不猜」，而惯量项只在 `MOVE_J`（自带 S 曲线）里算。旧后端用 JTC 的
`acceleration` 命令接口提供 `q̈`，那条路在默认通道上不存在。

## 包布局

| 包 | 内容 |
|---|---|
| `litearm` | 位于 `src/litearm`（工作区自带，上游快照自 `robot_description/` 仓库），提供 URDF 与网格 |
| `litearm_ros2_control` | 共享内存契约（C++/Python 双侧）、`LitearmSystem` 插件、硬件守护进程、控制器配置、总 launch |
| `litearm_moveit_config` | SRDF、KDL 运动学、关节限位、OMPL 配置、move_group launch、碰撞矩阵生成工具 |

## 构建与运行

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**⚠️ 先设 ROS 域，否则跨机串扰。** ROS 2 默认域 0，且组播发现是全网段的。
同网段若有其他机器人/实验在跑 ROS 2，你的 `/move_action` 会看到**多个 server**，
规划目标可能被陌生节点的 move_group 接管，表现为"报 FAILURE 但臂确实动了"
或"轨迹发出去却不执行"这类极难定位的现象。本工作区的验收脚本强制：

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
```

建议长期写进你的环境脚本。

### 无硬件演练（推荐先跑通这个）

```bash
# 控制栈：守护进程在 pty 上起一块**假固件**，链路仍走真实 USB CDC 协议
ros2 launch litearm_ros2_control litearm_control.launch.py dry_run:=true

# MoveIt：规划 + 执行全链路
ros2 launch litearm_moveit_config litearm_moveit.launch.py dry_run:=true
```

一键验收（起栈 → 探针 → 收栈，返回码即结论）：

```bash
ros2_ws/src/litearm_ros2_control/scripts/acceptance_control.sh
ros2_ws/src/litearm_moveit_config/scripts/acceptance_moveit.sh --execute
```

**另有一条只测协议与硬件的冒烟脚本**（不依赖 ROS，可对着真板子跑）：

```bash
ros2_ws/src/litearm_ros2_control/scripts/stm32_smoke.py --fake        # 无硬件，九段全跑
ros2_ws/src/litearm_ros2_control/scripts/stm32_smoke.py --read-only   # 真机零副作用预检
ros2_ws/src/litearm_ros2_control/scripts/stm32_smoke.py               # 真机：ENABLE 但不动
ros2_ws/src/litearm_ros2_control/scripts/stm32_smoke.py --move        # 再允许动 0.03 rad
```

### 真机

```bash
# 1) 板子已上电、USB 已连（自动发现 VID:PID 1d50:606f）
# 2) **license 已激活** —— 未激活时固件拒绝 ENABLE（ERR{0x10,0x08}），
#    所有运动命令都会被间接挡住。用 litearm-stm32 的 tools/litearm-license。
# 3) RT 环境（可选但推荐）
sudo ./rt_env.sh rt-status       # 确认 RT 内核 + realtime 组

export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
ros2 launch litearm_ros2_control litearm_control.launch.py          # 或 litearm_moveit
```

上电前请确认：机械臂已被支撑或处于安全姿态、急停可触达。

守护进程退出时先按 `--exit-hold-s`（默认 2.0s）持续下发持位参考（这段时间固件
仍叠重力前馈，臂稳稳不动），再发 `0x20 park` 声明高刚度持位。所以 Ctrl-C 掉
launch 不会让臂掉下来，但 **`park` 之后的持位不含重力前馈**（固件 `hold` 分支
`tau=0`），臂会按 `G/kp` 轻微下垂——要彻底不掉只能保持栈运行或断电前支撑。
期间再按一次 Ctrl-C 可立刻结束。

## 接口语义

命令接口（每个关节）：`position` / `velocity` / `acceleration` / `effort` / `kp` / `kd`

**默认通道只消费前两个**，把它们送到固件的 `(q_ref, dq_ref)`；固件内部执行

```
tau = kp · (q_ref − q) + kd · (dq_ref − dq) + G(q) + 摩擦 + ki·∫e + kd_extra·Δdq
```

其中 `kp/kd` 取自**固件参数表**（守护进程启动时用 `0x24` 读回）。

### ⚠️ `velocity` 是位置参考的 slew 速率上限（真机核实）

固件（`control_loop.c` 的 `ARM_MODE_MOVE_JS` 分支）：

```c
v_lim = clampf(fabsf_(target_dq[i]), 0.0f, jp->speed_limit * gov_ratio);
cmd->q_ref = slew_linear(target_q, cmd->q_ref, v_lim * LITEARM_CTRL_DT);
```

即**参考每拍最多朝命令目标走 `|dq_ref|·dt`**。三条直接后果：

1. **`velocity=0` 时关节一步都不会动** —— "只发位置不给速度"在默认通道上等于
   "停住"，不是"位置伺服"。所以驱动方必须同时给位置与速度（JTC 两者都给，
   没问题；但手工发 `Float64MultiArray` 时容易只想到位置）。
2. **轨迹结束时 velocity 归零 → 参考冻结**。如果运动过程中参考落后于 JTC 的
   参考，那份滞后会**永久留在原地**（没有"追赶"能力，因为追赶速度上限就是 0）。
   规划侧应保证末点速度平滑归零；实测这条的影响量级见手册。
3. **HOLDING 的"锚定实测位置"在默认通道下是信息性的**：`dq=0` 时固件冻结的是
   它自己的参考，臂会从实测位置收敛到固件参考（位移 ≈ 进入持位那一刻的跟踪
   滞后，真机 ~0.01 rad）。这是**期望行为**（停在命令它去的地方）。只有在
   `--mit-passthrough` 下，参考才真的 slew 到我们给的 `q_hold`。

| 命令接口 | 默认通道（MOVE_JS） | `--mit-passthrough`（MIT_ALL） |
|---|---|---|
| `position` | ✅ `q_ref` | ✅ `q_ref` |
| `velocity` | ✅ `dq_ref` | ✅ `dq_ref` |
| `effort` | ❌ 不转发 | ✅ `tau_ff`（原样透传） |
| `kp` / `kd` | ❌ 取固件参数表 | ✅ 逐帧 MIT 增益 |
| `acceleration` | ❌ 不转发 | ❌ 固件无加速度通道 |

未被子控制器 claim 的接口是惰性的，所以"多导出"不产生副作用；保留它们是为了
让回退通道可用。**默认栈只 claim `position` + `velocity`**（见
`config/litearm_controllers.yaml`）。

⚠️ 默认通道**刻意不带 `tau_ff`**：固件的 `builtin_mode` 要求
`MOVE_JS && !s_js_user_ff`，载荷里一旦出现 tau（哪怕全 0）就把整套内置前馈
（重力/摩擦/积分/kd_extra/量化补偿）**整段关掉**。所以"顺手传个全 0 的 tau
省事"会让臂失去重力补偿——这是个安静且危险的失效，
`test_move_js_channel_never_carries_tau_ff` 把它钉死。

### 回退通道：`--mit-passthrough`

切到 MIT_ALL 全透传：`kp/kd/effort` 逐帧生效、**固件不叠任何自家前馈**，
前馈由 ROS 侧自己提供。用途是 A/B 对照与"我要自己算"。此时
`litearm_kp_controller` / `litearm_kd_controller` 才有意义（默认通道下它们
是纯惰性的）。

### 参数从哪来

| 参数 | 真源 |
|---|---|
| kp / kd / tau_max / 软限位 / ff_mask / kd_extra / hold_kp_gain | **固件参数表**（守护进程启动时 `0x24`/`0x2B`/`0x2C` 读回并打进启动日志） |
| 端口 / 频率 / 超时 / 通道 / 退出持位 | 命令行 > `config/litearm_hw.yaml` > 代码默认值 |

`config/litearm_hw.yaml` **只放 PC 侧概念**；关节级参数一概不放（放进来就是
第二份真相，改了没反应且很难查）。该文件未知键会直接报错，不静默忽略。

想让守护进程显式覆盖固件前馈，用五个三态开关（不传 = 不碰固件，
`--no-` = 清位）：

| 开关 | 对应 |
|---|---|
| `--gravity-compensation` | `FF_G` |
| `--friction-compensation` | `FF_FRICTION` |
| `--inertia-compensation` | `FF_INERTIA \| FF_CORIOLIS`（⚠ MOVE_JS 下无效） |
| `--integral-compensation` | `FF_INTEGRAL` |
| `--damping-compensation` | `kd_extra` 向量（无独立 FF 位） |

标准状态接口 `position` / `velocity` / `effort`，另导出诊断量
`temperature_mos` / `temperature_coil` / `error_code` / `feedback_age`
（不需要可用 `litearm_export_diagnostics:=false` 关掉）。

⚠️ `feedback_age` 是**整帧状态帧的龄**，不是逐关节的：固件只给一个全局
`FB_STALE` 标志，没有逐关节龄字段。逐关节判据请用 `error_code` 与 flags。

## 安全模型

**检测在固件**（`safety_check` 五类：位置越限 / 超速 / 过温 / 跟随误差 /
反馈陈旧），守护进程只**翻译**成 ROS 侧看得懂的原因码，写进状态块：

| 优先级 | 条件 | 行为 |
|---|---|---|
| 1 | 状态帧陈旧（> `--feedback-timeout-s`） | HOLDING（链路断了） |
| 2 | 命令帧陈旧（> `--command-timeout-s`） | HOLDING 持续下发冻结参考 |
| 3 | `enable=0`（**仅命令帧新鲜时认**） | 失能电机（臂会失力） |
| 4 | `estop≠0` | HOLDING（**不发固件急停**） |
| 5 | 固件 `FAULT` / `joint_fault` | HOLDING |
| 6 | 固件 `FB_STALE` | HOLDING |
| 7 | 固件 `TEMP_WARN` | HOLDING |
| 8 | 命令含 NaN/Inf | 拒绝本帧，保持持位 |
| 9 | 固件 `WD_TRIPPED` | **仍发帧**（发帧即 kick），仅上报 |
| 10 | 正常 | TRACKING |

三个刻意的设计：

* **第 3 条的顺序**：先判陈旧再判 `enable`。反过来时，"ROS 侧挂掉时最后一帧
  恰好是 enable=0"会让臂直接掉下来。`test_stale_enable_zero_is_ignored` 锁住它。
* **第 4 条不发固件的 `0x12` 急停**：固件急停会失能电机（臂掉下来），而软急停
  的既有语义是**保持高刚度持位**。真急停请用硬件急停。
* **第 9 条不停发**：停发会让"看门狗接管"变成自锁——固件等不到 kick 就一直
  处在 fail-soft，而守护进程也一直不敢发。发帧本身就是 kick，标志会自己清掉。

持位不用"停发等固件看门狗"实现（那是 0.6× 刚度且 `tau=0`，负载下会下垂），
而是**持续下发冻结参考**：固件用正常刚度 + `G(q_hold)` 把臂持住。

## 已知限制

* **默认通道没有 `M·q̈` 与 `C·q̇`**（见上文"分工差异"）。需要它就走
  `--mit-passthrough` 自己算。
* **kp/kd 不能逐帧给**（默认通道）：要调刚度请改固件参数（`0x22` / USB 工具），
  或切 `--mit-passthrough`。
* **`gazebo_ros2_control` 未安装**，本工作区不提供 Gazebo 仿真。无硬件验证走
  `--dry-run`（pty 假固件，**一阶运动学模型，不含动力学**，只能验证接口与
  数据通路，不能用来整定增益）。
* **KDL 是唯一的 IK 求解器**（本机无 ikfast/pick_ik/trac_ik），7 轴冗余臂上
  笛卡尔规划成功率一般。若失败率偏高，优先考虑加装 IK 插件而不是加大 timeout。
* **碰撞矩阵是点云近似**（`tools/compute_collision_matrix.py`，150 位形采样、
  每 link 1.2 万抽稀顶点）。改 URDF 或换网格后必须重新生成。
* `effort` 命令接口在 Humble 的 JTC 里**必须单独使用**
  （`command_interfaces: [effort]`），不能与 `position` 组合 —— 这是 JTC 自身的
  约束，已在配置注释里说明。
* 诊断状态接口（temperature 等）不是标准三件套。本环境下 `joint_state_broadcaster`
  会把它们一并 claim 并发布；换 ROS 发行版时若出现"接口无人认领"的告警属预期。
* **固件参数掉电丢失**（除 `0x25` 存 Flash）：`0x22/0x23/0x26/0x27/0x28` 只写
  RAM。守护进程每次启动都重读，所以不影响本层；手工改过参数想持久化要显式存 Flash
  （且须先失能）。
