#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hw_daemon.py — litearm 硬件守护进程（litearm-stm32 后端）。

职责与边界
----------
本进程是 **USB CDC 的唯一持有者**：它把 litearm-stm32 固件的 MOVE_JS / MIT_ALL
命令流，以共享内存双缓冲的形式暴露给 ros2_control 的
``hardware_interface::SystemInterface`` 插件。

    ros2_control 进程 (C++, 实时环)          本进程 (Python, 非实时)
    ────────────────────────────────         ─────────────────────────
    read()  ← seqlock 读状态块          ←──   状态帧解析 (100Hz 主动上报)
    write() → seqlock 写命令块          ──→   MOVE_JS(0x03) / MIT_ALL(0x05) @100Hz（见 transport.rate_hz）

这样切分的原因：

1. ros2_control 的 ``read()``/``write()`` 是纯 memcpy（无锁、不阻塞），
   实时环不含 Python 解释器/GIL/GC，也不会被串口写超时拖住。
2. USB CDC 归本进程独占。
3. **ROS 侧进程崩溃/重启期间，本进程继续持位** —— 命令陈旧就转入 HOLDING
   并持续下发冻结参考，臂不会因为上位机挂掉而下坠。

与 pylitearm 后端的分工差异（换底层的核心）
-------------------------------------------
STM32 固件自带电机控制环（300Hz）、内置前馈模型（由 URDF 生成）、安全包络与
命令看门狗。**这些职责原先都在本文件里用 Python 实现，现在全部下移**：

    ┌ 职责 ────────────────────────┬ 旧（pylitearm 后端）──────┬ 新（litearm-stm32）────┐
    │ 达妙 MIT 帧打包/收发          │ 本进程 (SocketCAN)         │ **固件**              │
    │ PD 增益 kp/kd                 │ 逐帧由命令帧给             │ **固件参数表**        │
    │ 前馈 G / 摩擦 / 积分 / kd_extra│ 本进程 (pinocchio 模型)    │ **固件 ff_mask** §1   │
    │ 命令看门狗 / 失败软持位        │ 本进程 + pylitearm 看门狗  │ **固件 100ms**        │
    │ 安全包络（限位/超速/温度/跟随）│ 本进程逐周期裁决           │ **固件 safety_check** │
    │ 关节限位 / tau_max / 零点      │ pylitearm litearm.yaml     │ **固件参数表** §2     │
    │ ROS 命令流节拍 / SHM 契约      │ 本进程                     │ 本进程（不变）        │
    └───────────────────────────────┴────────────────────────────┴───────────────────────┘

§1 决定前馈开关的不是"模式"而是**这一帧带不带 tau_ff**：固件的 ``builtin_mode``
   要求 ``MOVE_JS && !s_js_user_ff``。所以默认通道发**不带 tau_ff** 的 MOVE_JS
   （56B 载荷），让固件叠自己的前馈；一旦带上 tau（哪怕全 0）就整段关掉。

§1b **``MOVE_JS`` 的 ``dq_ref`` 不只是速度前馈，它同时是位置参考的 slew 速率上限**
   （真机核实 ``control_loop.c``）::

       v_lim = clampf(fabsf_(target_dq[i]), 0.0f, jp->speed_limit * gov_ratio);
       cmd->q_ref = slew_linear(target_q, cmd->q_ref, v_lim * LITEARM_CTRL_DT);

   即参考每拍最多朝命令目标走 ``|dq_ref|·dt``。两个直接后果：

   * **``dq_ref=0`` 时参考冻结，关节一步都不会动**。"只发位置、速度给 0"在默认
     通道上不是"位置伺服"，是"停住"。所以驱动必须同时给位置与速度（JTC 两者都给）。
   * HOLDING 下发的 ``q_hold`` 在**位置项上被忽略** —— 固件冻结的是它自己的参考。
     臂于是从"实测位置"收敛到"固件参考"，位移 = 进入持位那一刻的跟踪滞后。
     这是**正确的行为**（停在命令它去的地方，而不是停在它恰好在的地方），但
     ``self._q_hold`` 在默认通道下是**信息性的**（用于日志与退出持位流），不是
     直接可执行的锚点；只有 ``--mit-passthrough``（MIT_ALL，参考按 ``vel_max``
     slew）下它才真正决定位置。
§2 关节级参数**全部在启动时从固件读**（``0x24`` / ``0x2C``），本进程不再持有
   第二份真相。想改参数改固件（``0x22``/``0x26``/``0x27``/``0x28``），或看 §「前馈覆盖」。

双通道
------
默认 **位置模式**（MOVE_JS，不带 tau_ff）：PD + G + 摩擦 + 积分 + kd_extra +
量化补偿全由固件算。代价是 MOVE_JS 没有加速度源（``ddq_s ≡ 0``），所以
**没有 M·q̈ 与 C·q̇** —— 这是与 pylitearm 后端最实质的差异。

``--mit-passthrough`` 切换到 **力矩模式**（MIT_ALL 全透传）：kp/kd/effort 逐帧
由命令帧给，固件不叠任何自家前馈。**本进程不做任何动力学计算**——``effort``
原样透传，前馈由 ROS 侧或使用者提供。这条通道用于 A/B 对照与"我要自己算"的场景。

前馈覆盖（``--ff-*``）
----------------------
五个开关**默认一个都不接**（= 完全沿用固件现状，读回并记日志）。
显式给了才下发，把对应位写进固件 ``ff_mask``：

    --gravity-compensation   → FF_G
    --friction-compensation  → FF_FRICTION
    --inertia-compensation   → FF_INERTIA | FF_CORIOLIS
    --integral-compensation  → FF_INTEGRAL
    --damping-compensation   → 0x26 item15（kd_extra 向量，无独立 FF 位，0 = 关）

⚠️ MOVE_JS 模式下 ``FF_INERTIA``/``FF_CORIOLIS`` 位**不起作用**（固件只在
MOVE_J 里算惯量项）——打开它们不会有害，但也不会有用。

安全裁决（本进程保留骨架，判据改由固件提供）
--------------------------------------------
每周期按固定优先级裁决，抑制原因写入状态块的 ``last_error``：

    优先级  条件                                    行为
    ──────  ──────────────────────────────────────  ────────────────────────────
    1       状态帧陈旧（> --feedback-timeout-s）      HOLDING（未连接）
    2       命令帧陈旧（> --command-timeout-s）       HOLDING（冻结参考持续下发）
    3       enable=0（**仅命令帧新鲜时认**）          失能电机（臂会失力）
    4       estop≠0                                  HOLDING（**不发固件急停**）
    5       固件 FAULT / joint_fault                   HOLDING
    6       固件 FB_STALE / 状态帧过旧                  HOLDING
    7       固件 TEMP_WARN                             HOLDING
    8       命令含 NaN/Inf                            拒绝本帧，保持持位
    9       固件 WD_TRIPPED                            仍发帧（发帧即 kick），仅上报
    10      正常                                      TRACKING

第 3 条的**顺序是刻意的**：若先判 ``enable`` 再判陈旧，"ROS 侧挂掉时最后一帧
恰好是 enable=0"会让臂直接掉下来。``test_stale_enable_zero_is_ignored``
专门锁住这个分支。

第 4 条刻意**不映射成固件的 0x12 急停**：固件急停会失能电机（臂掉下来），
而 ROS 侧"软急停"的既有语义是**保持高刚度持位**。真急停请用硬件急停。

持位怎么实现
------------
HOLDING 下不发固件的失败软持位（那要停发 100ms 让看门狗超时，且 tau=0 无重力
前馈、刚度只有 0.6×），而是**持续下发冻结参考**：

* 位置模式：``MOVE_JS(q_hold, dq=0)`` → 固件 PD + **G(q_hold)** 持住，不下垂；
* 力矩模式：``MIT_ALL(q_hold, 0, kp×hold_kp_gain, kd, tau=0)``，增益取自固件
  （``0x2C`` item18 的 ``hold_kp_gain``），与固件自己的到位增刚同源。

退出
----
收到 SIGINT/SIGTERM 后先按 ``--exit-hold-s``（默认 2.0s）**持续下发冻结参考**
（这段时间固件仍叠重力前馈，臂稳稳不动），再发 ``0x20 park``（声明 park：
之后看门狗触发时用 1.0× 刚度而不是 0.6×）然后关闭链路。

⚠️ park 之后的持位**没有重力前馈**（固件 ``hold`` 分支 ``tau=0``），按
``G/kp`` 有轻微下垂。这是固件侧的既有行为，不是本进程引入的。要彻底不掉，
只能保持栈运行或断电前支撑。

再次 Ctrl-C 会立刻跳过剩余持位时间。
"""

import argparse
import errno
import fcntl
import logging
import math
import os
import signal
import sys
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

from litearm_ros2_control import fake_firmware
from litearm_ros2_control import stm32_proto as proto
from litearm_ros2_control import shm_bridge
from litearm_ros2_control.shm_bridge import (
    DAEMON_CONNECTING,
    DAEMON_DISABLED,
    DAEMON_HOLDING_BAD_COMMAND,
    DAEMON_HOLDING_ESTOP,
    DAEMON_HOLDING_FEEDBACK_STALE,
    DAEMON_HOLDING_MOTOR_FAULT,
    DAEMON_HOLDING_OVERTEMP,
    DAEMON_HOLDING_STALE_COMMAND,
    DAEMON_HOLDING_WATCHDOG,
    DAEMON_OK,
    DAEMON_SHUTTING_DOWN,
    NUM_JOINTS,
    LitearmCommand,
    LitearmState,
    SharedMemory,
    ShmError,
)
from litearm_ros2_control.stm32_link import (Stm32AccessDenied,
                                             Stm32Error,
                                             Stm32Link,
                                             Stm32NotConnected,
                                             rejected_text)

log = logging.getLogger("litearm.hw_daemon")

# 跟随模式（内部状态机，与旧后端同名以便对照阅读）。
MODE_TRACKING = "tracking"
MODE_HOLDING = "holding"
# "尚未进入任何模式"的哨兵：连接成功后的初值必须是它，这样第一个周期一定会执行
# ``_enter(HOLDING)`` 的锁位动作（把持位锚点锁到实测位置）。若把初值直接写成
# MODE_HOLDING，``_enter`` 开头的 ``if mode == self._mode`` 守卫会跳过锁位，
# ``_q_hold`` 保持 [0]*7，首个持位帧就会把机械臂高刚度拽回零位。
MODE_INIT = "init"
MODE_DISABLED = "disabled"

# 运动命令（固件四个运动收口 + 笛卡尔版本）。被拒的运动帧**不喂看门狗**
# （拒绝发生在 `ctrl_accept_*` 内部、`watchdog_kick()` 之前）⇒ 持续被拒会把固件
# 推进 fail-soft（`kp×0.6` / `τ=0` / 无重力前馈）⇒ 臂下坠/来回摇。故
# `_report_link_counters` 只对**这一类**拒帧报警，其余（如启动期首帧 ENABLE 必然
# 回的 `{0x10,0x03}`，那是正常路径）只记 INFO。
MOTION_CMDS = (proto.CMD_MOVE_J, proto.CMD_MOVE_P, proto.CMD_MOVE_JS,
               proto.CMD_MOVE_MIT, proto.CMD_MOVE_MIT_ALL,
               proto.CMD_MOVE_J_SYNC)

# 达妙 MIT 帧的硬性可表示范围（固件内部也按这个钳；超范围会被电调截断）。
MIT_KP_MIN, MIT_KP_MAX = 0.0, 500.0
MIT_KD_MIN, MIT_KD_MAX = 0.0, 5.0

# 固件 ENABLE 的两段语义：ACK 只表示"登记"，要等反馈齐了才真正加磁。
# 起因 0x03 是"反馈尚未就绪"，值得重试；0x08 是 license 未激活，重试无用。
ENABLE_REASON_NOT_READY = 0x03
ENABLE_REASON_EMERGENCY = 0x06
ENABLE_REASON_NO_LICENSE = 0x08

# 环内日志的最短间隔（秒）。控制环 100 Hz 下绝不允许"每周期一条日志"：launch 里
# 日志写终端/管道是阻塞 I/O，故障持续时恰好会把最需要时序的周期拖垮。策略：
# 首次出现立即输出，其后同一 key 最多每 LOG_THROTTLE_S 一次；抑制次数只在
# 退出时汇总一行（见 _shutdown），既不丢诊断也不进控制环的高频路径。
LOG_THROTTLE_S = 1.0

# 退出时 park 帧发出的等待窗口（秒）。见 _shutdown 的注释：USB CDC 有内部发送
# 缓冲，写完立刻关端口可能丢帧，而 park 声明丢了臂就多垂一截。
PARK_FLUSH_S = 0.05

# 五个 --ff-* 开关对应的固件 ff_mask 位。
FF_FLAG_TO_BITS = {
    "gravity": proto.FF_G,
    "friction": proto.FF_FRICTION,
    "inertia": proto.FF_INERTIA | proto.FF_CORIOLIS,
    "integral": proto.FF_INTEGRAL,
}

# kd_extra 的出厂向量（固件 params/defaults.c 的 joint[0..6].kd_extra）：
# 只有 J1~J4 四个承重轴带值。固件重启后会回到这个值，但一旦被 0x26 item15
# 清零就不在 RAM 里留痕了，所以"打开 --damping-compensation"时用它兜底。
KD_EXTRA_FACTORY = (6.0, 6.0, 6.0, 6.0, 0.0, 0.0, 0.0)


def _sleep_until(next_tick: float) -> float:
    """忙等到绝对时刻 ``next_tick``（粗睡 + 尾段 spin），返回实际结束时刻。

    绝对时基防漂移，尾段 spin 消除 ``sleep`` 精度不足。本进程不做轨迹积分，
    因此只影响下发帧的等间隔性。
    """
    while True:
        remain = next_tick - time.monotonic()
        if remain <= 0.0:
            break
        if remain > 0.0005:
            time.sleep(remain - 0.0003)
    return time.monotonic()


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


class DaemonFatalError(RuntimeError):
    """不可重试的启动失败（例如固件 license 未激活）——重试多少次都一样。"""


class _RetryableConnect(Exception):
    """可重试的连接失败（应答丢了、反馈还没就绪、板子刚拔插）。"""


class DaemonSettings:
    """守护进程运行参数。

    **只放 PC 侧概念**（端口、频率、超时、策略）。关节级参数（kp/kd/tau_max/
    限位/前馈开关）不在配置里，启动时从固件读——见模块 docstring §2。
    """

    def __init__(self, port: str = "",
                 rate_hz: float = 100.0,
                 shm_name: str = shm_bridge.DEFAULT_SHM_NAME,
                 dry_run: bool = False,
                 mit_passthrough: bool = False,
                 command_timeout_s: float = 0.1,
                 feedback_timeout_s: float = 0.08,
                 connect_retry_s: float = 2.0,
                 arm_timeout_s: float = 5.0,
                 exit_hold_s: float = 2.0,
                 request_timeout_s: float = 0.5,
                 ff_overrides: Optional[dict] = None,
                 verbose: bool = False) -> None:
        self.port = str(port).strip()
        self.shm_name = shm_name
        self.dry_run = bool(dry_run)
        self.mit_passthrough = bool(mit_passthrough)
        self.verbose = bool(verbose)

        self.rate_hz = float(rate_hz)
        if not math.isfinite(self.rate_hz) or self.rate_hz <= 0.0:
            raise ValueError("rate_hz 必须为正有限值")

        self.command_timeout_s = _positive("command_timeout_s",
                                           command_timeout_s)
        self.feedback_timeout_s = _positive("feedback_timeout_s",
                                            feedback_timeout_s)
        self.connect_retry_s = _positive("connect_retry_s", connect_retry_s)
        self.arm_timeout_s = _positive("arm_timeout_s", arm_timeout_s)
        self.exit_hold_s = float(exit_hold_s)
        if not math.isfinite(self.exit_hold_s) or self.exit_hold_s < 0.0:
            raise ValueError("exit_hold_s 必须是非负有限值")
        self.request_timeout_s = _positive("request_timeout_s",
                                           request_timeout_s)
        # 只保留显式给出的开关；缺省 = 不碰固件（见模块 docstring）。
        self.ff_overrides = {k: bool(v) for k, v in (ff_overrides or {}).items()
                             if v is not None}

        # ── 以下由 _connect() 从固件读回后填充 ──
        self.firmware_version = ""
        self.num_joints = NUM_JOINTS
        self.kp: List[float] = [0.0] * NUM_JOINTS
        self.kd: List[float] = [0.0] * NUM_JOINTS
        self.tau_max: List[float] = [0.0] * NUM_JOINTS
        self.q_min: List[float] = [-math.pi] * NUM_JOINTS
        self.q_max: List[float] = [math.pi] * NUM_JOINTS
        self.ff_mask = 0
        self.ff_damping: List[float] = [0.0] * NUM_JOINTS
        self.hold_kp_gain = 1.0

    def describe(self) -> str:
        channel = "MIT_ALL 全透传" if self.mit_passthrough else "MOVE_JS 位置模式"
        return (
            f"port={self.port or '(自动发现)'} shm={self.shm_name} "
            f"rate={self.rate_hz:g}Hz channel={channel} "
            f"dry_run={self.dry_run} cmd_timeout={self.command_timeout_s:g}s "
            f"fb_timeout={self.feedback_timeout_s:g}s "
            f"exit_hold={self.exit_hold_s:g}s"
        )

    def describe_firmware(self) -> str:
        """启动日志里的"硬件身份"块：动的是哪块板、哪一版固件、参数是多少。

        这一行的存在理由是**排障**：换了板子/刷了固件/改了参数之后，栈里必须
        一眼看出"守护进程实际在用什么"，而不是去猜。
        """
        params = "\n".join(
            f"  j{i + 1}: kp={self.kp[i]:g} kd={self.kd[i]:g} "
            f"tau_max={self.tau_max[i]:g} "
            f"q∈[{self.q_min[i]:.6f}, {self.q_max[i]:.6f}]"
            for i in range(self.num_joints))
        return (
            f"固件版本：{self.firmware_version}\n"
            f"  端口：{self.port or '(自动发现)'} · 关节数 {self.num_joints}\n"
            f"  通道：{'MIT_ALL 全透传（kp/kd/effort 逐帧生效）' if self.mit_passthrough else 'MOVE_JS（固件算 PD + 内置前馈）'}\n"
            f"  ff_mask=0x{self.ff_mask:03X}（{proto.format_ff_mask(self.ff_mask)}）"
            f" · kd_extra={self.ff_damping} · hold_kp_gain={self.hold_kp_gain:g}\n"
            f"{params}"
        )


def _positive(name: str, value) -> float:
    out = float(value)
    if not math.isfinite(out) or out <= 0.0:
        raise ValueError(f"{name} 必须是正有限值")
    return out


# ─────────────────── 硬件配置（PC 侧参数，可选） ───────────────────
# litearm_hw.yaml 只放**PC 侧概念**（端口、频率、超时、策略）。关节级参数
# （kp/kd/tau_max/限位/前馈开关）的唯一真源是固件参数表，放进来就是第二份真相。
#
# 键名用「点分扁平」形式，与 YAML 的嵌套一一对应：
#   transport.rate_hz  ←→  transport:\n  rate_hz:

HW_CONFIG_KEYS = {
    "transport.port": str,
    "transport.rate_hz": float,
    "transport.command_timeout_s": float,
    "transport.feedback_timeout_s": float,
    "transport.connect_retry_s": float,
    "transport.arm_timeout_s": float,
    "transport.request_timeout_s": float,
    "policy.mit_passthrough": bool,
    "policy.exit_hold_s": float,
    "policy.ff_overrides": dict,
}


def _flatten_config(raw: dict, prefix: str = "") -> List[Tuple[str, object]]:
    """把嵌套 YAML 压成点分扁平列表。

    是否下钻只看"这个键是不是某个已知键的前缀"——所以 ``transport`` 会下钻，
    而 ``policy.ff_overrides`` 本身就是叶子（没有以它开头的已知键）。
    拼错的键（比如 ``transport.por``）不会被下钻，于是会落到叶子位置，
    由 :func:`load_hw_config` 的未知键检查逮住。
    """
    out: List[Tuple[str, object]] = []
    for key, value in raw.items():
        name = f"{prefix}{key}"
        if any(known.startswith(f"{name}.") for known in HW_CONFIG_KEYS):
            if not isinstance(value, dict):
                raise ValueError(f"{name} 应该是一个映射，实际是 "
                                 f"{type(value).__name__}")
            out.extend(_flatten_config(value, f"{name}."))
            continue
        out.append((name, value))
    return out


def _check_config_type(path: str, key: str, value, expected: type):
    """类型校验（含 ``bool`` 与 ``int`` 的坑：``True`` 也是 ``int``）。"""
    # YAML 里写 250 而不是 250.0 是正常写法，靠 isinstance 会误判。
    if expected is float and isinstance(value, int) and not isinstance(value,
                                                                      bool):
        return float(value)
    if expected is not bool and isinstance(value, bool):
        raise ValueError(f"{path} 的 {key} 期望 {expected.__name__}，"
                         f"实际是 bool（{value!r}）")
    if not isinstance(value, expected):
        raise ValueError(f"{path} 的 {key} 期望 {expected.__name__}，"
                         f"实际是 {type(value).__name__}（{value!r}）")
    return value


def load_hw_config(path: str) -> dict:
    """读 ``litearm_hw.yaml``，返回点分扁平的字典。

    **未知键直接报错**，不静默忽略：配置文件里打错一个字母却沿用旧值，是最
    难查的一类问题（改了半天没反应，还以为参数不生效）。
    """
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - python3-yaml 是声明的依赖
        raise ValueError(
            "读 --hw-config 需要 PyYAML（rosdep: python3-yaml）；"
            "不装它就只用命令行参数即可") from exc
    try:
        with open(path, encoding="utf-8") as handle:
            raw = yaml.safe_load(handle)
    except OSError as exc:
        raise ValueError(f"读不到硬件配置文件 {path}：{exc}") from exc
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        raise ValueError(f"{path} 的顶层必须是映射（键: 值），实际是 "
                         f"{type(raw).__name__}")
    out = {}
    for key, value in _flatten_config(raw):
        if key not in HW_CONFIG_KEYS:
            raise ValueError(
                f"{path} 含未知键 {key!r}；可用键："
                f"{'、'.join(sorted(HW_CONFIG_KEYS))}")
        out[key] = _check_config_type(path, key, value, HW_CONFIG_KEYS[key])
    return out


def _build_settings(args) -> DaemonSettings:
    """把命令行与（可选的）配置文件合成 DaemonSettings。

    优先级：**命令行 > 配置文件 > 代码里的默认值**。命令行要能压过配置文件，
    否则"临时改一下"就得去编辑文件——那正是最容易被忘掉的地方。
    """
    config = load_hw_config(args.hw_config) if args.hw_config else {}

    def pick(cli_value, key, fallback=None):
        if cli_value is not None:
            return cli_value
        return config.get(key, fallback)

    kwargs = {"shm_name": args.shm_name, "dry_run": args.dry_run,
              "verbose": args.verbose}
    for name, key in (
            ("port", "transport.port"),
            ("rate_hz", "transport.rate_hz"),
            ("command_timeout_s", "transport.command_timeout_s"),
            ("feedback_timeout_s", "transport.feedback_timeout_s"),
            ("connect_retry_s", "transport.connect_retry_s"),
            ("arm_timeout_s", "transport.arm_timeout_s"),
            ("request_timeout_s", "transport.request_timeout_s"),
            ("mit_passthrough", "policy.mit_passthrough"),
            ("exit_hold_s", "policy.exit_hold_s")):
        value = pick(getattr(args, name), key)
        if value is not None:
            kwargs[name] = value

    # 前馈覆盖按**键**合并：配置文件给默认，命令行给的键压过去。
    overrides = dict(config.get("policy.ff_overrides") or {})
    overrides.update({key: value
                      for key, value in _ff_overrides(args).items()
                      if value is not None})
    kwargs["ff_overrides"] = overrides
    return DaemonSettings(**kwargs)


class LitearmHwDaemon:
    """共享内存 ⇄ litearm-stm32 固件的桥接守护进程。"""

    def __init__(self, settings: DaemonSettings) -> None:
        self.settings = settings
        self.shm = SharedMemory(settings.shm_name, create=True)
        self.link: Optional[Stm32Link] = None
        self._fake: Optional[fake_firmware.FakeFirmware] = None
        self._stop = False
        self._mode = MODE_INIT
        self._reason = DAEMON_CONNECTING
        # 持位锚点：None = 尚未从实测位置锁定。``_hold()`` 拒绝发送未锚定的
        # 持位帧——绝不允许出现"用零位去 hold 一台不在零位的臂"。
        self._q_hold: Optional[List[float]] = None
        self._cycle = 0.0
        self._applied_command_cycle = 0.0
        self._last_command_age = float("inf")
        self._last_status_age = float("inf")
        self._disabled_sent = False
        self._lock_handle = None
        # 第二次信号置位 → 跳过剩余的退出持位时间。必须在这里初始化：
        # 非主线程运行时 _install_signal_handlers 会提前返回，不初始化就会
        # 在 _exit_hold_stream 里 AttributeError。
        self._exit_hold_skip = False
        # "是否真的连上过硬件"。只给 _shutdown 发最后一份状态用：
        # 连接失败时 self.link 是个已关闭的对象而不是 None，不能用它当判据。
        self._connected = False
        # 环内日志限频状态：key -> [最近输出时刻, 累计抑制次数]（见 _log_throttled）。
        self._log_throttle: "dict[str, List[float]]" = {}
        # 诊断：本进程实际下发过多少帧、其中运动帧多少（退出时汇总）。
        self.tx_motion_frames = 0
        # 链路计数上报（见 _report_link_counters）：上一窗口的快照与下一次的时间
        self._diag_tx = 0
        self._diag_dropped = 0
        self._diag_err: "Dict[Tuple[int, int], int]" = {}
        self._diag_next_at = 0.0

    # ────────────────────────── 小工具 ──────────────────────────

    def _log_throttled(self, level: int, key: str, message: str, *args) -> None:
        """限频日志：同一 key 首次立即输出，其后最多每 ``LOG_THROTTLE_S`` 秒一次。

        只能用于**控制环内**的持续性条件（故障持续、串口异常、锚点缺失……）；
        一次性事件（模式切换、连接成功）继续直接 ``log.info``。抑制次数在
        ``_shutdown`` 里汇总成一行——诊断信息不丢，但不会每拍都写终端。
        """
        now = time.monotonic()
        entry = self._log_throttle.get(key)
        if entry is not None:
            if now - entry[0] < LOG_THROTTLE_S:
                entry[1] += 1.0
                return
            entry[0] = now
        else:
            self._log_throttle[key] = [now, 0.0]
        log.log(level, message, *args)

    # ── 链路计数周期上报（2026-09-29）────────────────────────────────────
    #
    # 为什么必须上报：本链路 OUT 方向常年被固件的状态流（153 B @ 100 Hz ≈ 15 kB/s —— 上报率见 RPT_STATUS_MS=10）
    # 挤着，`send()` 在拥塞时会**整帧丢弃**。丢一帧本身安全（4 ms 后就有新帧），
    # 但**丢得太多**就会让固件的 100 ms 命令看门狗反复接管 ⇒ 臂在 fail-soft 持位
    # 与跟踪之间来回切（现场 = 启动后/负载高时**臂来回摇晃**）。
    # 原实现里 `tx_frames` 照涨、`errors` 不涨 ⇒ 这件事在本进程**一点痕迹都没有**
    # （真机排查时只能从固件状态帧的 `watchdog_tripped` 倒推）。
    # 现在每 DIAG_PERIOD_S 秒汇总一行；**送达率过低直接 WARN**。
    DIAG_PERIOD_S = 5.0

    def _report_link_counters(self, now: float, connected: bool) -> None:
        link = self.link
        if link is None or not connected or now < self._diag_next_at:
            return
        self._diag_next_at = now + self.DIAG_PERIOD_S
        frames = link.tx_frames - self._diag_tx
        dropped = link.tx_dropped - self._diag_dropped
        self._diag_tx, self._diag_dropped = link.tx_frames, link.tx_dropped
        # 本窗口**新增**的拒帧（累计值做差，只报增量；总量见 Stm32Link.counters）。
        # 重连会换一块新的 Stm32Link，计数从 0 起 ⇒ 差值为负的一律滤掉，不报假增量。
        rejects = {code: count - self._diag_err.get(code, 0)
                   for code, count in link.err_by_code.items()}
        rejects = {code: count for code, count in rejects.items() if count > 0}
        self._diag_err = dict(link.err_by_code)
        offered = frames + dropped
        if offered == 0 and not rejects:
            return
        rate = frames / self.DIAG_PERIOD_S
        # ★ 一并报"本进程自己的判断"与"固件状态帧里的看门狗标志"：真机上曾观察到
        #   共享内存里 `watchdog_tripped=1` 与 `last_error=DAEMON_OK(9)` **并存**
        #   （daemon 每周期先 _evaluate 再 _publish，两者本该一致）⇒ 只有让本进程
        #   自己把这两个值一起打出来，才能判断那个标志到底是不是假象。
        fw_wd = 0.0
        if link.status is not None:
            fw_wd = 1.0 if link.status.watchdog_tripped else 0.0
        text = ("链路计数（%.0fs）：请求 %.1f Hz / %d 帧 · **丢弃 %d 帧（%.1f%%）** · "
                "送达 %.1f Hz · rx=%d crc=%d errors=%d · 本进程 reason=%d · "
                "固件状态帧 watchdog_tripped=%d · enabled=%d")
        args = (self.DIAG_PERIOD_S, offered / self.DIAG_PERIOD_S, offered,
                dropped, (100.0 * dropped / offered) if offered else 0.0, rate,
                link.rx_frames, link.crc_errors, link.errors, int(self._reason),
                int(fw_wd), int(bool(link.status and link.status.enabled)))
        # ★ 拒帧与「送不到」是**两回事**，分开报：送达率可以满格而命令被固件**整条拒绝**，
        #   但两者的后果一样 —— 被拒的帧**不喂看门狗**（拒绝发生在 `ctrl_accept_*`
        #   内部、`watchdog_kick()` 之前）⇒ 持续被拒同样把固件推进 fail-soft
        #   （`kp×0.6` / `τ=0` / 无重力前馈）⇒ 臂下坠、来回摇。混进「拥塞」那条会查错方向。
        #
        #   ⚠ 但**只有运动帧被拒才值得报警**：启动期第一帧 ENABLE 必然回 `{0x10,0x03}`
        #   （固件要先写 7 台电机的 CMODE），那条由 `_enable()` 显式重发处理，属正常
        #   路径 —— 对它报警只会每次启动都刷一条假警报，把真信号淹掉。
        motion_rejects = {code: count for code, count in rejects.items()
                          if code[0] in MOTION_CMDS}
        if motion_rejects:
            log.warning(text + " —— ⚠" + rejected_text(motion_rejects)
                        + "（运动帧被拒）：被拒的帧**不喂看门狗**，持续被拒与「送不到」"
                          "后果相同（fail-soft 持位：kp×0.6 / τ=0 / 无重力前馈）⇒ 臂下坠。"
                          "最常见的是 (0x03, 0x02)：MOVE_JS 因「dq 全 0 且目标离实测"
                          " > 5 mrad」被固件整条拒绝（2026-09-24 门禁，见 `_hold_target`）。",
                        *args)
        # 固件看门狗门限 100 ms ⇒ 送达跌破 ~10 Hz 就会反复接管；20 Hz 起就该报警。
        elif rate < 20.0:
            log.warning(text + " —— ⚠ 送达率低于 20 Hz：固件看门狗（100 ms）会反复接管，"
                               "臂会在持位/跟踪之间来回切（看起来在摇）。查主机负载与串口"
                               "拥塞，或把守护进程频率调低（launch 的 daemon_rate_hz）"
                               "让这条链路别再被灌满。", *args)
        else:
            # 非运动帧的拒帧（如启动期 `{0x10,0x03}`）照实带上，但不升级成告警。
            log.info(text + rejected_text(rejects), *args)

    # ────────────────────────── 生命周期 ──────────────────────────

    def _acquire_singleton_lock(self) -> None:
        """守护进程单实例锁。

        两个守护进程同时写同一段共享内存会互相覆盖状态、且两个主控抢同一批
        电机是明确禁止的。固件侧当然也只接受一条命令流，但那时已经晚了。
        """
        path = f"/dev/shm{self.settings.shm_name}.lock"
        self._lock_handle = open(path, "w", encoding="utf-8")
        try:
            fcntl.flock(self._lock_handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno in (errno.EACCES, errno.EAGAIN):
                raise RuntimeError(
                    f"另一个 litearm 硬件守护进程已持有 {path}；"
                    f"同一块板子只允许一个控制进程。"
                    f"请先停止旧进程（或确认它已退出）再启动。") from exc
            raise
        self._lock_handle.write(f"{os.getpid()}\n")
        self._lock_handle.flush()

    def _release_singleton_lock(self) -> None:
        if self._lock_handle is not None:
            try:
                fcntl.flock(self._lock_handle, fcntl.LOCK_UN)
            except OSError:
                pass
            self._lock_handle.close()
            self._lock_handle = None
            try:
                os.unlink(f"/dev/shm{self.settings.shm_name}.lock")
            except OSError:
                pass

    def _install_signal_handlers(self) -> None:
        # 非主线程（例如嵌入测试进程运行）没有信号注册权限，跳过即可：
        # 退出语义仍由 ``_stop`` 标志驱动；standalone 进程一定在主线程。
        if threading.current_thread() is not threading.main_thread():
            log.debug("非主线程运行，跳过信号处理器安装")
            return

        def handler(signum, _frame):
            if self._stop:
                # 第二次 Ctrl-C：跳过剩余的退出持位时间，立刻收敛。
                self._exit_hold_skip = True
                log.info("再次收到信号 %s，跳过剩余持位时间",
                         signal.Signals(signum).name)
                return
            log.info("收到信号 %s，准备退出", signal.Signals(signum).name)
            self._stop = True

        self._exit_hold_skip = False
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, handler)

    # ────────────────────── 连接与固件身份 ──────────────────────

    def _connect(self) -> None:
        """建立链路、读固件身份与关节参数、（按需）覆盖前馈、使能。

        任何一步失败都关闭链路重来——绝不留半连状态。

        Raises:
            DaemonFatalError: 不可重试（license 未激活）。
            _RetryableConnect: 可重试（应答丢失、反馈未就绪、端口不在）。
        """
        if self.settings.dry_run and self._fake is None:
            # 无硬件演练：在 pty 上起一块假固件，链路照常走真实协议。
            self._fake = fake_firmware.FakeFirmware(
                num_joints=NUM_JOINTS)
            port = self._fake.start()
            log.info("dry-run：假固件已起在 %s（一阶运动学模型，"
                     "只能验证接口与数据通路，不能整定增益）", port)
            self.link = Stm32Link(port)
        elif self.link is None:
            self.link = Stm32Link(self.settings.port)

        try:
            self.link.open()
            self._read_firmware_identity()
            self._read_joint_params()
            self._read_ff_state()
            self._apply_ff_overrides()
            self._enable()
        except BaseException:
            self._close_link()
            raise

    # ─────────────────────── 启动期保活（2026-09-29）───────────────────────
    #
    # 为什么需要：固件的命令看门狗是 **100 ms**，而 `_connect()` 里有一串
    # **阻塞等待**（读固件版本 / 等状态帧 / 读 7 个关节参数 / 读前馈 / 等使能位）。
    # 这些等待期间命令流是断的 ⇒ 固件转入 fail-soft 持位（kp/kd×0.6、tau=0、
    # **无重力前馈**）⇒ 臂下垂；一有帧又跟踪回来 ⇒ 在重试节奏上来回切。
    #
    # ★ 真机实测（2026-09-29）：一次 0x24 应答丢失 ⇒ `_read_joint_params` 抛
    #   `_RetryableConnect` ⇒ 关链路 + 等 connect_retry_s + 整轮重读 ⇒ 启动窗口
    #   约 13 s 内固件看门狗反复接管，臂在持位/跟踪之间反复切（**看起来在来回
    #   摇晃**），插件刷 45 条「守护进程未跟随命令」。
    #   台架复现（扣住 6 次 0x24 应答）：整段启动只交换 4 帧、最大帧间隔 2014 ms。
    #
    # 修法 = 把每个阻塞等待切成小片，**片与片之间发一帧"按实测位置持位"** ——
    # 这正是 `_hold()` 文档串那条原则（"别停发让看门狗接管"）在启动路径上的补齐。
    # 帧只发给**已经使能**的固件（`status.enabled`）⇒ 冷启动本来就没使能时一帧都
    # 不发、不会凭空加磁；目标取最近一帧状态里的实测位置、dq=0，与 HOLDING 同义。
    _STARTUP_SLICE_S = 0.06        # 每片等待 ≪ 100 ms 看门狗，保证片间喂得上
    _STARTUP_READ_ATTEMPTS = 3     # 单个参数读原地重试次数（仍失败才升级为重连）

    def _feed_keepalive(self) -> bool:
        """给"已使能"的固件喂一帧按实测位置的持位帧。返回是否真的发出。"""
        link = self.link
        if link is None or not link.is_open:
            return False
        status = link.status
        if status is None or not status.enabled:
            return False
        count = min(self.settings.num_joints, len(status.joints))
        if count == 0:
            return False
        try:
            link.move_js([float(j.q) for j in status.joints[:count]],
                         [0.0] * count)
            return True
        except Exception:      # 保活绝不能把启动流程带崩
            return False

    def _poll_sliced(self, reader, total_s: float):
        """把一次阻塞等待切成 ≤ ``_STARTUP_SLICE_S`` 的片段，片间喂保活帧。

        ``reader(slice_s)`` 返回"拿到的东西"，或 None 表示本片没等到。
        """
        deadline = time.monotonic() + total_s
        while True:
            slice_s = min(self._STARTUP_SLICE_S,
                          max(0.0, deadline - time.monotonic()))
            if slice_s <= 0.0:
                return None
            value = reader(slice_s)
            if value is not None:
                return value
            self._feed_keepalive()

    def _read_firmware_identity(self) -> None:
        version = self._poll_sliced(
            lambda t: self.link.get_firmware(timeout_s=t),
            self.settings.request_timeout_s)
        if version is None:
            raise _RetryableConnect("读固件版本超时（0x41）")
        self.settings.firmware_version = version
        status = self._poll_sliced(
            lambda t: self.link.wait_status(t),
            self.settings.request_timeout_s * 2)
        if status is None:
            raise _RetryableConnect("未收到状态帧（0x40）")
        count = len(status.joints)
        if count != NUM_JOINTS:
            # 关节数不符是**契约级**错误（SHM 布局写死 7），不能凑合。
            raise DaemonFatalError(
                f"固件报告 {count} 个关节，本包按 {NUM_JOINTS} 编译 —— "
                f"是不是刷了单关节台架版固件（Litearm*-1J）？")
        self.settings.num_joints = count

    def _read_joint_params(self) -> None:
        """从固件读 kp/kd/tau_max/软限位（0x24）。

        ★ 2026-09-29：读到 None **不再立刻抛 `_RetryableConnect`** —— 「丢一次应答
        = 连接级失败」会让上层关链路、等 connect_retry_s、把版本/参数/前馈整轮重读，
        期间一帧都不喂固件（台架：6 次丢应答 ⇒ 启动窗口 12 s 无帧 ⇒ 看门狗反复
        接管 ⇒ 臂来回摇）。现在先在**原地**重试同一关节（重试之间照常喂保活帧），
        仍读不到才升级为连接级失败 —— 升级那条路保留，因为"真的连不上"仍该重连。
        """
        settings = self.settings
        for index in range(settings.num_joints):
            param = None
            for attempt in range(self._STARTUP_READ_ATTEMPTS):
                param = self._poll_sliced(
                    lambda t, i=index: self.link.get_joint_param(i, timeout_s=t),
                    settings.request_timeout_s)
                if param is not None:
                    break
                log.warning("读关节 %d 参数超时（0x24），原地重试 %d/%d",
                            index + 1, attempt + 1, self._STARTUP_READ_ATTEMPTS)
            if param is None:
                raise _RetryableConnect(f"读关节 {index + 1} 参数超时（0x24）")
            settings.kp[index] = param.kp
            settings.kd[index] = param.kd
            settings.tau_max[index] = param.tau_max
            settings.q_min[index] = param.q_min
            settings.q_max[index] = param.q_max

    def _read_ff_state(self) -> None:
        """读 ff_mask 与 kd_extra / hold_kp_gain。

        读不到不算失败（旧固件可能没有 0x2B/0x2C 读回口），但要留下明确的
        日志：没有它们，"到底是哪套前馈在环"就说不清了。
        """
        settings = self.settings
        mask = self._poll_sliced(
            lambda t: self.link.get_ff_mask(timeout_s=t),
            self.settings.request_timeout_s)
        if mask is None:
            log.warning("固件未响应 ff_mask 读回（0x2C item9）——"
                        "前馈现状未知，按 0 处理；--ff-* 覆盖仍可显式下发")
            settings.ff_mask = 0
            return
        settings.ff_mask = mask
        damping = self._poll_sliced(
            lambda t: self.link.get_ff_vec(proto.FF_VEC_KD_EXTRA, timeout_s=t),
            self.settings.request_timeout_s)
        if damping is not None:
            settings.ff_damping = list(damping[:settings.num_joints])
        gain = self._poll_sliced(
            lambda t: self.link.get_ff_scalar(proto.FF_SCALAR_HOLD_KP_GAIN,
                                              timeout_s=t),
            self.settings.request_timeout_s)
        if gain is not None and math.isfinite(gain) and gain > 0.0:
            settings.hold_kp_gain = float(gain)

    def _apply_ff_overrides(self) -> None:
        """把显式给出的 ``--ff-*`` 开关写进固件（默认一个都不写）。

        三个设计要点：

        * **只动被显式提到的位**，其余保持固件现状——否则"打开重力补偿"
          会顺手把摩擦/积分关掉。
        * ``kd_extra`` 没有独立 FF 位，它在固件里是 ``builtin_mode`` 内无条件
          生效的向量，只能靠把向量清零来关闭。所以 ``--damping-compensation``
          要读回原值（已经读过）再决定是清零还是恢复出厂 6.0。
        * 写完**必须读回校验**：固件对非法值会静默钳制（``params.c``），
          不校验就等于"以为改了其实没改"。
        """
        settings = self.settings
        if not settings.ff_overrides:
            return
        mask = settings.ff_mask
        for name, bit in FF_FLAG_TO_BITS.items():
            if name in settings.ff_overrides:
                mask = (mask | bit) if settings.ff_overrides[name] \
                    else (mask & ~bit)
        if "damping" in settings.ff_overrides:
            if settings.ff_overrides["damping"]:
                # 打开：沿用固件现值；若已被清零则恢复出厂向量。出厂值是
                # 固件 params/defaults.c 里的 6/6/6/6/0/0/0（J1~J4 承重轴才加）。
                target = list(settings.ff_damping)
                if all(abs(v) < 1e-9 for v in target):
                    target = [KD_EXTRA_FACTORY[i]
                              for i in range(settings.num_joints)]
            else:
                target = [0.0] * settings.num_joints
            if any(abs(a - b) > 1e-9 for a, b in
                   zip(target, settings.ff_damping)):
                self.link.set_ff_vec(proto.FF_VEC_KD_EXTRA, target)
                readback = self.link.get_ff_vec(
                    proto.FF_VEC_KD_EXTRA,
                    timeout_s=settings.request_timeout_s)
                if readback is None or any(
                        abs(a - b) > 1e-6 for a, b in
                        zip(readback[:settings.num_joints], target)):
                    raise _RetryableConnect("下发 kd_extra 后读回不一致")
                settings.ff_damping = list(readback[:settings.num_joints])
                log.info("前馈覆盖：kd_extra → %s", settings.ff_damping)

        if mask != settings.ff_mask:
            self.link.set_ff_mask(mask)
            readback = self.link.get_ff_mask(
                timeout_s=settings.request_timeout_s)
            if readback is None or readback != mask:
                raise _RetryableConnect(
                    f"下发 ff_mask 后读回不一致："
                    f"{readback if readback is None else hex(readback)} != {hex(mask)}")
            log.info("前馈覆盖：ff_mask 0x%03X → 0x%03X（%s）",
                     settings.ff_mask, readback, proto.format_ff_mask(readback))
            settings.ff_mask = readback

    def _enable(self) -> None:
        """使能并要求"真的加磁"（**原地重发** ENABLE，不关串口）。

        固件的 ``ENABLE`` 有两段语义（见 ``control_loop.c`` 的 ``ctrl_enable``，
        注释里记着真机实测时序 "ENABLE#1 -> 0x03(首写 CMODE), ENABLE#2 -> ACK"）：

        1. **固件启动后的第一次**：要先写 7 台电机的 CMODE 寄存器（把它们切进
           MIT 模式），那一刻回 ``0x03`` 并登记 ``enable_pending``；
        2. 反馈齐了由 ``enable_pending_poll`` **自动完成加磁**，主机重发
           ENABLE 才拿到 ACK。

        所以 ``0x03`` **不是失败**，是正常的第一步。原来的实现把它当"可重试的
        连接失败"抛出去——上层于是关掉串口、重连、把版本/参数/前馈全部重读一遍。
        功能上能成，但每次冷启动都白跑一轮，而且日志看着像故障。

        同样不能用 ACK 判断成功：ACK 只表示"登记"，必须等状态帧的 ``enabled`` 位。
        """
        settings = self.settings
        deadline = time.monotonic() + settings.arm_timeout_s
        logged_first_write = False
        while True:
            # 启动期保活：ENABLE 属"非运动命令"、**不踢固件的看门狗**
            # （control_loop.c 的 H2 fix），所以这个等待循环里也得自己喂帧。
            self._feed_keepalive()
            ok, reason = self.link.command(
                proto.CMD_ENABLE, timeout_s=settings.request_timeout_s)

            if ok is False:
                if reason == ENABLE_REASON_NO_LICENSE:
                    raise DaemonFatalError(
                        "固件 license 未激活，ENABLE 被拒（ERR{0x10,0x08}）。"
                        "用 litearm-stm32 的 tools/litearm-license 激活后重试；"
                        "在此之前所有运动命令都会被拒。")
                if reason == ENABLE_REASON_EMERGENCY:
                    # 急停/单轴故障锁存：重发没用，必须 reset。
                    # 注意锁存不一定来自人为急停：固件的主循环心跳监督（使能中
                    # main 500ms 无心跳）也会调 ctrl_emergency_stop()，现象一样。
                    raise _RetryableConnect(
                        "ENABLE 被拒（原因码 0x06：急停或关节故障锁存）——"
                        "这是锁存态，重连重发解不开，须显式复位："
                        "scripts/firmware_reset.py --reset --clear-faults"
                        "（先跑不带开关的只读模式看 mode/flags/joint_fault）")
                if reason != ENABLE_REASON_NOT_READY:
                    raise _RetryableConnect(f"ENABLE 被拒（原因码 {reason}）")
                if not logged_first_write:
                    logged_first_write = True
                    log.info("ENABLE 回 0x03（固件首次写电机 CMODE / 反馈未就绪）"
                             "——按固件时序原地重发，不重连")
            elif ok is None:
                # 固件 TX 忙会丢应答。丢 ACK 不代表没使能，所以不当失败处理，
                # 交给下面的 wait_enabled 用状态帧判定。
                log.warning("未收到 ENABLE 应答（固件 TX 忙会丢），"
                            "改用状态帧判定使能结果")

            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise _RetryableConnect(
                    f"ENABLE 后 {settings.arm_timeout_s:g}s 内未见 enabled 位"
                    f"——电机反馈未就绪或硬件故障")
            if self._poll_sliced(lambda t: self.link.wait_enabled(t),
                                 min(0.4, remaining)) is not None:
                return

    def _close_link(self) -> None:
        if self.link is not None:
            try:
                self.link.close()
            except Exception:
                log.debug("关闭链路时出现异常", exc_info=True)
        if self._fake is not None:
            try:
                self._fake.stop()
            except Exception:
                log.debug("停止假固件时出现异常", exc_info=True)
            self._fake = None
        if self.settings.dry_run:
            # dry-run 重启时要重新起一块干净的假固件（下电再上电的语义）。
            self.link = None

    # ────────────────────────── 模式裁决 ──────────────────────────

    def _evaluate(self, cmd: Optional[LitearmCommand], now: float,
                  status) -> Tuple[str, int]:
        """按固定优先级裁决本周期跟随模式，返回 ``(mode, reason_code)``。"""
        # 0) 链路/状态帧本身不可用 —— 一切都免谈。
        if status is None:
            return MODE_HOLDING, DAEMON_CONNECTING
        age = float("inf") if cmd is None else now - float(cmd.stamp_s)
        self._last_command_age = age
        status_age = self.link.status_age_s()
        self._last_status_age = status_age

        # 1) 状态帧陈旧：固件在跑但我们收不到了（串口挂死、板子复位）。
        #    比"命令陈旧"更严重——连反馈都没了，任何动作都是盲发。
        if status_age > self.settings.feedback_timeout_s:
            return MODE_HOLDING, DAEMON_HOLDING_FEEDBACK_STALE

        # 2) 命令帧陈旧（含 seqlock 撕裂视同陈旧）——最高优先级。
        #    ROS 侧控制环停了就立刻转持位，这是"上位机挂掉臂不坠"的核心。
        if not math.isfinite(age) or age > self.settings.command_timeout_s:
            return MODE_HOLDING, DAEMON_HOLDING_STALE_COMMAND

        # 3) 显式失能请求（命令帧新鲜才认，避免陈旧帧里的 enable=0 误失能）。
        if float(cmd.enable) == 0.0:
            return MODE_DISABLED, DAEMON_DISABLED

        # 4) 软急停。刻意**不**映射成固件的 0x12 急停（那会失能电机、臂会掉）
        #    ——既有语义是"保持高刚度持位"。
        if float(cmd.estop) != 0.0:
            return MODE_HOLDING, DAEMON_HOLDING_ESTOP

        # 5) 固件侧故障：FAULT 总标志或 joint_fault 位图（G7 断轴）。
        if status.fault or status.faulted_joints():
            return MODE_HOLDING, DAEMON_HOLDING_MOTOR_FAULT

        # 6) 反馈陈旧（固件侧 80ms 判据）。
        if status.feedback_stale:
            return MODE_HOLDING, DAEMON_HOLDING_FEEDBACK_STALE

        # 7) 过温警告。
        if status.temp_warning:
            return MODE_HOLDING, DAEMON_HOLDING_OVERTEMP

        # 8) 命令数值健全性：非有限数直接拒绝本帧（保留上一帧的持位）。
        if not (_finite(cmd.position) and _finite(cmd.velocity)
                and _finite(cmd.effort) and _finite(cmd.kp)
                and _finite(cmd.kd)):
            return MODE_HOLDING, DAEMON_HOLDING_BAD_COMMAND

        # 9) 固件看门狗曾接管（本进程曾卡 >100ms）。**仍然发帧**——发帧本身就是
        #    kick，标志会自己清掉；这里只是让上层看得见，不制造死锁。
        if status.watchdog_tripped:
            return MODE_TRACKING, DAEMON_HOLDING_WATCHDOG

        return MODE_TRACKING, DAEMON_OK

    def _enter(self, mode: str) -> None:
        """模式切换的一次性动作：进入 HOLDING 时把持位锚点锁到实测位置。"""
        if mode == self._mode:
            return
        if mode == MODE_HOLDING:
            status = self.link.status
            if status is None:
                return
            measured = [float(j.q) for j in status.joints]
            self._q_hold = [
                min(self.settings.q_max[i],
                    max(self.settings.q_min[i], measured[i]))
                for i in range(self.settings.num_joints)]
            log.info("进入 HOLDING 持位于 %s",
                     [round(value, 4) for value in self._q_hold])
        elif mode == MODE_TRACKING:
            log.info("进入 TRACKING，跟随 ros2_control 命令"
                     "（通道：%s）",
                     "MIT_ALL 透传" if self.settings.mit_passthrough
                     else "MOVE_JS")
        elif mode == MODE_DISABLED:
            self._disabled_sent = False
        self._mode = mode

    # ────────────────────────── 每周期下发 ──────────────────────────

    def _status_position(self) -> List[float]:
        status = self.link.status
        if status is None:
            return [0.0] * self.settings.num_joints
        return [float(j.q) for j in status.joints]

    def _track(self, cmd: LitearmCommand) -> None:
        """把命令帧转发给固件。

        MOVE_JS 路径**绝不带 tau_ff**：带上就会让固件整段关掉内置前馈
        （``builtin_mode`` 要求 ``!s_js_user_ff``）。所以 ``effort`` 命令接口
        在这条通道上是有意忽略的——想用它请走 ``--mit-passthrough``。
        """
        settings = self.settings
        count = settings.num_joints
        q_ref = [float(cmd.position[i]) for i in range(count)]
        dq_ref = [float(cmd.velocity[i]) for i in range(count)]

        if not settings.mit_passthrough:
            self.link.move_js(q_ref, dq_ref)
        else:
            kp = [min(MIT_KP_MAX, max(MIT_KP_MIN, float(cmd.kp[i])))
                  for i in range(count)]
            kd = [min(MIT_KD_MAX, max(MIT_KD_MIN, float(cmd.kd[i])))
                  for i in range(count)]
            tau = [float(cmd.effort[i]) for i in range(count)]
            self.link.move_mit_all(
                [(q_ref[i], dq_ref[i], kp[i], kd[i], tau[i])
                 for i in range(count)])
        self.tx_motion_frames += 1
        self._applied_command_cycle = float(cmd.cycle_count)

    def _hold_target(self) -> List[float]:
        """MOVE_JS 通道下持位帧的**位置字段**（MIT 通道不用它，见 :meth:`_hold`）。

        为什么不能继续用 ``self._q_hold``：固件 2026-09-24 起有一道门禁 ——
        ``MOVE_JS`` 的 ``dq`` **全 0** 且任一轴 ``|目标 − 实测| > 5 mrad``
        （``control_loop.c`` 的 ``JS_ZERO_DQ_EPS``）时，**整条命令被拒**
        （``ERR{0x03,0x02}``）。而拒绝发生在 ``watchdog_kick()`` **之前** ⇒
        被拒的帧**不喂看门狗** ⇒ 100 ms 后固件转 fail-soft
        （``kp×0.6`` / ``τ=0`` / 无重力前馈）⇒ 臂下坠。

        ``self._q_hold`` 是**进入 HOLDING 那一刻**的旧锚点，与当前实测之差 = 跟踪残差
        （见模块 docstring §1b，真机约 0.01 rad = 10 mrad）⇒ 正好落在那道门禁里。

        而 MOVE_JS 下位置字段**本来就被忽略**（``dq=0 ⇒ v_lim=0 ⇒ q_ref`` 冻结，
        见 §1b），所以填**当前实测位置**是最省事且唯一正确的选择：
        门禁恒不命中，而臂的行为一字不变（参考照旧冻结）。
        """
        status = self.link.status if self.link is not None else None
        count = self.settings.num_joints
        if status is None or len(status.joints) < count:
            # 没有可用的状态帧：退回旧锚点。位置字段本就被忽略，
            # 退回不会造成动作；只是这一帧可能被门禁拒掉而已。
            return self._q_hold
        return [float(j.q) for j in status.joints[:count]]

    def _hold(self) -> None:
        """持续下发持位帧（不是"停发让固件看门狗接管"）。

        停发会让固件在 100ms 后转入 fail-soft ——``tau=0`` 且刚度只有 0.6×、
        **没有重力前馈**，负载下会下垂。持续发帧则让固件用正常刚度 + 重力前馈
        把臂持住，这正是"上位机挂掉臂不坠"想要的效果。

        ⚠ 位置语义按通道分（见模块 docstring §1b）：

        * 默认（MOVE_JS）：``dq_ref=0`` → 固件参考冻结在**它自己的**位置，
          ``self._q_hold`` 只作日志/退出持位流用。实测这一条会带来约
          「跟踪滞后」量级的收敛位移（真机 ~0.01 rad），不是零。
        * ``--mit-passthrough``（MIT_ALL）：参考按 ``vel_max`` slew 到
          ``self._q_hold``，锚点是**真的**锚点 —— 所以那条通道下
          "锚定实测位置而非零位"的防线是硬需求。
        """
        settings = self.settings
        if self._q_hold is None:
            # 防御：持位锚点只能由 _enter(HOLDING) 从实测位置锁定。宁可不发帧
            # （固件保持上一条命令），也绝不发"零位 + 高刚度"帧——那会把不在
            # 零位的臂直接拽回零位。该分支每周期都会命中，必须限频。
            self._log_throttled(logging.ERROR, "hold-anchor",
                                "持位锚点尚未锁定，跳过本周期持位帧"
                                "（拒绝发送零位命令）")
            return
        zeros = [0.0] * settings.num_joints
        if not settings.mit_passthrough:
            # ★ 位置字段填**当前实测位置**而非旧锚点 `_q_hold` —— 见 `_hold_target`：
            #   MOVE_JS 下它本就被忽略，但填旧锚点会撞上固件 2026-09-24 的
            #   「dq 全 0 且目标离实测 > 5 mrad ⇒ 拒帧」门禁，被拒就不喂看门狗。
            self.link.move_js(self._hold_target(), zeros)
        else:
            # 力矩通道没有固件的持位交接可用，本进程按固件同名语义复刻：
            # kp × hold_kp_gain（0x2C item18，出厂 2.0），tau = 0。
            kp = [min(MIT_KP_MAX, settings.kp[i] * settings.hold_kp_gain)
                  for i in range(settings.num_joints)]
            kd = [min(MIT_KD_MAX, settings.kd[i])
                  for i in range(settings.num_joints)]
            self.link.move_mit_all(
                [(self._q_hold[i], 0.0, kp[i], kd[i], 0.0)
                 for i in range(settings.num_joints)])
        self.tx_motion_frames += 1

    def _tick(self, now: float) -> None:
        self.link.poll()
        status = self.link.status
        cmd = self.shm.try_read_command()
        mode, reason = self._evaluate(cmd, now, status)
        self._reason = reason

        if mode == MODE_DISABLED:
            if not self._disabled_sent:
                log.warning("ROS 侧请求失能：电机将失力，请扶住机械臂")
                self.link.disable()
                self._disabled_sent = True
            self._enter(mode)
            return

        self._enter(mode)
        if mode == MODE_TRACKING:
            assert cmd is not None
            self._track(cmd)
        elif mode == MODE_HOLDING:
            self._hold()

    def _publish(self, connected: bool, now: float) -> None:
        """把最新状态发布到共享内存。"""
        state = LitearmState()
        # stamp_s 在下面拿到 status 之后再写 —— 它要表示"这一帧的到达时刻"，
        # 不是"发布的时刻"。理由见那几行的注释。
        state.heartbeat_s = now
        state.connected = 1.0 if connected else 0.0
        state.dry_run = 1.0 if self.settings.dry_run else 0.0
        state.cycle_count = self._cycle
        state.applied_command_cycle = self._applied_command_cycle
        state.command_age_s = (self._last_command_age
                               if math.isfinite(self._last_command_age) else -1.0)
        state.last_error = float(self._reason)

        status = self.link.status if self.link is not None else None

        # ── stamp_s = **这一帧的到达时刻**，不是发布时刻 ──────────────────────
        # 为什么必须这样：固件只以 100Hz 主动上报（usb_cmd.c 的 RPT_STATUS_MS=10），
        # 而本函数**每拍都调用**（改频前 250Hz vs 上报 100Hz ⇒ 同一帧被重复发布
        # 2~3 次；现已把频率对齐到 100Hz）。保留这段的用意：只要两者**可能**不同频，
        # stamp_s 就必须说实话。
        # 以前这里写 now ⇒ 时间戳每拍前进、数据 2.5 拍才变一次，画出来是台阶；
        # 一旦对位置做差分 / 看速度就变成**周期 5 拍的锯齿**，而且数据的真实龄期
        # 被时间戳掩盖了（分析时无从分辨"这是新值还是复制的旧值"）。
        # 改成帧自带的时间戳（stm32_link.py 在收到帧时打的本地 monotonic 时刻）后：
        # 重复样本带**同一个**时间戳 ⇒ 消费侧一眼能看出重复，也能按时间戳正确重采样。
        # ⚠ 前提是 status.stamp_s 与 now 同为 CLOCK_MONOTONIC 秒（见 shm_bridge 的时间戳约定）。
        # 没有帧时退回 now：此时 connected=0、heartbeat_s 照走，存活性判据不受影响；
        # 而 feedback_age_s（整帧龄）本来就会把"帧是旧的"这件事报出来。
        state.stamp_s = float(status.stamp_s) if status is not None else now

        if status is not None:
            count = min(self.settings.num_joints, len(status.joints))
            joints = status.joints
            # ctypes 数组支持切片赋值：一次整块拷贝替代逐元素写入。
            state.position[:count] = [float(j.q) for j in joints[:count]]
            state.velocity[:count] = [float(j.dq) for j in joints[:count]]
            state.effort[:count] = [float(j.tau) for j in joints[:count]]
            state.temperature_mos[:count] = [float(j.t_mos)
                                             for j in joints[:count]]
            state.temperature_coil[:count] = [float(j.t_coil)
                                              for j in joints[:count]]
            state.error_code[:count] = [float(j.err) for j in joints[:count]]
            # ⚠ 固件的状态帧没有"逐关节反馈龄"字段（它只给一个全局 FB_STALE
            # 标志），所以这里发布的是**整帧的龄**。逐关节判据请用 error_code
            # 与 flags，不要按这个字段逐轴卡阈值。
            age = self.link.status_age_s()
            state.feedback_age_s[:count] = [age] * count
            state.feedback_received[:count] = [1.0] * count
            state.enabled = 1.0 if status.enabled else 0.0
            state.faulted = 1.0 if (status.fault or status.faulted_joints()) \
                else 0.0
            state.watchdog_tripped = 1.0 if status.watchdog_tripped else 0.0
        self.shm.publish_state(state)

    # ────────────────────────── 主循环 ──────────────────────────

    def run(self) -> int:
        self._acquire_singleton_lock()
        self._install_signal_handlers()
        settings = self.settings
        period = 1.0 / settings.rate_hz
        log.info("litearm 硬件守护进程启动：%s", settings.describe())

        self._connected = False      # _shutdown 要用它发最后一份状态（见下）
        connected = False
        exit_code = 0
        next_connect_attempt = 0.0
        next_tick = time.monotonic()
        try:
            while not self._stop:
                now = time.monotonic()

                if not connected:
                    # 未连接也要持续发心跳，ROS 侧才能区分
                    # 「守护进程没起来」与「守护进程起了但硬件连不上」。
                    if now >= next_connect_attempt:
                        try:
                            self._connect()
                            connected = True
                            self._connected = True
                            self._reason = DAEMON_HOLDING_STALE_COMMAND
                            # 强制下一周期执行 _enter 的锁位动作：持位锚点必须
                            # 来自实测位置（见 MODE_INIT 的注释）。重连时同样
                            # 重新锚定，因为断开期间臂可能已被移动。
                            self._mode = MODE_INIT
                            self._q_hold = None
                            log.info("硬件连接成功（固件 %s，%s）",
                                     settings.firmware_version,
                                     settings.port or "自动发现")
                            log.info("固件参数：%s", settings.describe_firmware())
                        except DaemonFatalError as exc:
                            log.error("%s", exc)
                            exit_code = 2
                            break
                        except Stm32AccessDenied as exc:
                            # 权限不足**不会自己好**：等下去、重试都改变不了结果，
                            # 必须人去加组后重新登录。当致命处理，别刷日志。
                            log.error("%s", exc)
                            exit_code = 2
                            break
                        except (_RetryableConnect, Stm32Error, OSError) as exc:
                            log.error("连接硬件失败，%.1fs 后重试：%s",
                                      settings.connect_retry_s, exc)
                            self._close_link()
                            self._connected = False
                            next_connect_attempt = now + settings.connect_retry_s
                    # 未连接期间既要发心跳（让 ROS 侧能区分"守护进程没起来"与
                    # "起来了但连不上"），**也要给已经使能的固件喂保活帧** ——
                    # 重连之前/退避期间链路可能还是开的，而固件那 100 ms 看门狗
                    # 不会等我们（见上面「启动期保活」那段）。
                    self._feed_keepalive()
                    self._publish(connected=False, now=now)
                    self._cycle += 1.0
                    next_tick += period
                    if next_tick < now - period:  # 落后过多则重锚，避免疯狂追赶
                        next_tick = now + period
                    _sleep_until(next_tick)
                    continue

                try:
                    self._tick(now)
                except Stm32NotConnected as exc:
                    self._log_throttled(logging.ERROR, "link-io",
                                        "串口断开，尝试重连：%s", exc)
                    self._close_link()
                    connected = False
                    self._connected = False
                    next_connect_attempt = now + settings.connect_retry_s
                except Stm32Error as exc:
                    # 固件侧错误（含被拒的命令）——不退出，转持位并把原因发布
                    # 出去，让 ROS 侧决定。限频：故障持续时每周期都会命中。
                    self._log_throttled(logging.ERROR, "cycle-error",
                                        "控制周期异常，转入持位：%s", exc)
                    self._reason = DAEMON_HOLDING_MOTOR_FAULT
                except OSError as exc:
                    self._log_throttled(logging.ERROR, "serial-io",
                                        "串口异常，尝试重连：%s", exc)
                    self._close_link()
                    connected = False
                    next_connect_attempt = now + settings.connect_retry_s

                self._report_link_counters(now, connected)
                self._publish(connected=connected, now=now)
                self._cycle += 1.0
                next_tick += period
                if next_tick < now - period:
                    next_tick = now + period
                _sleep_until(next_tick)
        finally:
            self._shutdown()
        return exit_code

    # ────────────────────────── 退出 ──────────────────────────

    def _exit_hold_stream(self) -> None:
        """退出前的持位流：把最后一段时间"买"给调用方收栈。

        launch 的 Ctrl-C 是同时打到所有进程的，ros2_control_node 停控制器、
        发下最后几条命令都要时间。这段时间里我们继续以正常周期发冻结参考，
        固件就会用正常刚度 **+ 重力前馈** 把臂持住；一旦停发，固件会因为
        ``hold`` 分支 ``tau=0`` 而按 ``G/kp`` 轻微下垂。

        第二次信号（或超时）立刻结束，不耽误人为急停。
        """
        seconds = self.settings.exit_hold_s
        if seconds <= 0.0 or self.link is None or self.link.status is None:
            return
        q_hold = self._q_hold if self._q_hold is not None \
            else self._status_position()
        self._q_hold = list(q_hold)
        log.info("退出持位流 %.1fs（再按一次 Ctrl-C 立即结束）", seconds)
        zeros = [0.0] * self.settings.num_joints
        period = 1.0 / self.settings.rate_hz
        deadline = time.monotonic() + seconds
        next_tick = time.monotonic()
        while time.monotonic() < deadline and not self._exit_hold_skip:
            try:
                # 本条循环不发 `poll()` 的话状态帧永远停在退出那一刻，`_hold_target()`
                # 就会一直报同一个陈旧位置（臂在持位中还会微沉）⇒ 迟早撞上固件那道
                # 「目标离实测 > 5 mrad」的门禁、被拒、不再喂看门狗。补上 poll。
                self.link.poll()
                if not self.settings.mit_passthrough:
                    self.link.move_js(self._hold_target(), zeros)
                else:
                    kp = [min(MIT_KP_MAX,
                              self.settings.kp[i] * self.settings.hold_kp_gain)
                          for i in range(self.settings.num_joints)]
                    kd = [min(MIT_KD_MAX, self.settings.kd[i])
                          for i in range(self.settings.num_joints)]
                    self.link.move_mit_all(
                        [(self._q_hold[i], 0.0, kp[i], kd[i], 0.0)
                         for i in range(self.settings.num_joints)])
            except OSError:
                break
            next_tick += period
            _sleep_until(next_tick)

    def _shutdown(self) -> None:
        """退出流程：持位流 → park 声明 → 关链路。

        ``0x20 park`` 是**声明**：之后命令停发、看门狗触发时固件用 1.0× 刚度
        持位（而不是 fail-soft 的 0.6×）。固件侧注释称它"PC 断开前的高刚度
        park 声明"，设计意图就是本函数这条路径。

        ⚠ park 后的持位**不含重力前馈**（固件 ``hold`` 分支 ``tau=0``），臂会
        按 ``G/kp`` 轻微下垂。这是固件既有行为，要彻底不掉只能保持栈运行。
        """
        self._reason = DAEMON_SHUTTING_DOWN
        try:
            # ⚠ 必须用"是否真的连上过"，不能用 `self.link is not None`：
            # 连接失败后 self.link 是个**已关闭的对象而不是 None**，用后者会在
            # 临死前发出 connected=1.0 —— 而 ROS 插件的 on_configure 会据此
            # 认定"守护进程就绪"并继续激活。真机踩过：串口没权限 → 守护进程
            # 退出 → 插件却报「守护进程就绪（状态：守护进程退出中）」并激活，
            # 拿的是一份冻结的关节位置。
            self._publish(connected=self._connected, now=time.monotonic())
        except Exception:
            log.debug("退出前发布状态失败", exc_info=True)

        self._exit_hold_stream()

        if self.link is not None and self.link.is_open:
            log.info("退出：PARK 高刚度持位声明 — SUPPORT THE ARM")
            try:
                self.link.park()
                # 给这帧一点时间真正出串口。USB CDC 有内部发送缓冲，"写完立刻
                # 关端口"可能把帧丢掉——而 park 声明是"PC 断开后按 1.0× 刚度
                # 持位而不是 fail-soft 0.6×"的唯一依据，丢了臂就多垂一截。
                deadline = time.monotonic() + PARK_FLUSH_S
                while time.monotonic() < deadline:
                    self.link.poll()
                    time.sleep(0.005)
            except Exception:
                log.exception("退出时 park 失败")
        self._close_link()
        self.shm.close()
        self._release_singleton_lock()
        suppressed = {key: entry[1] for key, entry in self._log_throttle.items()
                      if entry[1] > 0}
        if suppressed:
            # 环内日志限频的汇总：被压掉的都是"故障持续"类重复日志，这里一次性
            # 交代清楚（不打印的话运维无从知道故障持续了多久、发生了多少次）。
            log.info("环内日志限频汇总：%s",
                     "，".join(f"{key} 抑制 {int(count)} 次"
                               for key, count in suppressed.items()))
        log.info("守护进程已退出（下发运动帧 %d）", self.tx_motion_frames)


# ────────────────────────────── CLI ──────────────────────────────


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="litearm_hw_daemon",
        description="litearm 硬件守护进程：独占 USB CDC，经共享内存为 "
                    "ros2_control 提供命令/状态通道（litearm-stm32 固件）")
    parser.add_argument("--port", default=None,
                        help="litearm-stm32 的 USB CDC 设备路径"
                             "（默认按 VID:PID 1d50:606f 自动发现）")
    parser.add_argument("--shm-name", default=shm_bridge.DEFAULT_SHM_NAME,
                        help="POSIX 共享内存对象名")
    # ⚠ 下面这些 default=None 是**刻意的**：None 表示"命令行没给"，好让
    #   --hw-config 里的值补进来。写成具体默认值就分不清"没给"和"给成了默认值"，
    #   配置文件会被静默架空。真正的默认值在 DaemonSettings 里。
    parser.add_argument("--hw-config", default=None,
                        help="可选的 litearm_hw.yaml：只放 PC 侧参数"
                             "（端口/频率/超时/策略），关节级参数一律从固件读。"
                             "命令行优先于本文件")
    parser.add_argument("--rate-hz", type=float, default=None,
                        help="命令下发频率（固件控制环是 300Hz，看门狗 100ms）")
    parser.add_argument("--command-timeout-s", type=float, default=None,
                        help="命令帧陈旧阈值，超过即转持位")
    parser.add_argument("--feedback-timeout-s", type=float, default=None,
                        help="状态帧陈旧阈值（固件 100Hz 上报）")
    parser.add_argument("--connect-retry-s", type=float, default=None,
                        help="连接失败后的重试间隔")
    parser.add_argument("--arm-timeout-s", type=float, default=None,
                        help="ENABLE 后等待「真正加磁」（enabled 位）的超时")
    parser.add_argument("--request-timeout-s", type=float, default=None,
                        help="启动期配置查询的单条应答超时（固件 TX 忙会丢应答）")
    parser.add_argument("--exit-hold-s", type=float, default=None,
                        help="退出前继续下发持位参考的秒数（0 = 直接 park 退出）")
    parser.add_argument("--mit-passthrough", action=argparse.BooleanOptionalAction,
                        default=None,
                        help="切到 MIT_ALL 全透传通道：kp/kd/effort 逐帧生效、"
                             "固件不叠任何自家前馈（默认 MOVE_JS 位置模式）")
    parser.add_argument("--dry-run", action="store_true",
                        help="无硬件模式：在 pty 上起一块假固件，链路走真实协议")
    # 五个前馈开关是**三态**的（BooleanOptionalAction + default=None）：
    # 不传 = 不碰固件；给正的开关 = 置位；给 --no-xxx = 清位。
    # 没有 --no-xxx 就无法做"退回纯 PD"的 A/B 对照。
    ff = argparse.BooleanOptionalAction
    parser.add_argument("--gravity-compensation", action=ff, default=None,
                        help="覆盖固件 ff_mask 的 FF_G 位（不传=不碰固件）")
    parser.add_argument("--friction-compensation", action=ff, default=None,
                        help="覆盖固件 ff_mask 的 FF_FRICTION 位")
    parser.add_argument("--inertia-compensation", action=ff, default=None,
                        help="覆盖固件 ff_mask 的 FF_INERTIA|FF_CORIOLIS 位"
                             "（⚠ MOVE_JS 模式下固件不算惯量项，置位无用）")
    parser.add_argument("--integral-compensation", action=ff, default=None,
                        help="覆盖固件 ff_mask 的 FF_INTEGRAL 位")
    parser.add_argument("--damping-compensation", action=ff, default=None,
                        help="覆盖固件 kd_extra 向量（无独立 FF 位；"
                             "关 = 清零，开 = 恢复出厂 6/6/6/6/0/0/0）")
    parser.add_argument("--verbose", action="store_true",
                        help="透传链路层的详细日志")
    parser.add_argument("--log-level", default="INFO",
                        choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    return parser.parse_args(list(argv) if argv is not None else None)


def _ff_overrides(args: argparse.Namespace) -> dict:
    return {
        "gravity": args.gravity_compensation,
        "friction": args.friction_compensation,
        "inertia": args.inertia_compensation,
        "integral": args.integral_compensation,
        "damping": args.damping_compensation,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")
    if args.verbose:
        logging.getLogger("litearm.stm32_link").setLevel(logging.DEBUG)

    try:
        settings = _build_settings(args)
    except (ValueError, ShmError) as exc:
        log.error("参数无效：%s", exc)
        return 2

    try:
        daemon = LitearmHwDaemon(settings)
    except RuntimeError as exc:
        log.error("%s", exc)
        return 3
    except ShmError as exc:
        log.error("共享内存初始化失败：%s", exc)
        return 3

    return daemon.run()


if __name__ == "__main__":
    sys.exit(main())
