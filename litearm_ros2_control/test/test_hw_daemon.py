#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""硬件守护进程的行为测试（无硬件，全程 dry-run + pty 假固件）。

覆盖守护进程**自己**保留的职责。换底层之后，"前馈/PD/安全包络"整套都下移到
固件了，所以原先那批"逐项等于 pylitearm 模型输出"的测试没有对应物
——它们要测的东西现在跑在 STM32 上，不在本进程里。这里只测本进程还负责的：

* 数据通路：命令 → 状态收敛；状态帧 → 共享内存
* 命令通道语义：默认 MOVE_JS **不带 tau_ff**（带了就关掉固件内置前馈）
* 安全裁决：陈旧 / 软急停 / enable=0 / 非有限 / 固件故障标志 → 原因码
* 持位锚点：必须锚定实测位置（不得把臂拽回零位）
* 启动身份：从固件读回的版本与关节参数必须进日志
* 前馈覆盖：`--ff-*` 只动被点名的位，且读回校验
* 退出：先持位流、再 park 声明
* 单实例锁

两种夹具：

* ``_Daemon``（子进程）：跑真实 CLI 与信号路径，测端到端行为。
* ``_inproc_daemon``（同进程线程）：测**协议语义**——子进程里的假固件在测试
  进程里看不见，而"这一帧到底带没带 tau_ff"只能看发出去的字节。
"""

import logging
import math
import os
import signal
import subprocess
import sys
import threading
import time
import types
from contextlib import contextmanager
from pathlib import Path

import pytest

_SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _SOURCE_PYTHON.is_dir() and str(_SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PYTHON))

from litearm_ros2_control import hw_daemon as daemon_module  # noqa: E402
from litearm_ros2_control import stm32_proto as proto  # noqa: E402
from litearm_ros2_control import shm_bridge  # noqa: E402
from litearm_ros2_control.fake_firmware import FakeFirmware  # noqa: E402
from litearm_ros2_control.hw_daemon import (DaemonFatalError,  # noqa: E402
                                            DaemonSettings,
                                            LitearmHwDaemon)
from litearm_ros2_control.shm_bridge import (DAEMON_DISABLED,  # noqa: E402
                                             DAEMON_HOLDING_BAD_COMMAND,
                                             DAEMON_HOLDING_ESTOP,
                                             DAEMON_HOLDING_FEEDBACK_STALE,
                                             DAEMON_HOLDING_MOTOR_FAULT,
                                             DAEMON_HOLDING_OVERTEMP,
                                             DAEMON_HOLDING_STALE_COMMAND,
                                             DAEMON_HOLDING_WATCHDOG,
                                             DAEMON_OK, DAEMON_SHUTTING_DOWN,
                                             NUM_JOINTS, LitearmCommand,
                                             SharedMemory)
from litearm_ros2_control.stm32_link import Stm32Link  # noqa: E402

SHM_NAME = "/litearm_hw_pytest"
RATE_HZ = 250.0


# ────────────────────────────── 公共工具 ──────────────────────────────


def _daemon_environment():
    env = dict(os.environ)
    # 源码树直接跑 pytest 时，保证子进程能 import 到本包
    source_python = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "python"))
    parts = [p for p in env.get("PYTHONPATH", "").split(os.pathsep) if p]
    if source_python not in parts:
        parts.insert(0, source_python)
    env["PYTHONPATH"] = os.pathsep.join(parts)
    return env


def _cleanup(name=SHM_NAME):
    try:
        SharedMemory.unlink(name)
    except Exception:
        pass
    for suffix in (".lock",):
        try:
            os.unlink(f"/dev/shm{name}{suffix}")
        except OSError:
            pass


def _wait_until(predicate, timeout, interval=0.02):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(interval)
    return None


def _new_command(positions, **kwargs):
    cmd = LitearmCommand()
    for index in range(NUM_JOINTS):
        cmd.position[index] = float(positions[index])
        cmd.velocity[index] = float(kwargs.get("velocities",
                                               [0.0] * NUM_JOINTS)[index])
        cmd.effort[index] = float(kwargs.get("effort", 0.0))
        cmd.kp[index] = float(kwargs.get("kp", 200.0))
        cmd.kd[index] = float(kwargs.get("kd", 3.0))
    cmd.enable = float(kwargs.get("enable", 1.0))
    cmd.estop = float(kwargs.get("estop", 0.0))
    cmd.cycle_count = float(kwargs.get("cycle_count", 1.0))
    cmd.stamp_s = float(kwargs.get("stamp_s", time.monotonic()))
    return cmd


# 驱动速度（rad/s）。**必须给非零速度**：固件 MOVE_JS 的 dq_ref 同时是位置参考的
# slew 速率上限（`control_loop.c:2106/2227`：`v_lim = clamp(|dq_ref|,0,speed_limit)`
# 然后 `q_ref = slew_linear(target, q_ref, v_lim*dt)`），所以"只给位置不给速度"
# 在真机上参考根本不动。真实 JTC 两者都给，这里照它的做法来。
DRIVE_SPEED_RAD_S = 3.0


def _drive(shm, positions, cycles=60, period=0.01, **kwargs):
    """按真实 JTC 的做法持续发布"位置 + 朝目标的速度"，返回最后的状态。

    速度**每周期按最新实测重算**（方向 = 目标 − 实测，大小封顶到
    DRIVE_SPEED_RAD_S）。不在开头算一次就算完：那时守护进程可能还没发布状态
    （位置全 0），方向会算错。
    """
    cmd = _new_command(positions, **kwargs)
    for _ in range(cycles):
        if "velocities" not in kwargs:
            state = shm.read_state()
            for i in range(NUM_JOINTS):
                error = positions[i] - state.position[i]
                cmd.velocity[i] = max(-DRIVE_SPEED_RAD_S,
                                      min(DRIVE_SPEED_RAD_S,
                                          error / max(period, 1e-3)))
        cmd.cycle_count += 1.0
        cmd.stamp_s = time.monotonic()
        shm.publish_command(cmd)
        time.sleep(period)
    return shm.read_state()


class _Daemon:
    """守护进程子进程的上下文管理器（dry-run：它在 pty 上自带假固件）。"""

    def __init__(self, *extra_args, shm_name=SHM_NAME):
        self.extra_args = list(extra_args)
        self.shm_name = shm_name
        self.proc = None

    def __enter__(self):
        _cleanup(self.shm_name)
        self.proc = subprocess.Popen(
            [sys.executable, "-m", "litearm_ros2_control.hw_daemon",
             "--dry-run", "--shm-name", self.shm_name,
             "--rate-hz", str(RATE_HZ), "--log-level", "WARNING",
             *self.extra_args],
            env=_daemon_environment(),
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        shm = _wait_until(self._try_open, timeout=20.0)
        if shm is None:
            self.stop()
            raise RuntimeError("守护进程未在 20s 内连接就绪")
        return self

    def _try_open(self):
        try:
            shm = SharedMemory(self.shm_name, create=False)
        except shm_bridge.ShmError:
            return None
        try:
            if shm.read_state().connected == 1.0:
                return shm
        except Exception:
            pass
        shm.close()
        return None

    def __exit__(self, exc_type, exc, tb):
        self.stop()

    def stop(self, timeout=15):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
            try:
                self.proc.communicate(timeout=timeout)
            except subprocess.TimeoutExpired:  # pragma: no cover
                self.proc.kill()
                self.proc.communicate(timeout=timeout)
        _cleanup(self.shm_name)


@pytest.fixture
def daemon():
    with _Daemon():
        with SharedMemory(SHM_NAME, create=False) as shm:
            yield shm


# ─────────────────────── 同进程夹具（协议语义） ───────────────────────


class _InProc:
    """同进程运行的守护进程，暴露内部对象供断言。

    只用于"发出去的字节长什么样"这类问题——子进程夹具看不见里面的假固件。
    线程非主线程，所以 ``_install_signal_handlers`` 自动跳过（不会劫持 pytest）。
    """

    def __init__(self, shm_name, **setting_kwargs):
        self.shm_name = shm_name
        self.settings = DaemonSettings(shm_name=shm_name, dry_run=True,
                                       rate_hz=RATE_HZ, **setting_kwargs)
        self.daemon = LitearmHwDaemon(self.settings)
        self.thread = None
        self.fake = None

    def start(self, timeout=15.0):
        self.thread = threading.Thread(target=self.daemon.run, daemon=True)
        self.thread.start()

        def ready():
            fake = self.daemon._fake
            link = self.daemon.link
            if fake is None or link is None or link.status is None:
                return None
            return fake if self.daemon._mode != daemon_module.MODE_INIT \
                else None

        self.fake = _wait_until(ready, timeout=timeout)
        if self.fake is None:
            self.stop()
            raise RuntimeError("同进程守护进程未在超时内就绪")
        return self

    def stop(self, timeout=15.0):
        self.daemon._stop = True
        self.daemon._exit_hold_skip = True
        if self.thread is not None:
            self.thread.join(timeout=timeout)
            self.thread = None
        _cleanup(self.shm_name)

    def __enter__(self):
        return self.start()

    def __exit__(self, exc_type, exc, tb):
        self.stop()


@contextmanager
def _inproc_daemon(shm_name, **setting_kwargs):
    _cleanup(shm_name)
    runner = _InProc(shm_name, **setting_kwargs)
    try:
        yield runner.start()
    finally:
        runner.stop()


def _motion_frames(fake):
    """假固件收到的 MOVE_JS 帧：(q, dq, tau_or_None) 列表。"""
    return list(fake.move_js_log)


# ────────────────────────────── 数据通路 ──────────────────────────────


def test_tracks_commands(daemon):
    """正常跟踪：状态应收敛到命令位置，且守护进程报告 OK。"""
    target = [0.3, -0.2, 0.15, -0.1, 0.05, 0.0, 0.0]
    state = _drive(daemon, target, cycles=80)
    worst = max(abs(state.position[i] - target[i]) for i in range(NUM_JOINTS))
    assert state.last_error == DAEMON_OK, "应处于 TRACKING"
    assert worst < 0.01, f"跟踪误差过大：{worst}"
    assert state.applied_command_cycle > 0, "守护进程应确认已应用命令"
    assert state.command_age_s >= 0.0
    assert state.enabled == 1.0, "固件应处于使能态"
    assert state.faulted == 0.0


def test_publishes_firmware_diagnostics(daemon):
    """状态块里的诊断量必须来自固件状态帧，不能是占位零。

    这几个字段是 controller_manager / rviz 侧唯一能看到"硬件健康度"的窗口，
    全零会让"电机真的过温了"看起来和"一切正常"一模一样。
    """
    state = _drive(daemon, [0.1] * NUM_JOINTS, cycles=40)
    assert all(state.feedback_received[i] == 1.0 for i in range(NUM_JOINTS))
    assert all(math.isfinite(state.feedback_age_s[i])
               for i in range(NUM_JOINTS))
    assert all(state.feedback_age_s[i] < 0.2 for i in range(NUM_JOINTS))
    assert all(state.error_code[i] == 1.0 for i in range(NUM_JOINTS)), \
        "使能态下 err 应逐轴为 1"
    assert all(state.temperature_mos[i] > 0.0 for i in range(NUM_JOINTS))
    assert all(state.temperature_coil[i] > 0.0 for i in range(NUM_JOINTS))


def test_move_js_channel_never_carries_tau_ff():
    """默认通道的每一帧都必须是 2N 浮点（**不带 tau_ff**）。

    回归背景：固件的 ``builtin_mode`` 要求 ``MOVE_JS && !s_js_user_ff``——
    一旦载荷里出现 tau_ff（哪怕全 0），固件就把整套内置前馈（重力/摩擦/
    积分/kd_extra/量化补偿）**整段关掉**。所以"顺手传个全 0 的 tau 省事"
    会让臂失去重力补偿，是个安静且危险的失效。
    """
    with _inproc_daemon("/litearm_hw_notau", exit_hold_s=0.05) as runner:
        with SharedMemory("/litearm_hw_notau", create=False) as shm:
            _drive(shm, [0.15] * NUM_JOINTS, cycles=25)
            frames = _motion_frames(runner.fake)
            assert frames, "守护进程一帧都没发"
            assert all(tau is None for _q, _dq, tau in frames), \
                "默认通道出现了 tau_ff —— 固件的内置前馈会被整段关掉"
            assert all(len(q) == NUM_JOINTS for q, _dq, _t in frames)
            assert runner.settings.ff_mask == proto.FF_FACTORY_MASK, \
                "没给 --ff-* 时不应改动固件的 ff_mask"


def test_mit_passthrough_carries_kp_kd_effort():
    """``--mit-passthrough`` 走 MIT_ALL：kp/kd/effort 必须逐帧原样透传。"""
    kp_cmd, kd_cmd, tau_cmd = 137.0, 2.75, 1.25
    with _inproc_daemon("/litearm_hw_mit", exit_hold_s=0.05,
                        mit_passthrough=True) as runner:
        with SharedMemory("/litearm_hw_mit", create=False) as shm:
            _drive(shm, [0.05] * NUM_JOINTS, cycles=25, kp=kp_cmd,
                   kd=kd_cmd, effort=tau_cmd)
            snapshot = runner.fake.snapshot()
            assert snapshot["mode"] == proto.ARM_MODE_MOVE_MIT_ALL
            assert snapshot["kp_cmd"] == pytest.approx([kp_cmd] * NUM_JOINTS)
            assert snapshot["kd_cmd"] == pytest.approx([kd_cmd] * NUM_JOINTS)
            assert snapshot["tau_user"] == pytest.approx([tau_cmd] * NUM_JOINTS)


# ────────────────────────────── 安全裁决 ──────────────────────────────


def test_state_stamp_tracks_the_frame_not_the_publish(daemon):
    """``stamp_s`` 必须是**这一帧的到达时刻**，不是"发布时刻"。

    固件只以 100Hz 主动上报（``usb_cmd.c`` 的 ``RPT_STATUS_MS=10``），而守护进程
    以 250Hz 调用 ``_publish`` ⇒ 同一帧被重复发布 2~3 次。以前 ``stamp_s`` 写 ``now``：
    时间戳每拍前进、位置却 2.5 拍才变一次 —— 画出来是台阶，对位置做差分就成
    **周期 5 拍的锯齿**（真机上就是这么观察到的），而且数据的真实龄期被时间戳掩盖，
    分析时分不清"这是新值还是复制的旧值"。

    判据不依赖具体频率，只看两者是否同步：
      · 相邻两次读取之间**位置没变** ⇒ ``stamp_s`` 也必须没变；
      · 位置**变了** ⇒ ``stamp_s`` 必须前进；
      · 窗口内两种情形都要出现（否则用例是空的）；
      · ``heartbeat_s`` 仍然每拍前进 —— 存活性判据不能被这次改动削弱。
    """
    _drive(daemon, [0.05] * NUM_JOINTS, cycles=40)  # 先进入稳定跟踪

    samples = []
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        s = daemon.read_state()
        samples.append((s.stamp_s, s.heartbeat_s, tuple(s.position)))
        time.sleep(0.004)  # ≈ 发布节拍，尽量采到连续拍

    assert len(samples) > 20, f"采样太少（{len(samples)}），判据不成立"

    repeats = 0
    advanced = 0
    for (stamp0, hb0, pos0), (stamp1, hb1, pos1) in zip(samples, samples[1:]):
        if pos1 == pos0:
            repeats += 1
            assert stamp1 == stamp0, (
                "位置没变而 stamp_s 前进了 ⇒ stamp_s 还是发布时刻；"
                "重复样本会带不同时间戳，曲线就成锯齿")
        else:
            advanced += 1
            assert stamp1 > stamp0, "位置变了而 stamp_s 没前进"
        assert hb1 >= hb0, "heartbeat_s 必须单调不减（守护进程存活性判据）"

    assert repeats > 0, "窗口内没有重复样本 —— 这条用例测不出问题"
    assert advanced > 0, f"窗口内 stamp_s 一次都没前进（{len(samples)} 次采样）"


def test_stale_command_switches_to_holding(daemon):
    """ROS 侧停发命令 → 必须自动转持位并锁在原位（而不是继续追旧目标）。"""
    target = [0.25, -0.15, 0.1, -0.05, 0.0, 0.0, 0.0]
    _drive(daemon, target, cycles=60)

    state = _wait_until(
        lambda: (lambda s: s if int(s.last_error)
                 == DAEMON_HOLDING_STALE_COMMAND else None)(daemon.read_state()),
        timeout=3.0)
    assert state is not None, "命令陈死后未转入 HOLDING"
    held = list(state.position)

    # 再等一段，确认它真的锁住了（不是继续漂移）。
    #
    # ⚠ 允许的"漂移"不是 0，原因是 MOVE_JS 的语义：持位帧发的是
    # ``move_js(q_hold, dq=0)``，而 ``dq_ref`` 同时是位置参考的 slew 上限
    # （`v_lim = clamp(|dq_ref|,0,…)`）—— dq=0 时**固件冻结的是它自己的参考**，
    # 守护进程给的 ``q_hold`` 在位置项上被忽略。
    # 于是臂会从"实测位置"收敛到"固件参考"，位移 = 进入持位那一刻的跟踪滞后。
    # 这是正确的行为（停在**命令它去的地方**，而不是停在它恰好在的地方），
    # 所以这里卡的是"滞后量级的界"，不是零。
    time.sleep(0.4)
    after = daemon.read_state()
    assert int(after.last_error) == DAEMON_HOLDING_STALE_COMMAND
    drift = max(abs(after.position[i] - held[i]) for i in range(NUM_JOINTS))
    assert drift < 0.05, (
        f"持位期间位置漂移过大：{drift}（预期量级 = 跟踪滞后，不是 0；见上方注释）")


def test_resumes_tracking_after_stale(daemon):
    """命令恢复后应重新进入 TRACKING。"""
    _drive(daemon, [0.2] * NUM_JOINTS, cycles=40)
    time.sleep(0.3)  # 进入 HOLDING
    assert int(daemon.read_state().last_error) == DAEMON_HOLDING_STALE_COMMAND

    target = [-0.3] * NUM_JOINTS
    state = _drive(daemon, target, cycles=100)
    assert int(state.last_error) == DAEMON_OK, "命令恢复后未重新跟踪"
    worst = max(abs(state.position[i] - target[i]) for i in range(NUM_JOINTS))
    assert worst < 0.01, f"恢复跟踪后误差过大：{worst}"


def test_estop_holds(daemon):
    """软急停：即使命令帧新鲜也不跟随。

    刻意**不**映射成固件的 0x12 急停——那会失能电机、臂会掉下来，而软急停
    的既有语义是"保持高刚度持位"。这条测试同时守着"别把两者接错"。
    """
    _drive(daemon, [0.1] * NUM_JOINTS, cycles=40)
    state = _drive(daemon, [0.5] * NUM_JOINTS, cycles=20, estop=1.0)
    assert int(state.last_error) == DAEMON_HOLDING_ESTOP
    assert abs(state.position[0] - 0.5) > 0.1, "急停期间仍在跟随命令"
    assert state.enabled == 1.0, "软急停不该让电机失能"


def test_rejects_non_finite_command(daemon):
    """含 NaN/Inf 的命令必须被整帧拒绝，不得下发到电机。"""
    _drive(daemon, [0.1] * NUM_JOINTS, cycles=30)
    cmd = _new_command([0.1] * NUM_JOINTS)
    cmd.position[2] = float("nan")
    cmd.cycle_count += 1.0
    cmd.stamp_s = time.monotonic()
    daemon.publish_command(cmd)

    state = _wait_until(
        lambda: (lambda s: s if int(s.last_error)
                 == DAEMON_HOLDING_BAD_COMMAND else None)(daemon.read_state()),
        timeout=3.0)
    assert state is not None, "非有限命令未被拒绝"


def test_enable_zero_disables(daemon):
    """enable=0 走失能分支（真机上臂会下坠，因此只在显式请求时发生）。"""
    _drive(daemon, [0.1] * NUM_JOINTS, cycles=30)
    state = _drive(daemon, [0.1] * NUM_JOINTS, cycles=20, enable=0.0)
    assert int(state.last_error) == DAEMON_DISABLED
    assert state.enabled == 0.0, "固件侧应已失能"


def test_stale_enable_zero_is_ignored(daemon):
    """陈旧的 enable=0 不得让电机失能。

    守护进程先判陈旧再判 enable —— 否则"ROS 侧挂掉时最后一帧恰好是
    enable=0"会让臂直接掉下来。这是顺序敏感的安全分支。
    """
    cmd = _new_command([0.1] * NUM_JOINTS, enable=0.0,
                       stamp_s=time.monotonic() - 10.0)
    daemon.publish_command(cmd)

    state = _wait_until(
        lambda: (lambda s: s if int(s.last_error)
                 == DAEMON_HOLDING_STALE_COMMAND else None)(daemon.read_state()),
        timeout=3.0)
    assert state is not None, "陈旧 enable=0 未被识别为陈旧"
    assert int(state.last_error) != DAEMON_DISABLED
    assert state.enabled == 1.0, "陈旧 enable=0 不该让电机失能"


@pytest.mark.parametrize("inject,expected_reason", [
    ("joint_fault", DAEMON_HOLDING_MOTOR_FAULT),
    ("feedback_stale", DAEMON_HOLDING_FEEDBACK_STALE),
    ("temp_warning", DAEMON_HOLDING_OVERTEMP),
])
def test_firmware_flags_map_to_reason_codes(inject, expected_reason):
    """固件的故障标志必须映射成 ROS 侧能看懂的原因码。

    换底层之后**检测**在固件里（safety_check 五类），本进程只**翻译**。
    这层翻译要是漏了，故障就会表现为"臂不动但一切都是绿的"。
    """
    shm_name = f"/litearm_hw_flag_{inject}"
    with _inproc_daemon(shm_name, exit_hold_s=0.05) as runner:
        with SharedMemory(shm_name, create=False) as shm:
            _drive(shm, [0.1] * NUM_JOINTS, cycles=20)
            assert int(shm.read_state().last_error) == DAEMON_OK

            if inject == "joint_fault":
                runner.fake.inject_joint_fault(2)
            else:
                getattr(runner.fake, f"inject_{inject}")(True)

            state = _wait_until(
                lambda: (lambda s: s if int(s.last_error) == expected_reason
                         else None)(shm.read_state()),
                timeout=3.0)
            assert state is not None, f"{inject} 未映射成原因码 {expected_reason}"

            # 解除注入后必须能自动回到 TRACKING（不是一次性锁死）。
            if inject == "joint_fault":
                runner.fake.inject_joint_fault(2, on=False)
            else:
                getattr(runner.fake, f"inject_{inject}")(False)
            back = _wait_until(
                lambda: (lambda s: s if int(s.last_error) == DAEMON_OK
                         else None)(shm.read_state()),
                timeout=3.0)
            assert back is not None, f"{inject} 解除后未回到 TRACKING"


def _synthetic_status(flags):
    """造一帧状态（只关心 flags 时用）。"""
    joints = tuple(proto.JointStatus(0.0, 0.0, 0.0, 30.0, 35.0, 1)
                   for _ in range(NUM_JOINTS))
    return proto.StatusFrame(flags | proto.FLAG_ENABLED, 0, joints, 0,
                             stamp_s=time.monotonic())


def test_watchdog_trip_is_reported_but_does_not_stall():
    """固件看门狗曾接管时只上报，**不能**因此停发。

    停发会让"看门狗接管"变成自锁：固件等不到 kick 就一直处在 fail-soft，
    而守护进程也一直不敢发。发帧本身就是 kick，标志会自己清掉。

    ⚠ 这条**只能在 ``_evaluate`` 层测**：固件的 ``watchdog_check()`` 在命令
    正常流动时会主动清掉 WD_TRIPPED（``watchdog.c`` 的 else 分支），所以端到端
    跑的时候这个标志一个 300Hz 周期内就消失了，从状态帧上根本抓不到。
    """
    with _inproc_daemon("/litearm_hw_wd", exit_hold_s=0.05) as runner:
        daemon = runner.daemon
        cmd = _new_command([0.1] * NUM_JOINTS)
        now = time.monotonic()

        tripped = _synthetic_status(proto.ARM_FLAG_WATCHDOG_TRIPPED)
        mode, reason = daemon._evaluate(cmd, now, tripped)
        assert reason == DAEMON_HOLDING_WATCHDOG, "看门狗接管未被上报"
        assert mode == daemon_module.MODE_TRACKING, \
            "看门狗接管时不得停止发帧——发帧即 kick，停了就自锁"

        # 下一周期固件清掉标志 → 立刻回到 OK，不需要任何恢复动作。
        clean = _synthetic_status(0)
        mode, reason = daemon._evaluate(cmd, time.monotonic(), clean)
        assert reason == DAEMON_OK and mode == daemon_module.MODE_TRACKING


def test_evaluate_priority_stale_command_beats_enable_zero():
    """陈旧判定必须**先于** enable=0 —— 这是"上位机挂掉臂不坠"的核心。

    反过来的话，"ROS 侧挂掉时最后一帧恰好是 enable=0"会让电机直接失力。
    端到端那边有 ``test_stale_enable_zero_is_ignored`` 守着，这里再在
    裁决层钉一次优先级，免得将来重排时只改了其中一处。
    """
    with _inproc_daemon("/litearm_hw_prio", exit_hold_s=0.05) as runner:
        daemon = runner.daemon
        status = _synthetic_status(0)
        stale = _new_command([0.1] * NUM_JOINTS, enable=0.0,
                             stamp_s=time.monotonic() - 10.0)
        mode, reason = daemon._evaluate(stale, time.monotonic(), status)
        assert reason == DAEMON_HOLDING_STALE_COMMAND
        assert mode == daemon_module.MODE_HOLDING

        fresh = _new_command([0.1] * NUM_JOINTS, enable=0.0)
        mode, reason = daemon._evaluate(fresh, time.monotonic(), status)
        assert reason == DAEMON_DISABLED
        assert mode == daemon_module.MODE_DISABLED


def test_second_daemon_refuses_to_start(daemon):
    """单实例锁：同一块板子上不允许两个控制进程。"""
    second = subprocess.Popen(
        [sys.executable, "-m", "litearm_ros2_control.hw_daemon",
         "--dry-run", "--shm-name", SHM_NAME, "--log-level", "ERROR"],
        env=_daemon_environment(),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        out, _ = second.communicate(timeout=20)
    except subprocess.TimeoutExpired:  # pragma: no cover
        second.kill()
        out, _ = second.communicate()
        pytest.fail("第二个守护进程未被单实例锁拦住，仍在运行")
    assert second.returncode != 0, "第二个守护进程本应启动失败"
    assert "另一个" in out or "已持有" in out, f"错误信息不清晰：{out}"


# ────────────────────────────── 持位锚点 ──────────────────────────────


def test_holding_anchors_on_measured_position_not_zero():
    """启动后的持位帧必须锚定实测位置——绝不能把臂高刚度拽回零位。

    回归背景（真机实测）：``run()`` 曾在连接成功时把 ``_mode`` 预置成
    MODE_HOLDING，而 ``_enter`` 开头的 ``if mode == self._mode`` 守卫会因此
    跳过锁位动作，``_q_hold`` 保持初值 [0]*7 —— 首个持位帧直接把不在零位的
    臂拽回零位。这里让假固件停在明显非零位形，断言**所有**下发帧都锚在实测位置。
    """
    shm_name = "/litearm_hw_anchor"
    with _inproc_daemon(shm_name, exit_hold_s=0.05) as runner:
        measured = list(runner.fake.snapshot()["q"])
        assert max(abs(v) for v in measured) > 0.3, \
            "夹具前提：假固件应停在明显非零位形"
        # 不发任何命令，纯持位一段时间。
        time.sleep(0.3)
        frames = _motion_frames(runner.fake)
        assert frames, "守护进程未下发任何持位帧"
        worst = max(max(abs(q[i] - measured[i]) for i in range(NUM_JOINTS))
                    for q, _dq, _tau in frames)
        assert worst < 1e-6, (
            f"持位帧未锚定实测位置（最大偏差 {worst}）；"
            f"首帧位置={[round(v, 4) for v in frames[0][0]]}，"
            f"实测={[round(v, 4) for v in measured]}")
        assert all(all(abs(v) < 1e-9 for v in dq) for _q, dq, _t in frames), \
            "持位帧的速度参考必须为 0"


def test_hold_frames_follow_the_measured_position_not_a_frozen_anchor():
    """持位帧的位置字段必须跟着**当前实测位置**走，不能钉死在进入持位时的旧锚点。

    回归背景（固件门禁，2026-09-24）：``MOVE_JS`` 的 ``dq`` **全 0** 且任一轴
    ``|目标 − 实测| > 5 mrad``（``control_loop.c`` 的 ``JS_ZERO_DQ_EPS``）时，
    固件把**整条命令**拒掉（``ERR{0x03,0x02}``）；而拒绝发生在 ``watchdog_kick()``
    **之前** ⇒ 被拒的帧**不喂看门狗** ⇒ 100 ms 后固件转 fail-soft
    （``kp×0.6`` / ``τ=0`` / 无重力前馈）⇒ 臂下坠。

    ``_q_hold`` 是**进入 HOLDING 那一刻**的锚点；臂在持位中一旦被重力/外力带偏
    （真机约 0.01 rad = 10 mrad，是门禁阈值的两倍），差值就超过了 5 mrad ⇒
    之后的**每一个**持位帧都会被拒。

    而 ``MOVE_JS`` 下位置字段**本来就被忽略**（``dq=0 ⇒ v_lim=0 ⇒ q_ref`` 冻结，
    见模块 docstring §1b）⇒ 填实测位置**不改变臂的行为**，只是让命令不被拒。
    """
    shm_name = "/litearm_hw_hold_follow"
    with _inproc_daemon(shm_name, exit_hold_s=0.05) as runner:
        time.sleep(0.3)                 # 先攒一段持位帧（锚点 = 进入持位时的实测）
        before = list(runner.fake.snapshot()["q"])
        assert _motion_frames(runner.fake), "夹具前提：应已在持续下发持位帧"

        moved = [v + 0.02 for v in before]      # 20 mrad ≫ 门禁的 5 mrad
        runner.fake.set_position(moved)
        time.sleep(0.35)

        tail = _motion_frames(runner.fake)[-5:]
        worst = max(max(abs(q[i] - moved[i]) for i in range(NUM_JOINTS))
                    for q, _dq, _tau in tail)
        assert worst < 1e-6, (
            f"持位帧仍钉在旧锚点上（与当前实测最大偏差 {worst} rad）——"
            f"真机上这会被固件的 >5 mrad 门禁整条拒绝，而被拒的帧不喂看门狗，"
            f"臂会在 100 ms 后掉进 fail-soft")
        assert all(all(abs(v) < 1e-9 for v in dq) for _q, dq, _t in tail), \
            "持位帧的速度参考必须仍为 0"


# ────────────────────────────── 启动身份 ──────────────────────────────


def test_startup_log_reports_firmware_identity():
    """启动日志必须说清"动的是哪块板、哪一版固件、参数是多少"。

    换底层之后参数的唯一真源是固件，所以这一行是可排障性的下限：换了板子/
    刷了固件/改了参数之后，必须一眼看出守护进程实际在用哪一套值，而不是去猜。
    """
    settings = DaemonSettings(port="/dev/ttyACM0", dry_run=True,
                              rate_hz=RATE_HZ)
    settings.firmware_version = "Litearm1.8.0-7J"
    settings.kp = [400.0] * 2 + [300.0] * 2 + [50.0] * 3
    settings.ff_mask = proto.FF_FACTORY_MASK
    text = settings.describe_firmware()

    assert "Litearm1.8.0-7J" in text, "固件版本没进启动日志"
    assert "/dev/ttyACM0" in text, "端口没进启动日志"
    assert "MOVE_JS" in text, "通道选择没进启动日志"
    assert "MASTER|G|INERTIA|CORIOLIS" in text, "ff_mask 没展开成人能读的名字"
    for index in range(NUM_JOINTS):
        assert f"j{index + 1}:" in text, f"关节 {index + 1} 的参数没进日志"


def test_mit_passthrough_is_visible_in_startup_log():
    settings = DaemonSettings(dry_run=True, mit_passthrough=True)
    assert "MIT_ALL" in settings.describe_firmware()
    assert "MIT_ALL" in settings.describe()


def test_settings_reject_nonsense():
    with pytest.raises(ValueError):
        DaemonSettings(rate_hz=0.0)
    with pytest.raises(ValueError):
        DaemonSettings(command_timeout_s=-1.0)
    with pytest.raises(ValueError):
        DaemonSettings(exit_hold_s=-0.5)
    with pytest.raises(ValueError):
        DaemonSettings(feedback_timeout_s=float("nan"))


# ────────────────────────────── 前馈覆盖 ──────────────────────────────


def test_ff_override_touches_only_named_bits():
    """``--no-friction-compensation`` 只能清 FF_FRICTION，别的位一个都不许动。"""
    shm_name = "/litearm_hw_ff1"
    with _inproc_daemon(shm_name, exit_hold_s=0.05,
                        ff_overrides={"friction": False}) as runner:
        mask = runner.fake.snapshot()["ff_mask"]
        assert not mask & proto.FF_FRICTION, "FF_FRICTION 没被清掉"
        assert mask == proto.FF_FACTORY_MASK & ~proto.FF_FRICTION, (
            f"只该清 FRICTION 一位，实际 {proto.format_ff_mask(mask)}")


def test_ff_override_all_off_matches_pure_pd():
    """退回纯 PD 对照：五个开关都给 --no-*，位全清、kd_extra 归零。"""
    shm_name = "/litearm_hw_ff2"
    overrides = {"gravity": False, "friction": False, "inertia": False,
                 "integral": False, "damping": False}
    with _inproc_daemon(shm_name, exit_hold_s=0.05,
                        ff_overrides=overrides) as runner:
        snapshot = runner.fake.snapshot()
        assert snapshot["ff_mask"] & proto.FF_MASTER, \
            "MASTER 是总开关，不在五个开关的管辖范围内"
        assert not snapshot["ff_mask"] & (proto.FF_G | proto.FF_INERTIA
                                          | proto.FF_CORIOLIS
                                          | proto.FF_FRICTION
                                          | proto.FF_INTEGRAL)
        assert runner.settings.ff_damping == [0.0] * NUM_JOINTS


def test_ff_override_damping_on_restores_factory_vector():
    """先被清零、再打开 ``--damping-compensation`` → 回到出厂 kd_extra。"""
    shm_name = "/litearm_hw_ff3"
    with _inproc_daemon(shm_name, exit_hold_s=0.05,
                        ff_overrides={"damping": True}) as runner:
        assert runner.settings.ff_damping == \
            pytest.approx(list(daemon_module.KD_EXTRA_FACTORY))


def test_no_ff_override_leaves_firmware_untouched():
    """一个 ``--ff-*`` 都不给时，守护进程不许写固件的任何前馈参数。"""
    shm_name = "/litearm_hw_ff4"
    with _inproc_daemon(shm_name, exit_hold_s=0.05) as runner:
        written = [cmd for cmd, _payload in runner.fake.command_log
                   if cmd in (proto.CMD_SET_FF_FLAGS, proto.CMD_SET_FF_VEC,
                              proto.CMD_SET_FF_SCALAR)]
        assert written == [], f"未给开关却写了固件前馈参数：{written}"


# ────────────────────────────── 使能 / 退出 ──────────────────────────────


def test_enable_without_license_is_fatal():
    """license 未激活 → 不可重试的硬失败，且错误信息要说清怎么办。"""
    settings = DaemonSettings(shm_name="/litearm_hw_lic", dry_run=False,
                              arm_timeout_s=0.3, request_timeout_s=0.2)
    daemon = LitearmHwDaemon(settings)
    fake = FakeFirmware(licensed=False)
    try:
        daemon.link = Stm32Link(fake.start())
        daemon.link.open()
        with pytest.raises(DaemonFatalError) as excinfo:
            daemon._enable()
        text = str(excinfo.value)
        assert "license" in text and "0x08" in text
        assert "激活" in text, "错误信息没给恢复路径"
    finally:
        if daemon.link is not None:
            daemon.link.close()
        fake.stop()
        _cleanup(settings.shm_name)


def test_exit_streams_hold_then_declares_park():
    """退出顺序：先持续下发持位参考，最后发 ``0x20`` park 声明。

    持位流那段是有意义的——它让固件在栈关停期间仍用正常刚度 **+ 重力前馈**
    把臂持住；顺序反了（先 park 再持位）就会退化成一停发就下垂。
    """
    shm_name = "/litearm_hw_exit"
    _cleanup(shm_name)
    runner = _InProc(shm_name, exit_hold_s=0.3)
    try:
        runner.start()
        fake = runner.fake
        runner.daemon._stop = True
        runner.thread.join(timeout=15.0)

        commands = [cmd for cmd, _p in fake.command_log]
        assert proto.CMD_SET_MOTION_MODE in commands, "退出时没发 park 声明"
        park_index = len(commands) - 1 - commands[::-1].index(
            proto.CMD_SET_MOTION_MODE)
        after_park = commands[park_index + 1:]
        assert not [c for c in after_park if c == proto.CMD_MOVE_JS], \
            "park 之后还在发运动帧（顺序错了）"
        hold_frames = [f for f in fake.move_js_log]
        assert hold_frames, "退出持位流一帧都没发"
    finally:
        runner.stop()


def test_exit_hold_zero_skips_the_window():
    """``--exit-hold-s 0`` = 直接 park 退出（旧的即退行为）。"""
    shm_name = "/litearm_hw_exit0"
    _cleanup(shm_name)
    runner = _InProc(shm_name, exit_hold_s=0.0)
    try:
        runner.start()
        fake = runner.fake
        runner.daemon._stop = True
        started = time.monotonic()
        runner.thread.join(timeout=15.0)
        assert time.monotonic() - started < 5.0
        assert proto.CMD_SET_MOTION_MODE in [c for c, _p in fake.command_log]
    finally:
        runner.stop()


# ────────────────────────────── 日志限频 ──────────────────────────────


def test_loop_logs_are_throttled(caplog):
    """环内持续性故障不许每周期写一条日志（250Hz 会把控制环拖垮）。"""
    settings = DaemonSettings(shm_name="/litearm_hw_logthrottle", dry_run=True)
    daemon = LitearmHwDaemon(settings)
    try:
        with caplog.at_level(logging.WARNING, logger="litearm.hw_daemon"):
            for _ in range(200):
                daemon._log_throttled(logging.WARNING, "probe", "持续故障")
        emitted = [r for r in caplog.records if "持续故障" in r.getMessage()]
        assert len(emitted) == 1, f"限频失效，{len(emitted)} 条日志"
        assert daemon._log_throttle["probe"][1] == 199.0
    finally:
        daemon.shm.close()
        _cleanup("/litearm_hw_logthrottle")


def test_parse_args_ff_switches_are_tristate():
    """``--ff-*`` 必须三态：不传 / 打开 / ``--no-`` 关掉。

    两态的话就没法做"退回纯 PD"的 A/B 对照——那正是这些开关存在的理由。
    """
    args = daemon_module.parse_args([])
    assert daemon_module._ff_overrides(args) == {
        "gravity": None, "friction": None, "inertia": None,
        "integral": None, "damping": None}

    args = daemon_module.parse_args(["--gravity-compensation",
                                     "--no-friction-compensation"])
    overrides = daemon_module._ff_overrides(args)
    assert overrides["gravity"] is True
    assert overrides["friction"] is False
    assert overrides["integral"] is None

    # 只有显式给出的才进 settings（其余不碰固件）。
    settings = DaemonSettings(ff_overrides=overrides)
    assert settings.ff_overrides == {"gravity": True, "friction": False}


def test_mit_passthrough_flag_is_tristate():
    """``--mit-passthrough`` 也是三态：不传 = 让 litearm_hw.yaml 或默认值说话。

    两态（store_true）的话，配置文件里的 ``policy.mit_passthrough`` 就永远
    压不过命令行——命令行总会"显式地"给一个 False。
    """
    assert daemon_module.parse_args([]).mit_passthrough is None
    assert daemon_module.parse_args(["--mit-passthrough"]).mit_passthrough is True
    assert daemon_module.parse_args(
        ["--no-mit-passthrough"]).mit_passthrough is False
    assert daemon_module.parse_args(["--port", "/dev/pts/9"]).port == "/dev/pts/9"


# ─────────────────────── 硬件配置文件（litearm_hw.yaml） ───────────────────────

HW_CONFIG = os.path.join(os.path.dirname(__file__), "..", "config",
                         "litearm_hw.yaml")


def test_shipped_hw_config_matches_daemon_defaults():
    """随包分发的 litearm_hw.yaml 必须与守护进程的默认值逐项一致。

    这份文件的价值就是"它是一份说得清的默认值参考"（外加部署时不必改 launch）。
    一旦它和代码里的默认值分家，读文件的人就会以为自己在用这套值、实际不是。
    """
    config = daemon_module.load_hw_config(HW_CONFIG)
    defaults = DaemonSettings()
    assert config["transport.port"] == defaults.port
    assert config["transport.rate_hz"] == pytest.approx(defaults.rate_hz)
    assert config["transport.command_timeout_s"] == pytest.approx(
        defaults.command_timeout_s)
    assert config["transport.feedback_timeout_s"] == pytest.approx(
        defaults.feedback_timeout_s)
    assert config["transport.connect_retry_s"] == pytest.approx(
        defaults.connect_retry_s)
    assert config["transport.arm_timeout_s"] == pytest.approx(
        defaults.arm_timeout_s)
    assert config["transport.request_timeout_s"] == pytest.approx(
        defaults.request_timeout_s)
    assert config["policy.mit_passthrough"] == defaults.mit_passthrough
    assert config["policy.exit_hold_s"] == pytest.approx(defaults.exit_hold_s)
    assert config["policy.ff_overrides"] == defaults.ff_overrides == {}


def test_hw_config_does_not_carry_joint_level_parameters():
    """配置文件里**不许**出现关节级参数（它们的真源是固件参数表）。

    放进来就成了第二份真相：改了文件里的 kp 却没有任何效果（固件用的是自己
    那份），而且很难查——接口键校验把这条路直接堵死。
    """
    config = daemon_module.load_hw_config(HW_CONFIG)
    for key in config:
        assert not any(word in key for word in
                       ("kp", "kd", "tau_max", "q_min", "q_max", "limits")), \
            f"litearm_hw.yaml 里出现了关节级参数 {key!r}；它的真源是固件"


def test_hw_config_rejects_unknown_key(tmp_path):
    """打错一个字母必须报错，不能静默沿用旧值。"""
    path = tmp_path / "typo.yaml"
    path.write_text("transport:\n  por: /dev/ttyACM0\n", encoding="utf-8")
    with pytest.raises(ValueError) as excinfo:
        daemon_module.load_hw_config(str(path))
    assert "por" in str(excinfo.value)
    assert "transport.port" in str(excinfo.value), "报错应列出可用键"


@pytest.mark.parametrize("text", [
    'transport:\n  rate_hz: "250"\n',       # 字符串当数字
    "transport:\n  rate_hz: true\n",         # bool 当数字（True 也是 int）
    "policy:\n  mit_passthrough: 1\n",       # 数字当布尔
    "transport: 250\n",                      # 该是映射的地方给了标量
])
def test_hw_config_rejects_wrong_type(tmp_path, text):
    path = tmp_path / "bad.yaml"
    path.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError) as excinfo:
        daemon_module.load_hw_config(str(path))
    assert "期望" in str(excinfo.value) or "映射" in str(excinfo.value)


def test_hw_config_accepts_integer_for_float(tmp_path):
    """YAML 里写 250 而不是 250.0 是正常写法，不该被判成类型错。"""
    path = tmp_path / "int.yaml"
    path.write_text("transport:\n  rate_hz: 250\n", encoding="utf-8")
    assert daemon_module.load_hw_config(str(path)) == {"transport.rate_hz":
                                                      250.0}


def test_hw_config_cli_overrides_file():
    """命令行压过配置文件——临时改动不该被一次忘记删的文件悄悄覆盖。

    这也是所有可覆盖参数的 argparse 默认值都写成 None 的原因：写死默认值就
    分不清"没给"和"给成了默认值"，配置文件会被静默架空。
    """
    args = daemon_module.parse_args(
        ["--hw-config", HW_CONFIG, "--rate-hz", "111",
         "--no-mit-passthrough", "--friction-compensation"])
    settings = daemon_module._build_settings(args)
    assert settings.rate_hz == pytest.approx(111.0), "命令行没压过文件"
    assert settings.mit_passthrough is False
    # 前馈覆盖按**键**合并：文件是空 dict，命令行给的 friction 独立生效。
    assert settings.ff_overrides == {"friction": True}
    # 文件里给了、命令行没给的，仍然取自文件。
    assert settings.exit_hold_s == pytest.approx(2.0)


def test_hw_config_is_optional():
    """不给 --hw-config 时用**内置默认值**（不报错、不静默改别的）。

    ⚠ rate_hz 的默认值 2026-09-29 从 250 改成 100（与固件的状态上报率对齐，
      见 litearm_hw.yaml 的注释）。这里跟着改是**预期值随有意变更**，不是放宽断言 ——
      另一条 test_shipped_hw_config_matches_daemon_defaults 仍然逐值锁着
      "出厂 yaml == 代码默认"，所以两边不可能各改一边。
    """
    settings = daemon_module._build_settings(daemon_module.parse_args([]))
    assert settings.port == ""
    assert settings.rate_hz == pytest.approx(100.0)
    assert settings.exit_hold_s == pytest.approx(2.0)
    assert settings.ff_overrides == {}


def test_hw_config_missing_file_reports_clearly():
    with pytest.raises(ValueError) as excinfo:
        daemon_module.load_hw_config("/nonexistent/litearm_hw.yaml")
    assert "读不到" in str(excinfo.value)


# ──────────────── 连接失败时的退出状态（真机踩过的谎报） ────────────────


def test_shutdown_after_failed_connect_publishes_disconnected():
    """连接失败的守护进程退出时**不许**发 ``connected=1``。

    回归背景（真机，2026-09-21）：串口没权限 → 守护进程启动即退出 →
    但它临死前发了 ``connected=1.0``：判据用的是 ``self.link is not None``，
    而连接失败后 ``link`` 是个**已关闭的对象、不是 None**。
    后果是 ROS 插件的 ``on_configure`` 据此认定"守护进程就绪"并继续激活，
    而那份状态块里的关节位置是**冻结**的 —— 控制器会照着它发命令。
    """
    shm_name = "/litearm_hw_failconnect"
    _cleanup(shm_name)
    settings = DaemonSettings(shm_name=shm_name,
                              port="/dev/definitely-not-a-tty",
                              connect_retry_s=0.2)
    daemon = LitearmHwDaemon(settings)
    thread = threading.Thread(target=daemon.run, daemon=True)
    try:
        thread.start()
        time.sleep(1.2)
        daemon._stop = True
        thread.join(timeout=15.0)
        assert not thread.is_alive(), "守护进程未在超时内退出"

        with SharedMemory(shm_name, create=False) as shm:
            state = shm.read_state()
        assert state.connected == 0.0, (
            "连接失败的守护进程在退出时谎报 connected=1 —— "
            "插件的 on_configure 会据此激活并读到冻结的位置")
        assert int(state.last_error) == DAEMON_SHUTTING_DOWN
    finally:
        daemon._stop = True
        thread.join(timeout=5.0)
        _cleanup(shm_name)


# ────────────────────── 链路计数上报：拒帧分类 ──────────────────────


def _report_stub(link):
    """给 `_report_link_counters` 用的最小宿主 —— 直接调未绑定方法，不起线程。"""
    return types.SimpleNamespace(
        link=link, _diag_tx=0, _diag_dropped=0, _diag_err={},
        _diag_next_at=0.0, _reason=DAEMON_OK,
        DIAG_PERIOD_S=LitearmHwDaemon.DIAG_PERIOD_S)


def _fake_link(**kwargs):
    link = types.SimpleNamespace(
        tx_frames=2500, tx_dropped=0, rx_frames=2500, crc_errors=0,
        errors=0, err_by_code={}, status=None)
    for name, value in kwargs.items():
        setattr(link, name, value)
    return link


def test_link_report_warns_only_for_rejected_motion_frames(caplog):
    """拒帧上报**只对运动帧报警** —— 启动期首帧 ENABLE 必然回的 `{0x10,0x03}` 是正常路径。

    为什么非运动帧不能报警：固件 `ctrl_enable` 启动后第一次使能要先写 7 台电机的
    CMODE，因而**必然**回 `ERR{0x10,0x03}`，那条由 `_enable()` 显式重发处理
    （见 `stm32_proto` 的「按真机时序使能」）。对它报警只会**每次启动刷一条假警报**，
    把真信号淹掉。

    为什么运动帧必须报警：被拒 = **不喂看门狗**（拒绝发生在 `ctrl_accept_*` 内部、
    `watchdog_kick()` 之前），持续被拒会把固件推进 fail-soft
    （`kp×0.6` / `τ=0` / 无重力前馈）⇒ 臂下坠/来回摇 —— 而主机侧原来**毫无痕迹**。
    """
    report = LitearmHwDaemon._report_link_counters
    link = _fake_link(err_by_code={(proto.CMD_ENABLE, 0x03): 1})
    stub = _report_stub(link)

    with caplog.at_level(logging.INFO, logger="litearm.hw_daemon"):
        report(stub, 1000.0, True)
        assert not [r for r in caplog.records if r.levelno >= logging.WARNING], \
            f"正常的 ENABLE 0x03 被误报成告警：{caplog.text}"
        assert "0x10/0x03" in caplog.text, \
            f"非运动拒帧也应照实记进 INFO：{caplog.text}"

        caplog.clear()
        stub._diag_next_at = 0.0
        link.err_by_code = {(proto.CMD_ENABLE, 0x03): 1,
                            (proto.CMD_MOVE_JS, 0x02): 7}
        report(stub, 1001.0, True)

    warnings = [r.getMessage() for r in caplog.records
                if r.levelno >= logging.WARNING]
    assert len(warnings) == 1, f"运动帧被拒应恰好一条告警：{warnings}"
    assert "0x03/0x02×7" in warnings[0], \
        f"告警没带上被拒的运动帧计数：{warnings[0]}"
    assert "fail-soft" in warnings[0], \
        f"告警没说清后果（fail-soft 持位）：{warnings[0]}"


def test_link_report_counts_are_per_window_not_cumulative(caplog):
    """上报的是**本窗口增量** —— 否则稳态下每 5 s 重复刷同一条历史拒帧。"""
    report = LitearmHwDaemon._report_link_counters
    link = _fake_link(err_by_code={(proto.CMD_MOVE_JS, 0x02): 3})
    stub = _report_stub(link)

    with caplog.at_level(logging.INFO, logger="litearm.hw_daemon"):
        report(stub, 1000.0, True)
        assert "0x03/0x02×3" in caplog.text
        caplog.clear()
        # 累计值没变 ⇒ 本窗口零新增 ⇒ 不应再报。
        stub._diag_next_at = 0.0
        link.tx_frames += 2500
        report(stub, 1001.0, True)
        assert "0x03/0x02" not in caplog.text, \
            f"累计拒帧被当成本窗口增量重复上报：{caplog.text}"

        # 再涨一次 ⇒ 只报新增的那部分。
        caplog.clear()
        stub._diag_next_at = 0.0
        link.tx_frames += 2500
        link.err_by_code[(proto.CMD_MOVE_JS, 0x02)] = 5
        report(stub, 1002.0, True)
        assert "0x03/0x02×2" in caplog.text, \
            f"应只报新增的 2 条：{caplog.text}"
