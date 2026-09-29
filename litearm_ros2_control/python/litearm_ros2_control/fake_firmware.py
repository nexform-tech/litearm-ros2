#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm-stm32 固件的行为替身（pty 上的 USB CDC 协议）。

用途
----
1. **无硬件演练**：``--dry-run`` 让守护进程照常跑真实协议，对面是这个假固件，
   于是协议编解码、状态帧、看门狗、模式切换全部走通——比"在协议之上再套一层
   运动学模型"保真得多。
2. **测试夹具**：可注入故障/陈旧/丢帧，把真机上难以复现的分支逼出来。

纪律（为什么值得当真）
--------------------
* **编码侧手写**：状态帧/应答用 ``struct.pack`` 直接拼，**不复用
  ``stm32_proto`` 的打包器**——否则两边同一个 bug 会互相掩盖，
  往返测试就成了自证。真正的字节级对齐由 golden vector 测试负责。
* **只实现协议，不实现动力学**：关节响应是一阶滞后（``q ← q + (q_ref−q)·dt/τ``），
  用来验证接口与数据通路，**不能用来整定增益**。真机上动力学在固件里、
  前馈模型由 URDF 生成，这里一概没有。

默认参数取自固件 ``params/defaults.c``（7 关节整臂版），所以守护进程读回来的
kp/kd/tau_max/限位 就是真机上的那一组值——测试可以对着它们断言。
"""

import errno
import math
import os
import select
import struct
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

# 固件 params/defaults.c（Litearm1.8.0-7J）的出厂值。
FW_VERSION = "Litearm1.8.0-7J"

DEFAULT_KP = (400.0, 400.0, 300.0, 300.0, 50.0, 50.0, 50.0)
DEFAULT_KD = (5.0, 5.0, 4.0, 5.0, 2.5, 2.5, 2.5)
DEFAULT_TAU_MAX = (78.0, 78.0, 21.0, 21.0, 10.0, 10.0, 10.0)
DEFAULT_Q_MIN = (-2.809547, -1.727547, -2.809547, -3.071547,
                 -2.809547, -1.553547, -1.553547)
DEFAULT_Q_MAX = (2.809547, 1.727547, 2.809547, 0.017547,
                 2.809547, 1.553547, 1.553547)
DEFAULT_VEL_MAX = (2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.0)
DEFAULT_KD_EXTRA = (6.0, 6.0, 6.0, 6.0, 0.0, 0.0, 0.0)
DEFAULT_KI = (40.0, 40.0, 25.0, 25.0, 10.0, 10.0, 10.0)
DEFAULT_I_MAX = (20.0, 20.0, 6.0, 6.0, 4.0, 3.0, 3.0)

# 出厂 ff_mask = FF_MASTER|FF_G|FF_INERTIA|FF_CORIOLIS|FF_INTEGRAL|FF_QUANT
#                |FF_VELREF|FF_FRICTION  = 0x1BF。
# ⚠ FF_INERTIA 是 0x04、FF_WALL 是 0x40，差一位就变成"有墙无惯量"——
#   测试里用 proto.FF_FACTORY_MASK 交叉校验，别凭记忆改。
# 另有 0x1AF = 上述去掉 FRICTION（defaults.c 注释里的"回退=ff_mask 431"）。
DEFAULT_FF_MASK = 0x1BF

# 固件常量（litearm.h / usb_cmd.h）。
SOF = 0xA5
CMD_MOVE_J = 0x01
CMD_MOVE_JS = 0x03
CMD_MOVE_MIT = 0x04
CMD_MOVE_MIT_ALL = 0x05
CMD_ZERO_G = 0x06
CMD_ENABLE = 0x10
CMD_DISABLE = 0x11
CMD_EMERGENCY_STOP = 0x12
CMD_CLEAR_FAULTS = 0x13
CMD_RESET = 0x14
CMD_SET_MOTION_MODE = 0x20
CMD_SET_SPEED_PERCENT = 0x21
CMD_SET_JOINT_PARAM = 0x22
CMD_SET_JOINT_LIMITS = 0x23
CMD_GET_JOINT_PARAM = 0x24
CMD_SET_FF_VEC = 0x26
CMD_SET_FF_FLAGS = 0x27
CMD_SET_FF_SCALAR = 0x28
CMD_GET_FF_VEC = 0x2B
CMD_GET_FF_SCALAR = 0x2C
CMD_FF_PRESET = 0x31
CMD_GET_STATUS = 0x40
CMD_GET_FIRMWARE = 0x41

RSP_STATUS = 0x40
RSP_FIRMWARE = 0x44
RSP_ACK = 0x45
RSP_ERR = 0x46
RSP_JOINT_PARAM = 0x49
RSP_FF_VEC = 0x4B
RSP_FF_SCALAR = 0x4C

MODE_INIT = 0
MODE_MOVE_J = 1
MODE_MOVE_JS = 3
MODE_MOVE_MIT = 4
MODE_MOVE_MIT_ALL = 5
MODE_EMERGENCY = 6
MODE_ZERO_G = 7

FLAG_FAULT = 1 << 0
FLAG_WATCHDOG_TRIPPED = 1 << 1
FLAG_FEEDBACK_STALE = 1 << 2
FLAG_TEMP_WARNING = 1 << 3
FLAG_POSITION_VIOLATION = 1 << 4
FLAG_OVERSPEED = 1 << 5

# 命令看门狗 0.10s（params/defaults.c）。
WATCHDOG_TIMEOUT_S = 0.10

# 状态帧上报周期（usb_cmd.c 的 RPT_STATUS_MS=10 → 100Hz）。
REPORT_PERIOD_S = 0.01

# 一阶响应的时间常数（**只是运动学近似**，不是动力学）。
JOINT_LAG_S = 0.08


def _crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE。与固件 crc16.c 一致（测试里有独立黄金向量）。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 \
                else (crc << 1) & 0xFFFF
    return crc


def _frame(cmd: int, payload: bytes = b"") -> bytes:
    body = bytes([SOF, cmd, len(payload)]) + payload
    crc = _crc16(body)
    return body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


class FakeFirmware:
    """在 pty 上模拟一块跑 Litearm1.8.0-7J 的板子。

    用法::

        fw = FakeFirmware().start()        # start() 返回客户端要打开的路径
        link.open(fw)
        link.enable(); link.wait_enabled()
        ...
        fw.stop()

    线程模型：一个后台线程按 ``tick_hz`` 步进（固件是 TIM3 300Hz），
    每步收命令、推关节、按 100Hz 发状态帧。
    """

    def __init__(self, num_joints: int = 7, *, licensed: bool = True,
                 tick_hz: float = 300.0, report_period_s: float = REPORT_PERIOD_S,
                 enable_delay_s: float = 0.05) -> None:
        self.n = int(num_joints)
        self.licensed = bool(licensed)
        self.tick_hz = float(tick_hz)
        self.report_period_s = float(report_period_s)
        # 固件 ENABLE 两段：先写 CMODE，反馈齐了才加磁。这里用固定延时替身。
        self.enable_delay_s = float(enable_delay_s)

        # 关节参数（0x22/0x23/0x24 读写的就是这些）。只取前 n 项。
        self.kp = list(DEFAULT_KP[:self.n])
        self.kd = list(DEFAULT_KD[:self.n])
        self.tau_max = list(DEFAULT_TAU_MAX[:self.n])
        self.q_min = list(DEFAULT_Q_MIN[:self.n])
        self.q_max = list(DEFAULT_Q_MAX[:self.n])
        self.vel_max = list(DEFAULT_VEL_MAX[:self.n])
        self.ff_mask = DEFAULT_FF_MASK
        self.ff_vec: Dict[int, List[float]] = {
            1: [0.0] * self.n,          # friction（误差方向库伦）
            2: list(DEFAULT_KI[:self.n]),
            3: list(DEFAULT_I_MAX[:self.n]),
            7: [1.0] * self.n,          # gravity_scale
            8: [1.0] * self.n,          # inertia_scale
            15: list(DEFAULT_KD_EXTRA[:self.n]),
        }
        self.ff_scalar: Dict[Tuple[int, int], float] = {
            (1, 0): 0.15,               # fric_db
            (3, 0): 60.0,               # friction_slew
            (4, 0): 0.0,                # payload_mass
            (7, 0): 1.0,                # friction_model = v2
            (8, 0): 0.05,               # fric_v2_eps
            (18, 0): 2.0,               # hold_kp_gain
        }
        self.speed_percent = 100

        # 运动状态
        self.q = [-0.4, 1.3, -0.9, 0.0, 0.5, -0.2, 0.1][:self.n]
        self.dq = [0.0] * self.n
        self.tau = [0.0] * self.n
        self.t_mos = [32.0] * self.n
        self.t_coil = [35.0] * self.n
        self.q_ref = list(self.q)
        self.js_target_q = list(self.q)
        self.dq_ref = [0.0] * self.n
        self.tau_user = [0.0] * self.n
        self.kp_cmd = list(self.kp)
        self.kd_cmd = list(self.kd)

        self.mode = MODE_INIT
        self.enabled = False
        self.enable_pending_at: Optional[float] = None
        # 与固件 ctrl_mode_written 同义：固件启动后是否已写过电机 CMODE。
        # 未写过时第一次 ENABLE 必然回 0x03（见 _cmd_enable）。
        self._cmode_written = False
        self.park_requested = False
        self.joint_fault = 0
        self.seq = 0
        self.watchdog_tripped = False
        self.last_kick_s = 0.0
        self.s_js_user_ff = False
        self.err_code = [0] * self.n       # 0=失能，1=使能，其余=故障码

        # 故障注入（测试用）
        self._inj_feedback_stale = False
        self._inj_temp_warning = False
        self._inj_overspeed = False
        self._inj_pos_violation = False
        self._drop_replies = 0

        # 观测出口（测试断言用）
        self.command_log: List[Tuple[int, bytes]] = []
        self.move_js_log: List[Tuple[List[float], List[float],
                                     Optional[List[float]]]] = []

        self._master: Optional[int] = None
        self._slave: Optional[int] = None
        self._port: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._rx = bytearray()
        self._lock = threading.Lock()

    # ────────────────────────── 生命周期 ──────────────────────────

    @property
    def port(self) -> Optional[str]:
        """客户端要打开的路径（``/dev/pts/N``）。"""
        return self._port

    def start(self) -> str:
        """建 pty、起后台线程，返回客户端路径。"""
        master, slave = os.openpty()
        # 原始模式必须从我们这侧设置，否则默认的 ECHO 会把客户端自己发的
        # 命令回显给它（CdcFramer 能容忍，但会污染诊断计数）。
        self._set_raw(slave)
        self._master, self._slave = master, slave
        self._port = os.ttyname(slave)
        self.last_kick_s = time.monotonic()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="fake-firmware",
                                        daemon=True)
        self._thread.start()
        return self._port

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        for fd in (self._master, self._slave):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        self._master = self._slave = None

    def __enter__(self) -> "FakeFirmware":
        self.start()
        return self

    def __exit__(self, *_exc) -> None:
        self.stop()

    @staticmethod
    def _set_raw(fd: int) -> None:
        import termios
        attrs = termios.tcgetattr(fd)
        attrs[0] &= ~(termios.ICRNL | termios.IXON)
        attrs[1] &= ~termios.OPOST
        attrs[3] &= ~(termios.ECHO | termios.ICANON | termios.ISIG)
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, attrs)

    # ─────────────────────── 故障/行为注入 ───────────────────────

    def inject_joint_fault(self, index: int, on: bool = True) -> None:
        """置/清 ``joint_fault`` 位图（固件单轴断轴时只失能该轴）。"""
        if on:
            self.joint_fault |= 1 << index
            self.err_code[index] = 0x0D
        else:
            self.joint_fault &= ~(1 << index)
            self.err_code[index] = 1 if self.enabled else 0

    def inject_feedback_stale(self, on: bool = True) -> None:
        self._inj_feedback_stale = bool(on)

    def inject_temp_warning(self, on: bool = True) -> None:
        self._inj_temp_warning = bool(on)

    def inject_overspeed(self, on: bool = True) -> None:
        self._inj_overspeed = bool(on)

    def inject_position_violation(self, on: bool = True) -> None:
        self._inj_pos_violation = bool(on)

    def drop_replies(self, count: int = 1) -> None:
        """接下来的 ``count`` 条**应答**不发（模拟固件 TX 忙丢帧）。

        只丢 ACK/ERR/读回应答，不丢状态帧——否则"等不到 ACK"与"等不到状态"
        两件事会混在一起，测不出想测的那条分支。
        """
        self._drop_replies = int(count)

    def set_position(self, values: Sequence[float]) -> None:
        """直接摆放关节位置（测试装配，不走运动模拟）。"""
        with self._lock:
            self.q = [float(v) for v in values]
            self.q_ref = list(self.q)

    def snapshot(self) -> Dict[str, object]:
        with self._lock:
            return {
                "mode": self.mode, "enabled": self.enabled,
                "q": list(self.q), "dq": list(self.dq),
                "q_ref": list(self.q_ref), "tau_user": list(self.tau_user),
                "kp_cmd": list(self.kp_cmd), "kd_cmd": list(self.kd_cmd),
                "watchdog_tripped": self.watchdog_tripped,
                "joint_fault": self.joint_fault,
                "s_js_user_ff": self.s_js_user_ff,
                "ff_mask": self.ff_mask,
            }

    # ────────────────────────── 主循环 ──────────────────────────

    def _run(self) -> None:
        tick = 1.0 / self.tick_hz
        next_tick = time.monotonic()
        next_report = next_tick
        last = next_tick
        while not self._stop.is_set():
            now = time.monotonic()
            dt = max(1e-6, now - last)
            last = now
            self._receive()
            self._step(dt, now)
            if now >= next_report:
                next_report = now + self.report_period_s
                self._send(RSP_STATUS, self._build_status(), droppable=False)
            next_tick += tick
            sleep_s = next_tick - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            elif sleep_s < -tick:      # 落后太多就重锚，别疯狂追赶
                next_tick = time.monotonic()

    def _receive(self) -> None:
        if self._master is None:
            return
        while True:
            try:
                ready = select.select([self._master], [], [], 0)[0]
                if not ready:
                    return
                chunk = os.read(self._master, 4096)
            except (BlockingIOError, OSError) as exc:
                if isinstance(exc, OSError) and exc.errno not in (
                        errno.EAGAIN, errno.EWOULDBLOCK, errno.EIO):
                    raise
                return
            if not chunk:
                return
            self._rx += chunk
            self._drain_frames()

    def _drain_frames(self) -> None:
        while len(self._rx) >= 3:
            if self._rx[0] != SOF:
                del self._rx[0]
                continue
            length = self._rx[2]
            need = 3 + length + 2
            if len(self._rx) < need:
                return
            body = bytes(self._rx[:3 + length])
            crc = self._rx[3 + length] | (self._rx[4 + length] << 8)
            payload = bytes(self._rx[3:3 + length])
            del self._rx[:need]
            if _crc16(body) != crc:
                continue            # 坏帧丢弃，不回 ERR（固件也只计 CRC 错误）
            cmd = body[1]
            with self._lock:
                self.command_log.append((cmd, payload))
            self._handle(cmd, payload)

    # ────────────────────────── 命令处理 ──────────────────────────

    def _handle(self, cmd: int, payload: bytes) -> None:
        now = time.monotonic()
        with self._lock:
            if cmd == CMD_MOVE_JS:
                self._cmd_move_js(payload, now)
            elif cmd == CMD_MOVE_MIT_ALL:
                self._cmd_move_mit_all(payload, now)
            elif cmd == CMD_MOVE_J:
                self._cmd_move_j(payload, now)
            elif cmd == CMD_ENABLE:
                self._cmd_enable(now)
            elif cmd == CMD_DISABLE:
                self._cmd_disable()
            elif cmd == CMD_EMERGENCY_STOP:
                self._cmd_emergency()
            elif cmd == CMD_CLEAR_FAULTS:
                self._cmd_clear_faults()
            elif cmd == CMD_RESET:
                self._cmd_reset(now)
            elif cmd == CMD_ZERO_G:
                if len(payload) < 1:
                    self._err(cmd, 0x01)
                else:
                    self.mode = MODE_ZERO_G if payload[0] else MODE_INIT
                    self.last_kick_s = now      # 0x06 自带 kick
                    self._ack(cmd)
            elif cmd == CMD_SET_MOTION_MODE:
                if len(payload) < 1:
                    self._err(cmd, 0x01)
                else:
                    # mode 0 = 声明 park；其余清除声明。刻意不 kick 看门狗。
                    self.park_requested = payload[0] == 0
                    self._ack(cmd)
            elif cmd == CMD_SET_SPEED_PERCENT:
                if len(payload) < 1:
                    self._err(cmd, 0x01)
                else:
                    self.speed_percent = max(0, min(100, payload[0]))
                    self._ack(cmd)
            elif cmd == CMD_GET_JOINT_PARAM:
                self._cmd_get_joint_param(payload)
            elif cmd == CMD_SET_JOINT_PARAM:
                self._cmd_set_joint_param(payload)
            elif cmd == CMD_SET_JOINT_LIMITS:
                self._cmd_set_joint_limits(payload)
            elif cmd == CMD_SET_FF_FLAGS:
                if len(payload) < 4:
                    self._err(cmd, 0x01)
                else:
                    self.ff_mask = struct.unpack_from("<I", payload, 0)[0]
                    self._ack(cmd)
            elif cmd == CMD_SET_FF_VEC:
                self._cmd_set_ff_vec(payload)
            elif cmd == CMD_SET_FF_SCALAR:
                self._cmd_set_ff_scalar(payload)
            elif cmd == CMD_GET_FF_VEC:
                self._cmd_get_ff_vec(payload)
            elif cmd == CMD_GET_FF_SCALAR:
                self._cmd_get_ff_scalar(payload)
            elif cmd == CMD_FF_PRESET:
                if len(payload) < 1 or payload[0] > 2:
                    self._err(cmd, 0x02)
                else:
                    self.ff_mask = (0 if payload[0] == 0
                                    else 0x1F if payload[0] == 1
                                    else 0x1FF)
                    self._ack(cmd)
            elif cmd == CMD_GET_STATUS:
                self._send(RSP_STATUS, self._build_status(), droppable=False)
            elif cmd == CMD_GET_FIRMWARE:
                self._send(RSP_FIRMWARE, FW_VERSION.encode("ascii"))
            else:
                self._err(cmd, 0x00)    # 固件无此命令

    # ── 运动命令 ──

    def _require_enabled(self, cmd: int) -> bool:
        """固件对运动命令的统一门禁：未使能/急停锁存一律 ERR{cmd,0x03}。"""
        if not self.enabled or self.mode == MODE_EMERGENCY:
            self._err(cmd, 0x03)
            return False
        return True

    def _cmd_move_js(self, payload: bytes, now: float) -> None:
        n = self.n
        if len(payload) not in (8 * n, 12 * n):
            self._err(CMD_MOVE_JS, 0x01)
            return
        if not self._require_enabled(CMD_MOVE_JS):
            return
        q = list(struct.unpack_from(f"<{n}f", payload, 0))
        dq = list(struct.unpack_from(f"<{n}f", payload, 4 * n))
        tau = None
        if len(payload) == 12 * n:
            tau = list(struct.unpack_from(f"<{n}f", payload, 8 * n))
        # 带 tau_ff 就把内置前馈整段关掉 —— 这是固件最关键的一条语义。
        if self.s_js_user_ff and tau is None:
            self._reset_law()
        self.s_js_user_ff = tau is not None
        self.mode = MODE_MOVE_JS
        # ⚠ MOVE_JS 的语义与直觉不同（真机核实，control_loop.c:2106/2227）：
        #   v_lim = clamp(|dq_ref|, 0, speed_limit)   —— dq_ref **同时是位置参考的
        #                                               slew 速率上限**
        #   cmd->q_ref = slew_linear(target_q, cmd->q_ref, v_lim * dt)
        #   即参考每拍最多朝 target_q 走 v_lim·dt。
        #   ⇒ **发 dq_ref=0 时参考根本不动，臂不会动**。本字段就是那个 target_q，
        #     真正的参考在 self.q_ref，由 _step 里的 slew 推进。
        # 这条曾被漏掉：假固件原来直接用一阶滞后逼近 q_ref，于是"dq=0 也能动"，
        # 给了假阳性——真机上一跑就露馅。
        self.js_target_q = list(q)
        self.dq_ref = dq
        self.tau_user = tau or [0.0] * n
        self.last_kick_s = now
        self.move_js_log.append((list(q), list(dq),
                                 None if tau is None else list(tau)))

    def _cmd_move_mit_all(self, payload: bytes, now: float) -> None:
        n = self.n
        if len(payload) < 20 * n:
            self._err(CMD_MOVE_MIT_ALL, 0x01)
            return
        if not self._require_enabled(CMD_MOVE_MIT_ALL):
            return
        # SoA 读取（与固件一致）。按 AoS 打包时这里解出来的就是垃圾。
        self.q_ref = list(struct.unpack_from(f"<{n}f", payload, 0))
        self.dq_ref = list(struct.unpack_from(f"<{n}f", payload, 4 * n))
        self.kp_cmd = list(struct.unpack_from(f"<{n}f", payload, 8 * n))
        self.kd_cmd = list(struct.unpack_from(f"<{n}f", payload, 12 * n))
        self.tau_user = list(struct.unpack_from(f"<{n}f", payload, 16 * n))
        self.mode = MODE_MOVE_MIT_ALL
        self.s_js_user_ff = False
        self.last_kick_s = now

    def _cmd_move_j(self, payload: bytes, now: float) -> None:
        n = self.n
        if len(payload) < 4 * n + 4:
            self._err(CMD_MOVE_J, 0x01)
            return
        if not self._require_enabled(CMD_MOVE_J):
            return
        self.q_ref = list(struct.unpack_from(f"<{n}f", payload, 0))
        self.dq_ref = [0.0] * n
        self.mode = MODE_MOVE_J
        self.last_kick_s = now

    # ── 状态控制 ──

    def _cmd_enable(self, now: float) -> None:
        if self.mode == MODE_EMERGENCY:
            self._err(CMD_ENABLE, 0x06)     # 急停锁存，须 RESET
            return
        if not self.licensed:
            # license 门禁是单点的：未激活时只挡 ENABLE，其他命令都不受影响。
            self._err(CMD_ENABLE, 0x08)
            return
        if self.enabled:
            self._ack(CMD_ENABLE)           # 幂等：已使能时重发是 no-op
            return
        if not self._cmode_written:
            # **固件启动后的第一次 ENABLE**：先写 7 台电机的 CMODE（把它们切进
            # MIT 模式），这一刻回 0x03 并登记待使能。真机实测时序见
            # control_loop.c 的 ctrl_enable 注释：
            #   "ENABLE#1 -> 0x03(首写 CMODE), ENABLE#2 -> ACK"
            # ⚠ 这条是**必须在假固件里复现**的：它曾在真机上咬到过一次 ——
            #   主机若把 0x03 当失败，冷启动时每次都要白跑一轮重连。
            self._cmode_written = True
            self.enable_pending_at = now
            self._err(CMD_ENABLE, 0x03)
            return
        self.enable_pending_at = now
        self._ack(CMD_ENABLE)

    def _cmd_disable(self) -> None:
        self.enabled = False
        self.enable_pending_at = None
        self.err_code = [0] * self.n
        if self.mode != MODE_EMERGENCY:
            self.mode = MODE_INIT
        self._ack(CMD_DISABLE)

    def _cmd_emergency(self) -> None:
        self.enabled = False
        self.enable_pending_at = None
        self.mode = MODE_EMERGENCY
        self.err_code = [0] * self.n
        self._ack(CMD_EMERGENCY_STOP)

    def _cmd_clear_faults(self) -> None:
        # 逐轴：err==1（使能中）不碰。EMERGENCY 锁存不解除。
        for i in range(self.n):
            if self.err_code[i] != 1:
                self.err_code[i] = 1 if self.enabled else 0
        self.joint_fault = 0
        self.q_ref = list(self.q)
        self._ack(CMD_CLEAR_FAULTS)

    def _cmd_reset(self, now: float) -> None:
        """``ctrl_reset``：**全清并回 INIT/失能**（control_loop.c:1325-1353）。

        注意它把 ``mode`` 也一起打回 ``ARM_MODE_INIT``、``enabled=false``、
        调速器回 100%——不是"只清故障码"。急停锁存也靠它解除。
        """
        self.enabled = False
        self.enable_pending_at = None
        self._cmode_written = False      # 同固件 ctrl_reset
        self.mode = MODE_INIT
        self.joint_fault = 0
        self.watchdog_tripped = False
        self.park_requested = False
        self.s_js_user_ff = False
        self.speed_percent = 100
        self.last_kick_s = now
        self.err_code = [0] * self.n
        self.tau_user = [0.0] * self.n
        self.q_ref = list(self.q)
        self._ack(CMD_RESET)

    # ── 参数 ──

    def _cmd_get_joint_param(self, payload: bytes) -> None:
        if len(payload) < 1:
            self._err(CMD_GET_JOINT_PARAM, 0x01)
            return
        idx = payload[0]
        if idx >= self.n:
            self._err(CMD_GET_JOINT_PARAM, 0x02)
            return
        # RSP_JOINT_PARAM 载荷**不带 RSP 前缀**：idx + 5×f32 = 21B。
        body = bytes([idx]) + struct.pack(
            "<fffff", self.kp[idx], self.kd[idx], self.tau_max[idx],
            self.q_min[idx], self.q_max[idx])
        self._send(RSP_JOINT_PARAM, body)

    def _cmd_set_joint_param(self, payload: bytes) -> None:
        if len(payload) < 13:
            self._err(CMD_SET_JOINT_PARAM, 0x01)
            return
        idx = payload[0]
        if idx >= self.n:
            self._err(CMD_SET_JOINT_PARAM, 0x02)
            return
        self.kp[idx], self.kd[idx], self.tau_max[idx] = struct.unpack_from(
            "<fff", payload, 1)
        self._ack(CMD_SET_JOINT_PARAM)

    def _cmd_set_joint_limits(self, payload: bytes) -> None:
        if len(payload) < 9:
            self._err(CMD_SET_JOINT_LIMITS, 0x01)
            return
        idx = payload[0]
        if idx >= self.n:
            self._err(CMD_SET_JOINT_LIMITS, 0x02)
            return
        q_min, q_max = struct.unpack_from("<ff", payload, 1)
        # 固件只允许收窄：放宽的写入被拒。
        if q_min > self.q_min[idx] or q_max < self.q_max[idx]:
            self._err(CMD_SET_JOINT_LIMITS, 0x02)
            return
        self.q_min[idx], self.q_max[idx] = q_min, q_max
        self._ack(CMD_SET_JOINT_LIMITS)

    def _cmd_set_ff_vec(self, payload: bytes) -> None:
        if len(payload) < 29:
            self._err(CMD_SET_FF_VEC, 0x01)
            return
        item = payload[0]
        if item < 1 or item > 15:
            self._err(CMD_SET_FF_VEC, 0x02)
            return
        self.ff_vec[item] = list(struct.unpack_from(f"<{self.n}f", payload, 1))
        self._ack(CMD_SET_FF_VEC)

    def _cmd_set_ff_scalar(self, payload: bytes) -> None:
        if len(payload) < 6:
            self._err(CMD_SET_FF_SCALAR, 0x01)
            return
        item, sub = payload[0], payload[1]
        if item < 1 or item > 18 or item == 9:   # item 9 是只读（ff_mask）
            self._err(CMD_SET_FF_SCALAR, 0x02)
            return
        (value,) = struct.unpack_from("<f", payload, 2)
        self.ff_scalar[(item, sub)] = value
        self._ack(CMD_SET_FF_SCALAR)

    def _cmd_get_ff_vec(self, payload: bytes) -> None:
        if len(payload) < 1:
            self._err(CMD_GET_FF_VEC, 0x01)
            return
        item = payload[0]
        if item < 1 or item > 15:
            self._err(CMD_GET_FF_VEC, 0x02)
            return
        values = list(self.ff_vec.get(item, [0.0] * self.n))
        values = (values + [0.0] * 7)[:7]     # 固件应答恒为 7 宽
        # RSP_FF_VEC 载荷**带 RSP 前缀**：[0x4B, item, 7×f32] = 30B。
        self._send(RSP_FF_VEC, bytes([RSP_FF_VEC, item])
                   + struct.pack("<7f", *values))

    def _cmd_get_ff_scalar(self, payload: bytes) -> None:
        if len(payload) < 2:
            self._err(CMD_GET_FF_SCALAR, 0x01)
            return
        item, sub = payload[0], payload[1]
        if item < 1 or item > 18:
            self._err(CMD_GET_FF_SCALAR, 0x02)
            return
        if item == 9:
            value = float(self.ff_mask)
        else:
            value = self.ff_scalar.get((item, sub), 0.0)
        # RSP_FF_SCALAR 载荷**带 RSP 前缀**：[0x4C, item, sub, f32] = 7B。
        self._send(RSP_FF_SCALAR, bytes([RSP_FF_SCALAR, item, sub])
                   + struct.pack("<f", value))

    # ── 应答 ──

    def _ack(self, cmd: int) -> None:
        self._send(RSP_ACK, bytes([cmd]))

    def _err(self, cmd: int, reason: int) -> None:
        self._send(RSP_ERR, bytes([cmd, reason]))

    def _send(self, cmd: int, payload: bytes, droppable: bool = True) -> None:
        if self._master is None:
            return
        if droppable and self._drop_replies > 0:
            self._drop_replies -= 1
            return
        try:
            os.write(self._master, _frame(cmd, payload))
        except OSError:
            pass

    # ────────────────────────── 仿真步进 ──────────────────────────

    def _step(self, dt: float, now: float) -> None:
        with self._lock:
            # 使能两段：登记后过一小段才真正加磁（对应反馈齐了才 do_enable）。
            if (self.enable_pending_at is not None and not self.enabled
                    and now - self.enable_pending_at >= self.enable_delay_s):
                self.enabled = True
                self.enable_pending_at = None
                self.err_code = [1] * self.n
                self.mode = MODE_INIT
                self.last_kick_s = now

            # 命令看门狗：MOVE_J 每拍自动 kick，其余模式靠命令保活。
            if self.enabled and self.mode == MODE_MOVE_J:
                self.last_kick_s = now
            # 与 watchdog_check() 逐行同构：**两个分支都要写标志**。
            # 命令流恢复（每拍 kick）时标志必须被清掉，否则状态帧会一直谎报
            # fail-soft 态——固件那边就是因为漏了这个 else 分支被改过一版。
            if not self.enabled:
                self.watchdog_tripped = False
            elif now - self.last_kick_s > WATCHDOG_TIMEOUT_S:
                if not self.watchdog_tripped:
                    # [S1] 掉线保持的上升沿把 q_ref 改写成**实测位置**——
                    # 否则臂会被拉回"最后一条命令的终点"而不是停在原地。
                    self.q_ref = list(self.q)
                self.watchdog_tripped = True
            else:
                self.watchdog_tripped = False

            if not self.enabled:
                self.dq = [0.0] * self.n
                self.tau = [0.0] * self.n
                return

            # MOVE_JS：按 |dq_ref| 限速把参考朝命令目标推进一步（同固件 slew_linear）。
            if self.mode == MODE_MOVE_JS and not self.watchdog_tripped:
                for i in range(self.n):
                    limit = abs(self.dq_ref[i])
                    step = limit * dt
                    err = self.js_target_q[i] - self.q_ref[i]
                    if abs(err) <= step:
                        self.q_ref[i] = self.js_target_q[i]
                    elif step > 0.0:
                        self.q_ref[i] += math.copysign(step, err)

            alpha = min(1.0, dt / JOINT_LAG_S)
            for i in range(self.n):
                # 单轴故障只影响该轴（G7：joint_fault 位图 → 只 DISABLE 该轴），
                # 其余轴继续跟随。
                faulted = bool(self.joint_fault & (1 << i))
                holding = self.watchdog_tripped or faulted
                if holding:
                    # 持位：目标 = 冻结的 q_ref，tau=0，刚度按 park 声明缩放。
                    target = self.q_ref[i]
                    kp = self.kp[i] * (1.0 if self.park_requested else 0.6)
                else:
                    target = self.q_ref[i]
                    kp = self.kp_cmd[i] if self.mode == MODE_MOVE_MIT_ALL \
                        else self.kp[i]
                q_old = self.q[i]
                q_new = q_old + (target - q_old) * alpha
                self.q[i] = q_new
                self.dq[i] = (q_new - q_old) / dt if dt > 0 else 0.0
                # tau 只是"固件给电机的力矩"的替身。持位时恒 0（固件不叠前馈）。
                self.tau[i] = 0.0 if holding else kp * (target - q_new)
                if not holding and self.mode == MODE_MOVE_JS \
                        and self.s_js_user_ff:
                    self.tau[i] += self.tau_user[i]

    def _reset_law(self) -> None:
        """MOVE_JS 从"带用户 tau_ff"切回"不带"时，固件复位积分/摩擦历史。"""
        self.tau_user = [0.0] * self.n

    # ────────────────────────── 状态帧 ──────────────────────────

    def _safety_flags(self) -> int:
        flags = 0
        if self.joint_fault or self.mode == MODE_EMERGENCY:
            flags |= FLAG_FAULT
        if self.watchdog_tripped:
            flags |= FLAG_WATCHDOG_TRIPPED
        if self._inj_feedback_stale:
            flags |= FLAG_FEEDBACK_STALE
        if self._inj_temp_warning:
            flags |= FLAG_TEMP_WARNING
        if self._inj_pos_violation:
            flags |= FLAG_POSITION_VIOLATION
        if self._inj_overspeed:
            flags |= FLAG_OVERSPEED
        return flags

    def _build_status(self) -> bytes:
        """手工拼 ``6 + 21N`` 状态帧（**不复用 stm32_proto**，见模块 docstring）。"""
        flags = (self._safety_flags() & 0x3F) | (self.mode << 6)
        if self.enabled:
            flags |= 1 << 9
        out = bytearray(struct.pack("<HH", flags, self.seq & 0xFFFF))
        for i in range(self.n):
            out += struct.pack("<fffff", self.q[i], self.dq[i], self.tau[i],
                               self.t_mos[i], self.t_coil[i])
            out += bytes([self.err_code[i] & 0xFF])
        out += struct.pack("<H", self.joint_fault & 0xFFFF)
        self.seq = (self.seq + 1) & 0xFFFF
        return bytes(out)
