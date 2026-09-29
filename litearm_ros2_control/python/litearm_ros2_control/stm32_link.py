#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm-stm32 主机侧链路层：串口打开、非阻塞收发、请求/应答。

职责与边界
----------
本模块是 :mod:`litearm_ros2_control.stm32_proto` 的**传输与状态缓存**外壳：
谁发什么命令、什么时候发，是 :mod:`litearm_ros2_control.hw_daemon` 的事。

为什么不用 pyserial
-------------------
``/dev/ttyACM0`` 是一个普通字符设备：``os.open`` + 原始 termios + ``select``
就是全部所需，CDC 也忽略波特率。用 stdlib 有三点实际好处：

1. 本环境（以及任何只有 ROS 基础依赖的机器）**没有 pyserial**，
   而 ``hw_daemon`` 是 ``ros2 launch`` 拉起的、不该因为一个串口库缺失而失败。
2. pty 上的假固件（``fake_firmware.py``）与真 ``/dev/ttyACM0`` 走**同一条**
   代码路径，无硬件演练才真的验到了生产路径。
3. 少一个 rosdep 依赖。

非阻塞纪律
----------
固件的 TX 是"忙则丢"（``usb_cmd.c`` 的 ``usb_send_raw``），所以**状态帧、
ACK、ERR 都可能丢**。据此定下两条规矩：

* 控制环里**绝不阻塞等应答**——:meth:`Stm32Link.send` 只写字节；
* 启动期的配置查询才用 :meth:`Stm32Link.request`（短超时 + 允许重试），
  且超时返回 ``None`` 而不抛异常，由调用方决定重试几次。

DTR/RTS 顺手拉高：真 USB CDC 设备有的要看到 DTR 才发数据。pty 不支持这个
ioctl，失败即忽略。
"""

import errno
import fcntl
import glob
import logging
import os
import select
import struct
import termios
import time
from typing import Dict, List, Optional, Sequence, Tuple

from litearm_ros2_control import stm32_proto as proto
from litearm_ros2_control.stm32_proto import Frame, StatusFrame

log = logging.getLogger("litearm.stm32_link")

# 板载 USB CDC 的 VID:PID（tools/ 里的 arm_audit/arm_console/_proto 同源）。
DEFAULT_VID = 0x1D50
DEFAULT_PID = 0x606F

# 非状态帧应答的缓存深度。启动期配置查询最多 7 关节 × 数条命令，
# 32 足够放下"一口气发完再逐条收"的场景。
REPLY_BACKLOG = 32

# 一次 poll 最多读多少字节。153B 状态帧 @100Hz ≈ 15KB/s，
# 单次 4KB 已远超一个控制周期的来量。
READ_CHUNK = 4096


class Stm32Error(RuntimeError):
    """链路层错误基类。"""


class Stm32NotConnected(Stm32Error):
    """端口未打开或已掉线。"""


class Stm32AccessDenied(Stm32NotConnected):
    """端口存在但没有权限打开。

    单独一个类型，是因为**它不会自己好**：等下去、重试都不会改变结果，
    必须人去加组（`usermod -aG dialout $USER`）后重新登录。把它和"板子还没插上"
    （插上就好）混在一个无限重试里，只会让日志刷满同一句没用的话。
    """

    REMEDY = ("串口需要 dialout 组权限：\n"
              "    sudo usermod -aG dialout $USER\n"
              "  然后**注销重新登录**（组成员身份在登录时生效）。\n"
              "  临时验证（拔插后失效）：sudo chmod 666 <设备路径>")


class Stm32CmdError(Stm32Error):
    """固件回了 ``RSP_ERR``——命令被拒，携带 (命令号, 原因码)。"""

    def __init__(self, cmd: int, reason: int) -> None:
        self.cmd = cmd
        self.reason = reason
        super().__init__(
            f"固件拒绝命令 0x{cmd:02X}：ERR{{{cmd:#04x},{reason:#04x}}}"
            f"（{ERR_HINTS.get(reason, '未收录的原因码')}）")


# 原因码速查（usb_cmd.h / usb_cmd.c 各分支）。用于把 ERR 翻成人能读的话。
ERR_HINTS = {
    0x00: "固件无此命令（usb_cmd.c 的 dispatch default 分支）",
    0x01: "载荷长度不足",
    0x02: "参数非法或越界",
    0x03: "未使能，或急停锁存中",
    0x04: "须先失能（使能中/使能在途）",
    0x05: "flash 保存进行中，稍后重试",
    0x06: "掉线刚性持位锁存中",
    0x07: "掩码不符（0x32 模型提交）",
    0x08: "license 未激活",
}


def rejected_text(counts: Dict[Tuple[int, int], int]) -> str:
    """把 ``{(命令号, 原因码): 次数}`` 打成一行摘要；空计数返回空串。

    控制环路径的拒帧原来**无处可看**（见 :attr:`Stm32Link.err_by_code`），
    这个函数是那批计数唯一的出口 —— :meth:`Stm32Link.counters` 与守护进程的
    周期上报都用它。按次数降序，等久了也能一眼看出是哪一条在反复被拒。
    """
    if not counts:
        return ""
    items = sorted(counts.items(), key=lambda kv: -kv[1])
    body = ", ".join(f"{cmd:#04x}/{reason:#04x}×{count}"
                     f"({ERR_HINTS.get(reason, '未收录的原因码')})"
                     for (cmd, reason), count in items)
    return f" 被拒[{body}]"


def find_port(vendor: int = DEFAULT_VID,
              product: int = DEFAULT_PID) -> Optional[str]:
    """按 USB VID:PID 扫 ``/sys/class/tty`` 找 tty 设备节点。

    pyserial 的 ``list_ports`` 在 Linux 上也是走这套 sysfs。找不到返回 ``None``，
    调用方应提示用 ``--port`` 显式指定。
    """
    for entry in sorted(glob.glob("/sys/class/tty/*")):
        name = os.path.basename(entry)
        if not name.startswith(("ttyACM", "ttyUSB")):
            continue
        node = os.path.realpath(os.path.join(entry, "device"))
        for _ in range(6):  # 从 tty 设备向上找 USB 接口/配置目录
            vid_path = os.path.join(node, "idVendor")
            pid_path = os.path.join(node, "idProduct")
            if os.path.exists(vid_path) and os.path.exists(pid_path):
                try:
                    with open(vid_path, encoding="ascii") as handle:
                        vid = int(handle.read().strip(), 16)
                    with open(pid_path, encoding="ascii") as handle:
                        pid = int(handle.read().strip(), 16)
                except (OSError, ValueError):
                    break
                if (vid, pid) == (vendor, product):
                    return f"/dev/{name}"
                break
            parent = os.path.dirname(node)
            if parent == node:
                break
            node = parent
    return None


def _set_raw(fd: int) -> None:
    """把 fd 设为原始模式（无回显/无行规程/无流控）。

    波特率**不动**：CDC 忽略它，pty 也没有这个概念，而
    ``termios.tcsetattr`` 传一个设备不支持的速率反而会失败。
    """
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag = attrs[0], attrs[1], attrs[2], attrs[3]
    iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
               | termios.ISTRIP | termios.INLCR | termios.IGNCR
               | termios.ICRNL | termios.IXON | termios.IXOFF | termios.IXANY)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON
               | termios.ISIG | termios.IEXTEN)
    cflag &= ~(termios.CSIZE | termios.PARENB | termios.CSTOPB)
    cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[0], attrs[1], attrs[2], attrs[3] = iflag, oflag, cflag, lflag
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def _raise_dtr(fd: int) -> None:
    """尽力拉高 DTR/RTS。pty 上这个 ioctl 会失败，忽略即可。"""
    try:
        fcntl.ioctl(fd, termios.TIOCMBIS,
                    struct.pack("I", termios.TIOCM_DTR | termios.TIOCM_RTS))
    except OSError:
        pass


class Stm32Link:
    """一条到 litearm-stm32 固件的 CDC 链路。

    所有命令都在**同一根线**上复用：状态帧（100Hz 主动上报）与命令应答
    交错到达，由 :class:`~litearm_ros2_control.stm32_proto.CdcFramer` 切分，
    :meth:`poll` 负责把状态帧更新到 :attr:`status`、把应答放进待取队列。
    """

    def __init__(self, port: Optional[str] = None, *,
                 read_timeout_s: float = 0.0) -> None:
        self.port = port
        self.read_timeout_s = float(read_timeout_s)
        self._fd: Optional[int] = None
        self._framer = proto.CdcFramer()
        self._replies: List[Frame] = []
        self._status: Optional[StatusFrame] = None
        self._status_count = 0
        # 诊断计数（守护进程周期性地报出去；TX 丢帧是本链路的常态而非异常）。
        self.tx_frames = 0
        self.tx_bytes = 0
        # ★ 2026-09-29：**整帧被丢掉**的次数（内核缓冲写不进、且超过重试窗口）。
        #   与 `errors` 分开计：errors 是异常，这个是"链路拥塞的正常代价"。
        #   ⚠ 但它高得离谱时**不是小事**：命令送不到固件 ⇒ 固件的 100 ms 命令看门狗
        #   反复接管 ⇒ 臂在 fail-soft 持位与跟踪之间来回切（现场 = 启动后/负载高时
        #   臂来回摇晃），而主机侧原来**一点痕迹都没有**。见 send() 与 hw_daemon
        #   的 `_report_link_counters`。
        self.tx_dropped = 0
        self.rx_frames = 0
        self.crc_errors = 0
        self.errors = 0
        # ★ 2026-09-29：**被固件拒绝的帧**，按 ``(命令号, 原因码)`` 计数。
        #   控制环路径走 `send()`（不等应答），`RSP_ERR` 只会躺进 `_replies` 并被
        #   `REPLY_BACKLOG` 封顶丢掉 ⇒ "命令被拒"在本进程原来**零痕迹**，与
        #   `tx_dropped` 是同一类缺陷。典型 = `(0x03, 0x02)`：MOVE_JS 因
        #   "dq 全 0 且目标离实测 > 5 mrad" 被整条拒绝（固件 2026-09-24 门禁），
        #   拒绝发生在 `watchdog_kick()` 之前 ⇒ 不喂看门狗 ⇒ 100 ms 后 fail-soft。
        self.err_by_code: Dict[Tuple[int, int], int] = {}

    # ────────────────────────── 生命周期 ──────────────────────────

    @property
    def is_open(self) -> bool:
        return self._fd is not None

    def open(self, port: Optional[str] = None) -> str:
        """打开端口并进入原始模式，返回实际使用的设备路径。"""
        if self._fd is not None:
            return self.port or ""
        path = port or self.port or find_port()
        if not path:
            raise Stm32NotConnected(
                f"未找到 litearm-stm32 的 USB CDC 设备"
                f"（VID:PID {DEFAULT_VID:04x}:{DEFAULT_PID:04x}）；"
                f"用 --port 显式指定，或确认板子已上电、USB 已连、"
                f"且未被别的进程占用")
        flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        try:
            self._fd = os.open(path, flags)
        except PermissionError as exc:
            raise Stm32AccessDenied(
                f"打不开 {path}：权限不足（{exc.strerror}）。\n"
                f"  {Stm32AccessDenied.REMEDY}") from exc
        try:
            _set_raw(self._fd)
            _raise_dtr(self._fd)
        except BaseException:
            os.close(self._fd)
            self._fd = None
            raise
        self.port = path
        self._framer.reset()
        self._replies.clear()
        log.info("已打开 %s", path)
        return path

    def close(self) -> None:
        if self._fd is not None:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = None
            log.info("已关闭 %s", self.port)

    # ────────────────────────── 收发 ──────────────────────────

    def _read_available(self) -> bytes:
        """非阻塞地取走当前可读的全部字节（一次 read 拿全部，避免 syscall 抖动）。"""
        assert self._fd is not None
        try:
            return os.read(self._fd, READ_CHUNK)
        except BlockingIOError:
            return b""
        except OSError as exc:
            if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                return b""
            # USB 拔掉 -> EIO/ENXIO/ENODEV；标记掉线并让上层重连。
            self.errors += 1
            self.close()
            raise Stm32NotConnected(f"{self.port} 读取失败：{exc}") from exc

    def poll(self) -> int:
        """读走所有可用字节并分发，返回本次处理的完整帧数（含 CRC 错的）。

        非阻塞：没有数据就立刻返回 0，绝不等待。控制环每周期调一次。
        """
        if self._fd is None:
            return 0
        handled = 0
        while True:
            chunk = self._read_available()
            if not chunk:
                break
            for frame in self._framer.feed(chunk):
                handled += 1
                self.rx_frames += 1
                if not frame.crc_ok:
                    self.crc_errors += 1
                    continue
                if frame.cmd == proto.RSP_STATUS:
                    status = proto.decode_status(frame.payload)
                    if status is not None:
                        status.stamp_s = time.monotonic()
                        self._status = status
                        self._status_count += 1
                else:
                    if frame.cmd == proto.RSP_ERR:
                        # 先记账再入队：控制环路径没人消费 `_replies`（`send()` 不等
                        # 应答），队列满了就从队头丢 ⇒ 只靠 `_replies` 是看不见拒帧的。
                        decoded = proto.decode_err(frame.payload)
                        if decoded is not None:
                            self.err_by_code[decoded] = \
                                self.err_by_code.get(decoded, 0) + 1
                    self._replies.append(frame)
                    if len(self._replies) > REPLY_BACKLOG:
                        del self._replies[0]
        return handled

    # 发送时如果内核缓冲暂时写不进，允许等多久（秒）。
    # **必须远小于控制周期**（250 Hz ⇒ 4 ms）：宁可丢掉这一帧（4 ms 后就有新帧，
    # 固件的 100 ms 看门狗照样喂得上），也不要为了保住它把控制周期拖长。
    TX_RETRY_WINDOW_S = 0.003

    def send(self, cmd: int, payload: bytes = b"") -> None:
        """发一帧。**不等待应答**——控制环路径专用。

        ★ 2026-09-29 修：原实现**第一次 `EAGAIN` 就丢掉这一帧**（只 `errors += 1`
        后 return，而 `tx_frames` 不涨 ⇒ 看不出丢了）。这条链路的 OUT 方向常年被
        固件的状态流（153 B @ 250 Hz ≈ 38 kB/s）挤着，`EAGAIN` 非常频繁 ⇒ 真正
        送到的命令帧只剩少数（真机实测：固件"接受"的命令间隔经常 >100 ms）⇒
        **固件的 100 ms 命令看门狗反复接管** ⇒ 臂在 fail-soft 持位与跟踪之间来回切
        （现象 = 启动后/负载高时臂来回摇晃），而主机侧完全没有痕迹。

        现在：在 ``TX_RETRY_WINDOW_S`` 内用 ``select`` 等"可写"再写剩余部分（**背压感知**：
        等的是缓冲排空，而不是忙等/硬灌）—— 只要内核在这几毫秒里腾出了地方，这一帧就
        仍然送到。窗口内写不完才**整帧**丢弃（半截帧才危险；本实现不会写半帧）。
        """
        if self._fd is None:
            raise Stm32NotConnected("端口未打开")
        data = proto.pack_frame(cmd, payload)
        sent = 0
        deadline = time.monotonic() + self.TX_RETRY_WINDOW_S
        while sent < len(data):
            try:
                sent += os.write(self._fd, data[sent:])
                continue
            except BlockingIOError:
                pass          # 缓冲满：等一小会儿再写剩下的
            except OSError as exc:
                self.errors += 1
                self.close()
                raise Stm32NotConnected(f"{self.port} 写入失败：{exc}") from exc
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                self.tx_dropped += 1      # 整帧丢弃（不是半帧）
                return
            select.select([], [self._fd], [], remaining)
        self.tx_frames += 1
        self.tx_bytes += len(data)

    def request(self, cmd: int, payload: bytes = b"",
                expect: Sequence[int] = (), timeout_s: float = 0.3,
                accept_err: bool = False) -> Optional[Frame]:
        """发一帧并等期望的应答。超时返回 ``None``（**不抛异常**）。

        只用于启动期/配置查询。控制环里一律用 :meth:`send`。

        ``expect`` 给出可接受的上行命令号；命中后从队列里摘掉并返回。
        ``RSP_ERR`` 默认也算命中（返回该帧），由调用方用
        :func:`~litearm_ros2_control.stm32_proto.decode_err` 判读；
        ``accept_err=False`` 时改为抛 :class:`Stm32CmdError`。
        """
        if self._fd is None:
            raise Stm32NotConnected("端口未打开")
        expect = tuple(expect)
        # 先把积压清空再发：否则上一次用 send() 发出去、ERR 拖到现在才到的帧，
        # 会被这次请求当成"本命令被拒"——那是彻底的误判（例如把一条陈旧 ERR
        # 当成 license 未激活而硬失败）。
        self.poll()
        self._replies.clear()
        self.send(cmd, payload)
        deadline = time.monotonic() + float(timeout_s)
        while True:
            self.poll()
            for index, frame in enumerate(self._replies):
                if frame.cmd in expect:
                    del self._replies[index]
                    return frame
                if frame.cmd == proto.RSP_ERR:
                    decoded = proto.decode_err(frame.payload)
                    if decoded is not None and decoded[0] == cmd:
                        del self._replies[index]
                        if accept_err:
                            return frame
                        raise Stm32CmdError(*decoded)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return None
            select.select([self._fd], [], [], remaining)

    def command(self, cmd: int, payload: bytes = b"",
                timeout_s: float = 0.3) -> Tuple[Optional[bool], Optional[int]]:
        """发命令并等 ``ACK``/``ERR``，返回 ``(是否成功, 原因码)``。

        超时返回 ``(None, None)``——**这是常态而非异常**：固件的 TX 是"忙则丢"，
        应答可能根本发不出来。调用方按 `"没收到应答"` 决定重试，不要当成失败。

        需要这个（而不是裸 :meth:`send`）的场合只有一个：**要区分失败原因**。
        典型是 ``ENABLE``：``ERR{0x10,0x08}`` = license 未激活（重试无用，硬失败），
        ``ERR{0x10,0x03}`` = 反馈尚未就绪（重试即可）。
        """
        frame = self.request(cmd, payload, expect=(proto.RSP_ACK,),
                             timeout_s=timeout_s, accept_err=True)
        if frame is None:
            return None, None
        if frame.cmd == proto.RSP_ACK:
            return True, None
        decoded = proto.decode_err(frame.payload)
        return False, (None if decoded is None else decoded[1])

    # ────────────────────────── 状态 ──────────────────────────

    @property
    def status(self) -> Optional[StatusFrame]:
        """最近一帧状态（100Hz 主动上报，无需请求）。"""
        return self._status

    @property
    def status_count(self) -> int:
        return self._status_count

    def status_age_s(self) -> float:
        """最近状态帧的本地龄（秒）。从未收到过返回 ``inf``。"""
        if self._status is None:
            return float("inf")
        return time.monotonic() - self._status.stamp_s

    def wait_status(self, timeout_s: float = 2.0) -> Optional[StatusFrame]:
        """等到一帧状态或超时。启动期用来确认固件在线并取得关节数。"""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll()
            if self._status is not None:
                return self._status
            select.select([self._fd], [], [], 0.01)
        return self._status

    def wait_enabled(self, timeout_s: float = 3.0) -> Optional[StatusFrame]:
        """等到 ``flags.enabled`` 为真或超时。

        固件的 ``ENABLE`` 有两段：先写 CMODE（那一刻回 ``0x03``），反馈齐了才
        真正加磁（``control_loop.c`` 的 ``enable_pending_poll``）。所以**不能**
        用 ACK 判断使能成功，必须看状态帧的 enabled 位。
        """
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.poll()
            if self._status is not None and self._status.enabled:
                return self._status
            select.select([self._fd], [], [], 0.01)
        return self._status if (self._status and self._status.enabled) else None

    # ─────────────────────── 命令封装 ───────────────────────
    # 纯 send_* 的绝不等待；get_* 的才等应答。

    def get_firmware(self, timeout_s: float = 0.5) -> Optional[str]:
        frame = self.request(proto.CMD_GET_FIRMWARE,
                             expect=(proto.RSP_FIRMWARE,), timeout_s=timeout_s)
        return None if frame is None else proto.decode_firmware(frame.payload)

    def get_joint_param(self, idx: int, timeout_s: float = 0.5
                        ) -> Optional[proto.JointParam]:
        frame = self.request(proto.CMD_GET_JOINT_PARAM,
                             proto.pack_get_joint_param(idx),
                             expect=(proto.RSP_JOINT_PARAM,),
                             timeout_s=timeout_s)
        return None if frame is None else proto.decode_joint_param(frame.payload)

    def get_ff_scalar(self, item: int, sub: int = 0, timeout_s: float = 0.5
                      ) -> Optional[float]:
        frame = self.request(proto.CMD_GET_FF_SCALAR,
                             proto.pack_get_ff_scalar(item, sub),
                             expect=(proto.RSP_FF_SCALAR,),
                             timeout_s=timeout_s)
        if frame is None:
            return None
        decoded = proto.decode_ff_scalar(frame.payload)
        return None if decoded is None else decoded[2]

    def get_ff_vec(self, item: int, timeout_s: float = 0.5
                   ) -> Optional[Tuple[float, ...]]:
        frame = self.request(proto.CMD_GET_FF_VEC,
                             proto.pack_get_ff_vec(item),
                             expect=(proto.RSP_FF_VEC,),
                             timeout_s=timeout_s)
        if frame is None:
            return None
        decoded = proto.decode_ff_vec(frame.payload)
        return None if decoded is None else decoded[1]

    def get_ff_mask(self, timeout_s: float = 0.5) -> Optional[int]:
        value = self.get_ff_scalar(proto.FF_SCALAR_FF_MASK, 0, timeout_s)
        return None if value is None else int(round(value))

    def enable(self) -> None:
        """请求使能。**ACK 只表示"登记"**——真正加磁要等反馈齐，见 :meth:`wait_enabled`。"""
        self.send(proto.CMD_ENABLE)

    def disable(self) -> None:
        self.send(proto.CMD_DISABLE)

    def emergency_stop(self) -> None:
        """急停：失能 ×5 并锁存。**臂会失力下坠**，非硬件急停勿轻用。"""
        self.send(proto.CMD_EMERGENCY_STOP)

    def clear_faults(self) -> None:
        self.send(proto.CMD_CLEAR_FAULTS)

    def reset(self) -> None:
        self.send(proto.CMD_RESET)

    def park(self) -> None:
        """``SET_MOTION_MODE(0x20)`` 传 0 = 声明 park。

        使能中置位后，命令停发（看门狗 100ms 触发）时固件按 **1.0× 刚度**持位，
        而不是 fail-soft 的 0.6×。这就是"PC 断开前的高刚度 park 声明"。
        ⚠ 刻意不 kick 看门狗：park 之后停止发帧，看门狗**必然**触发——这正是设计。
        """
        self.send(proto.CMD_SET_MOTION_MODE, proto.pack_u8(0))

    def set_speed_percent(self, percent: int) -> None:
        """``0x21``：调速器 0~100，只影响后续 move_j/move_js 的速度上限。"""
        self.send(proto.CMD_SET_SPEED_PERCENT, proto.pack_u8(percent))

    def set_ff_mask(self, mask: int) -> None:
        self.send(proto.CMD_SET_FF_FLAGS, proto.pack_set_ff_flags(mask))

    def set_ff_vec(self, item: int, values: Sequence[float]) -> None:
        self.send(proto.CMD_SET_FF_VEC, proto.pack_set_ff_vec(item, values))

    def set_ff_scalar(self, item: int, value: float, sub: int = 0) -> None:
        self.send(proto.CMD_SET_FF_SCALAR,
                  proto.pack_set_ff_scalar(item, sub, value))

    def move_js(self, q: Sequence[float], dq: Sequence[float],
                tau: Optional[Sequence[float]] = None) -> None:
        """流式关节伺服。**每周期都要重发**（不自动 kick 看门狗）。

        ``tau=None`` → 固件叠加内置前馈；给了 tau（哪怕全 0）→ 内置前馈整段关闭。
        """
        self.send(proto.CMD_MOVE_JS, proto.pack_move_js(q, dq, tau))

    def move_mit_all(self, rows: Sequence[Sequence[float]]) -> None:
        """全臂 MIT 透传。**每周期都要重发**；固件不叠加任何自家前馈。"""
        self.send(proto.CMD_MOVE_MIT_ALL, proto.pack_move_mit_all(rows))

    # ────────────────────────── 诊断 ──────────────────────────

    def counters(self) -> str:
        return (f"tx={self.tx_frames}帧 rx={self.rx_frames}帧 "
                f"crc_err={self.crc_errors} 丢弃={self._framer.discarded}B "
                f"其他错误={self.errors}" + rejected_text(self.err_by_code))
