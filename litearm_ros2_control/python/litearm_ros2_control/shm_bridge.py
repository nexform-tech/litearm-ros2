#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共享内存契约的 Python 侧绑定（ctypes）。

契约本身定义在 ``include/litearm_ros2_control/litearm_shm.h``——本模块只是它的
消费者，不重新定义布局语义：

* 结构体字段顺序/宽度与 C 侧逐字节一致，由 :func:`describe_layout` 与 C 侧探针
  （``litearm_shm_layout_probe``）交叉校验（见 ``test/test_shm_layout.py``）。
* seqlock 的 acquire/release 语义完全在 C 侧实现，Python 这边不做任何裸内存访问，
  只做“整块结构体 + 长度”的进出，因此不需要在 Python 里表达内存序。

时间戳约定：``stamp_s`` / ``heartbeat_s`` 均为 ``CLOCK_MONOTONIC`` 秒，
Linux 下与 :func:`time.monotonic` 同源，可与 C++ ``steady_clock`` 直接比较。
"""

import ctypes
import ctypes.util
import os
from pathlib import Path
from typing import List, Optional

NUM_JOINTS = 7
"""轴数，与 C 侧 LITEARM_SHM_NUM_JOINTS 一致。"""

DEFAULT_SHM_NAME = "/litearm_hw"
"""默认共享内存对象名。"""

SHM_OK = 0
SHM_TORN = 1
SHM_ERR_OPEN = -2
SHM_ERR_LAYOUT = -5
SHM_ERR_VERSION = -6

# ── LitearmState.last_error：守护进程抑制原因码 ──
# 必须与 include/litearm_ros2_control/litearm_shm.h 的 LITEARM_DAEMON_* 保持一致。
DAEMON_OK = 0
DAEMON_CONNECTING = 1
DAEMON_HOLDING_STALE_COMMAND = 2
DAEMON_HOLDING_ESTOP = 3
DAEMON_HOLDING_MOTOR_FAULT = 4
DAEMON_HOLDING_FEEDBACK_STALE = 5
DAEMON_HOLDING_OVERTEMP = 6
DAEMON_DISABLED = 7
DAEMON_HOLDING_WATCHDOG = 8
DAEMON_SHUTTING_DOWN = 9
DAEMON_HOLDING_BAD_COMMAND = 10

DAEMON_STATUS_TEXT = {
    DAEMON_OK: "跟踪 ros2_control 命令",
    DAEMON_CONNECTING: "尚未连接硬件（启动中、端口没找到或 license 未激活）",
    DAEMON_HOLDING_STALE_COMMAND: "命令帧陈旧：ROS 侧控制环已停止发布",
    DAEMON_HOLDING_ESTOP: "软急停中",
    DAEMON_HOLDING_MOTOR_FAULT: "存在非健康码关节",
    DAEMON_HOLDING_FEEDBACK_STALE: "关节反馈缺失或超时",
    DAEMON_HOLDING_OVERTEMP: "电机温度达到软件保护阈值",
    DAEMON_DISABLED: "ROS 侧请求失能（电机失力）",
    DAEMON_HOLDING_WATCHDOG: "litearm-stm32 固件看门狗曾接管",
    DAEMON_SHUTTING_DOWN: "守护进程退出中",
    DAEMON_HOLDING_BAD_COMMAND: "命令帧含非有限数，已拒绝",
}

_ERR_TEXT = {
    SHM_OK: "ok",
    SHM_TORN: "读取撕裂（写者正在更新，重试耗尽）",
    SHM_ERR_OPEN: "无法打开共享内存段",
    -1: "通用错误",
    -3: "ftruncate 失败",
    -4: "mmap 失败",
    SHM_ERR_LAYOUT: "共享内存段不存在或大小不符",
    SHM_ERR_VERSION: "共享内存布局版本不匹配",
    -7: "非法参数",
}


class ShmError(RuntimeError):
    """共享内存操作失败。"""

    def __init__(self, message: str, code: int = 0) -> None:
        super().__init__(f"{message}（code={code}）")
        self.code = code


# ─────────────────────────── 结构体镜像 ───────────────────────────
# 字段顺序必须与 litearm_shm.h 完全一致；全部为 double，
# 因此不存在隐式 padding，跨语言布局无歧义。

StateFields = [
    ("position", ctypes.c_double * NUM_JOINTS),
    ("velocity", ctypes.c_double * NUM_JOINTS),
    ("effort", ctypes.c_double * NUM_JOINTS),
    ("temperature_mos", ctypes.c_double * NUM_JOINTS),
    ("temperature_coil", ctypes.c_double * NUM_JOINTS),
    ("error_code", ctypes.c_double * NUM_JOINTS),
    ("feedback_age_s", ctypes.c_double * NUM_JOINTS),
    ("feedback_received", ctypes.c_double * NUM_JOINTS),
    ("stamp_s", ctypes.c_double),
    ("heartbeat_s", ctypes.c_double),
    ("connected", ctypes.c_double),
    ("enabled", ctypes.c_double),
    ("faulted", ctypes.c_double),
    ("watchdog_tripped", ctypes.c_double),
    ("dry_run", ctypes.c_double),
    ("cycle_count", ctypes.c_double),
    ("applied_command_cycle", ctypes.c_double),
    ("command_age_s", ctypes.c_double),
    ("last_error", ctypes.c_double),
]

CommandFields = [
    ("position", ctypes.c_double * NUM_JOINTS),
    ("velocity", ctypes.c_double * NUM_JOINTS),
    ("acceleration", ctypes.c_double * NUM_JOINTS),
    ("effort", ctypes.c_double * NUM_JOINTS),
    ("kp", ctypes.c_double * NUM_JOINTS),
    ("kd", ctypes.c_double * NUM_JOINTS),
    ("enable", ctypes.c_double),
    ("estop", ctypes.c_double),
    ("stamp_s", ctypes.c_double),
    ("cycle_count", ctypes.c_double),
]

HeaderFields = [
    ("magic", ctypes.c_uint32),
    ("layout_version", ctypes.c_uint32),
    ("state_seq", ctypes.c_uint64),
    ("command_seq", ctypes.c_uint64),
    ("state_publish_count", ctypes.c_uint64),
    ("command_publish_count", ctypes.c_uint64),
    ("state_torn_reads", ctypes.c_uint64),
    ("command_torn_reads", ctypes.c_uint64),
]


class LitearmState(ctypes.Structure):
    """守护进程 → ROS 的关节状态与健康度。字段语义见 litearm_shm.h。"""

    _fields_ = StateFields

    def joints(self) -> "dict[str, List[float]]":
        """按关节展开，便于打印/断言。"""
        return {
            name: [getattr(self, name)[i] for i in range(NUM_JOINTS)]
            for name, _ in StateFields[:8]
        }


class LitearmCommand(ctypes.Structure):
    """ROS → 守护进程的 MIT 命令。字段语义见 litearm_shm.h。"""

    _fields_ = CommandFields

    @classmethod
    def holding(
        cls,
        position: List[float],
        kp: List[float],
        kd: List[float],
        stamp_s: float = 0.0,
        cycle_count: float = 0.0,
        enable: float = 1.0,
    ) -> "LitearmCommand":
        """构造一帧“原位高刚度持位”命令（dq_ref=0，tau_ff=0）。"""
        command = cls()
        for index, value in enumerate(position):
            command.position[index] = float(value)
        for index in range(NUM_JOINTS):
            command.kp[index] = float(kp[index])
            command.kd[index] = float(kd[index])
        command.enable = float(enable)
        command.estop = 0.0
        command.stamp_s = float(stamp_s)
        command.cycle_count = float(cycle_count)
        return command


class LitearmHeader(ctypes.Structure):
    """段头部诊断信息。"""

    _fields_ = HeaderFields


# ─────────────────────────── 动态库定位与加载 ───────────────────────────


def _candidate_lib_paths() -> List[Path]:
    """按优先级列出 liblitearm_shm.so 的候选路径。

    两种运行形态都要覆盖：

    * ROS 安装树：``<prefix>/lib/python3.10/site-packages/litearm_ros2_control/``
      → 上溯 3 层得到 ``<prefix>/lib/``。
    * 源码树直接跑（跑单测时）：``<ws>/src/litearm_ros2_control/python/...``
      → 去 ``<ws>/build/litearm_ros2_control/`` 找 colcon 的构建产物。
    """
    candidates: List[Path] = []

    override = os.environ.get("LITEARM_SHM_LIB")
    if override:
        candidates.append(Path(override))

    here = Path(__file__).resolve()
    package_root = here.parents[2]          # .../litearm_ros2_control
    workspace_root = package_root.parents[1]  # .../<ws>

    # 1) ROS 安装树
    candidates.append(here.parents[3] / "liblitearm_shm.so")
    # 2) colcon 构建树
    candidates.append(workspace_root / "build" / package_root.name /
                      "liblitearm_shm.so")
    # 3) colcon 安装树（未 source 但构建过）
    candidates.append(workspace_root / "install" / package_root.name / "lib" /
                      "liblitearm_shm.so")
    # 4) 任何 AMENT_PREFIX_PATH 前缀
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if prefix:
            candidates.append(Path(prefix) / "lib" / "liblitearm_shm.so")
    # 5) 交给动态链接器
    found = ctypes.util.find_library("litearm_shm")
    if found:
        candidates.append(Path(found))

    return candidates


def _load_library() -> ctypes.CDLL:
    """加载 liblitearm_shm.so，并在加载时校验布局大小（防止新旧库错配）。"""
    errors: List[str] = []
    for path in _candidate_lib_paths():
        if not path.exists():
            errors.append(f"{path}（不存在）")
            continue
        try:
            lib = ctypes.CDLL(str(path), use_errno=True)
        except OSError as exc:  # pragma: no cover - 环境相关
            errors.append(f"{path}（{exc}）")
            continue
        _declare_signatures(lib)
        _verify_sizes(lib, path)
        return lib
    raise ShmError(
        "无法定位 liblitearm_shm.so，请先 colcon build 并 source 安装空间，"
        "或设置 LITEARM_SHM_LIB 环境变量。已尝试:\n  " + "\n  ".join(errors)
    )


def _declare_signatures(lib: ctypes.CDLL) -> None:
    """声明 C API 签名（无声明时 ctypes 会按 int 截断 64 位返回值）。"""
    size_t = ctypes.c_size_t
    lib.litearm_shm_state_size.restype = size_t
    lib.litearm_shm_command_size.restype = size_t
    lib.litearm_shm_header_size.restype = size_t
    lib.litearm_shm_segment_size.restype = size_t

    lib.litearm_shm_open.argtypes = [
        ctypes.c_char_p, ctypes.c_int, ctypes.POINTER(ctypes.c_void_p)
    ]
    lib.litearm_shm_open.restype = ctypes.c_int
    lib.litearm_shm_close.argtypes = [ctypes.c_void_p]
    lib.litearm_shm_close.restype = None
    lib.litearm_shm_unlink.argtypes = [ctypes.c_char_p]
    lib.litearm_shm_unlink.restype = ctypes.c_int

    lib.litearm_shm_publish_state.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmState)
    ]
    lib.litearm_shm_publish_state.restype = ctypes.c_int
    lib.litearm_shm_read_state.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmState), ctypes.c_int
    ]
    lib.litearm_shm_read_state.restype = ctypes.c_int

    lib.litearm_shm_publish_command.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmCommand)
    ]
    lib.litearm_shm_publish_command.restype = ctypes.c_int
    lib.litearm_shm_read_command.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmCommand), ctypes.c_int
    ]
    lib.litearm_shm_read_command.restype = ctypes.c_int

    lib.litearm_shm_read_header.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(LitearmHeader)
    ]
    lib.litearm_shm_read_header.restype = ctypes.c_int


def _verify_sizes(lib: ctypes.CDLL, path: Path) -> None:
    """加载即校验结构体大小，避免 ctypes 镜像与 .so 版本错配后静默错读。"""
    expected = (
        ("LitearmState", ctypes.sizeof(LitearmState), lib.litearm_shm_state_size()),
        ("LitearmCommand", ctypes.sizeof(LitearmCommand),
         lib.litearm_shm_command_size()),
        ("LitearmHeader", ctypes.sizeof(LitearmHeader), lib.litearm_shm_header_size()),
    )
    for name, mirror, native in expected:
        if mirror != native:
            raise ShmError(
                f"{path} 的 {name} 大小为 {native} 字节，"
                f"但 Python 镜像为 {mirror} 字节——共享内存布局已漂移，"
                f"请重新 colcon build 后同步更新 shm_bridge.py"
            )


_LIB: Optional[ctypes.CDLL] = None


def _lib() -> ctypes.CDLL:
    """惰性加载动态库（避免 import 阶段就依赖安装空间）。"""
    global _LIB
    if _LIB is None:
        _LIB = _load_library()
    return _LIB


def describe_layout() -> "dict[str, object]":
    """返回两侧布局的可比对快照，供交叉校验测试使用。"""
    lib = _lib()
    return {
        "state_size": int(lib.litearm_shm_state_size()),
        "state_mirror_size": ctypes.sizeof(LitearmState),
        "command_size": int(lib.litearm_shm_command_size()),
        "command_mirror_size": ctypes.sizeof(LitearmCommand),
        "header_size": int(lib.litearm_shm_header_size()),
        "header_mirror_size": ctypes.sizeof(LitearmHeader),
        "segment_size": int(lib.litearm_shm_segment_size()),
        "num_joints": NUM_JOINTS,
        "state_offsets": {name: getattr(LitearmState, name).offset
                          for name, _ in StateFields},
        "command_offsets": {name: getattr(LitearmCommand, name).offset
                            for name, _ in CommandFields},
    }


# ─────────────────────────── 高层封装 ───────────────────────────


class SharedMemory:
    """共享内存段的 RAII 封装。

    典型用法（守护进程侧）::

        with SharedMemory(create=True) as shm:
            shm.publish_state(state)

    ROS 插件侧用 ``SharedMemory(create=False)`` —— 段必须已由守护进程创建。
    """

    def __init__(self, name: str = DEFAULT_SHM_NAME, create: bool = False,
                 max_retries: int = 64) -> None:
        self.name = name
        self.max_retries = int(max_retries)
        self._handle = ctypes.c_void_p()
        code = _lib().litearm_shm_open(
            name.encode("utf-8"), 1 if create else 0, ctypes.byref(self._handle))
        if code != SHM_OK:
            raise ShmError(
                f"打开共享内存 {name!r} 失败：{_ERR_TEXT.get(code, '未知错误')}", code)

    # ── 生命周期 ──
    def close(self) -> None:
        if self._handle:
            _lib().litearm_shm_close(self._handle)
            self._handle = ctypes.c_void_p()

    def __enter__(self) -> "SharedMemory":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def __del__(self) -> None:  # pragma: no cover - 兜底
        self.close()

    @staticmethod
    def unlink(name: str = DEFAULT_SHM_NAME) -> None:
        """删除共享内存对象（幂等）。"""
        code = _lib().litearm_shm_unlink(name.encode("utf-8"))
        if code != SHM_OK:
            raise ShmError(f"删除共享内存 {name!r} 失败", code)

    # ── 状态块（守护进程写 / ROS 读）──
    def publish_state(self, state: LitearmState) -> None:
        code = _lib().litearm_shm_publish_state(self._handle, ctypes.byref(state))
        if code != SHM_OK:
            raise ShmError("发布状态失败", code)

    def read_state(self, max_retries: Optional[int] = None) -> LitearmState:
        """读取状态。撕裂重试耗尽时抛 :class:`ShmError`（code=SHM_TORN）。"""
        out = LitearmState()
        retries = self.max_retries if max_retries is None else int(max_retries)
        code = _lib().litearm_shm_read_state(self._handle, ctypes.byref(out), retries)
        if code != SHM_OK:
            raise ShmError(
                f"读取状态失败：{_ERR_TEXT.get(code, '未知错误')}", code)
        return out

    def try_read_state(self, max_retries: Optional[int] = None) -> Optional[LitearmState]:
        """读取状态，撕裂时返回 ``None`` 而非抛异常（守护进程侧轮询用）。"""
        try:
            return self.read_state(max_retries)
        except ShmError as exc:
            if exc.code == SHM_TORN:
                return None
            raise

    # ── 命令块（ROS 写 / 守护进程读）──
    def publish_command(self, command: LitearmCommand) -> None:
        code = _lib().litearm_shm_publish_command(self._handle, ctypes.byref(command))
        if code != SHM_OK:
            raise ShmError("发布命令失败", code)

    def read_command(self, max_retries: Optional[int] = None) -> LitearmCommand:
        out = LitearmCommand()
        retries = self.max_retries if max_retries is None else int(max_retries)
        code = _lib().litearm_shm_read_command(self._handle, ctypes.byref(out), retries)
        if code != SHM_OK:
            raise ShmError(
                f"读取命令失败：{_ERR_TEXT.get(code, '未知错误')}", code)
        return out

    def try_read_command(self, max_retries: Optional[int] = None) -> Optional[LitearmCommand]:
        try:
            return self.read_command(max_retries)
        except ShmError as exc:
            if exc.code == SHM_TORN:
                return None
            raise

    # ── 诊断 ──
    def header(self) -> LitearmHeader:
        out = LitearmHeader()
        code = _lib().litearm_shm_read_header(self._handle, ctypes.byref(out))
        if code != SHM_OK:
            raise ShmError("读取头部失败", code)
        return out


__all__ = [
    "DEFAULT_SHM_NAME",
    "NUM_JOINTS",
    "SHM_OK",
    "SHM_TORN",
    "LitearmCommand",
    "LitearmHeader",
    "LitearmState",
    "SharedMemory",
    "ShmError",
    "describe_layout",
]
