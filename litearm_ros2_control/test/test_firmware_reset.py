#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""``scripts/firmware_reset.py`` 的回归测试（无硬件，用假固件）。

锁死三件事——它们都是真机上"一锁就得翻代码"的直接教训：

1. **只读模式零副作用**：锁存态下不给 ``--reset``/``--clear-faults`` 时，
   脚本只读、只给建议，**状态一点都不能变**（否则一个"看看"就把臂解了锁）。
2. **``--reset`` 真能解 EMERGENCY 锁存**：这是 0x06 的唯一出路
   （守护进程只会重连重发 ENABLE，永远解不开）。
3. **``enabled=True`` 时拒绝动作**：有别的东西在控制它时清故障毫无意义 ——
   清了也会被下一帧命令覆盖，甚至制造"我清了但它还在动"的错觉。

脚本用 ``importlib`` 按路径加载（``scripts/`` 不在包里，不能 import）。
"""

import importlib.util
import sys
import time
from pathlib import Path

import pytest

_SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _SOURCE_PYTHON.is_dir() and str(_SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PYTHON))

from litearm_ros2_control import fake_firmware, stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import Stm32Link  # noqa: E402

_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "firmware_reset.py"


@pytest.fixture()
def reset_mod():
    """按路径加载 scripts/firmware_reset.py（它不是包的一部分）。"""
    spec = importlib.util.spec_from_file_location("firmware_reset", _SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _locked_fake():
    """起一块假固件并让进入 EMERGENCY 锁存（真机 0x06 的成因之一），返回 (fw, port)。"""
    fw = fake_firmware.FakeFirmware()
    port = fw.start()
    link = Stm32Link(port)
    link.open()
    try:
        # 让固件持有使能态：假固件的看门狗/急停语义与真机一致（急停 => 失能 + 锁存）
        link.enable()
        link.wait_enabled(timeout_s=2.0)
        time.sleep(0.1)
        link.emergency_stop()
        time.sleep(0.3)
    finally:
        link.close()
    return fw, port


def _status_of(port):
    link = Stm32Link(port)
    link.open()
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            link.poll()
            if link.status is not None:
                return link.status
            time.sleep(0.01)
        return None
    finally:
        link.close()


def test_read_only_leaves_state_untouched(reset_mod, capsys):
    """不给动作开关时：只读、给建议、状态零变化。"""
    fw, port = _locked_fake()
    try:
        before = _status_of(port)
        assert before is not None and before.mode == proto.ARM_MODE_EMERGENCY

        code = reset_mod.main(["--port", port])
        assert code == 0
        out = capsys.readouterr().out
        assert "--reset" in out, "只读模式必须给出下一步该发什么命令"

        after = _status_of(port)
        assert after.mode == proto.ARM_MODE_EMERGENCY, "只读模式动了锁存态"
        assert after.enabled is False
    finally:
        fw.stop()


def test_reset_unlocks_emergency(reset_mod, capsys):
    """--reset 解掉 EMERGENCY 与 FAULT（= ENABLE 不再回 0x06）。"""
    fw, port = _locked_fake()
    try:
        assert _status_of(port).mode == proto.ARM_MODE_EMERGENCY

        code = reset_mod.main(["--port", port, "--reset"])
        assert code == 0
        assert "已解除" in capsys.readouterr().out

        after = _status_of(port)
        assert after.mode != proto.ARM_MODE_EMERGENCY
        assert after.fault is False
        assert not (after.joint_fault or 0)

        # 解锁后固件应重新接受 ENABLE（这才是"能起栈了"的实证）
        link = Stm32Link(port)
        link.open()
        try:
            link.enable()
            assert link.wait_enabled(timeout_s=3.0) is not None
            link.disable()
        finally:
            link.close()
    finally:
        fw.stop()


def test_refuses_when_enabled(reset_mod, capsys):
    """enabled=True（别的东西在控制它）时必须拒绝动作，退出码 3。"""
    fw = fake_firmware.FakeFirmware()
    port = fw.start()
    link = Stm32Link(port)
    link.open()
    try:
        link.enable()
        assert link.wait_enabled(timeout_s=2.0) is not None

        code = reset_mod.main(["--port", port, "--reset"])
        assert code == 3
        err = capsys.readouterr().err
        assert "enabled=True" in err
    finally:
        link.close()
        fw.stop()
