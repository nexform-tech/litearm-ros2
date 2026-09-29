#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共享内存布局的跨语言一致性测试。

分两层：

1. **加载期校验**（shm_bridge._verify_sizes）只能比对结构体总大小 ——
   C 侧加一个字段、Python 侧忘了同步，总量也可能恰好对得上。
2. 本测试补上字段级校验：调用 C++ 的 litearm_shm_layout_probe，
   把每个字段的 offsetof 与 ctypes 镜像的 .offset 逐项比对。

任何一处不一致都意味着两侧对同一段内存的解释不同，属于静默数据损坏级别的
缺陷，必须硬失败。
"""

import ctypes
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from litearm_ros2_control import shm_bridge
from litearm_ros2_control.shm_bridge import (CommandFields, HeaderFields,
                                             LitearmCommand, LitearmHeader,
                                             LitearmState, StateFields)


def _find_probe():
    """定位 litearm_shm_layout_probe。"""
    explicit = os.environ.get("LITEARM_SHM_LAYOUT_PROBE")
    # LITEARM_SHM_LAYOUT_PROBE 可能是「路径列表」：ament_add_pytest_test 的
    # APPEND_ENV 是按 PATH 语义实现的（会先插一个路径分隔符再拼值），
    # 直接当单个路径用会拿到形如 ":/path/to/probe" 的串。
    # 这里按搜索路径解析，两种写法都能用。
    explicit = os.environ.get("LITEARM_SHM_LAYOUT_PROBE", "")
    for part in explicit.split(os.pathsep):
        if part and os.path.exists(part):
            return part

    # colcon 构建/安装树：从本测试文件位置向上找工作区根。包在源码树里的
    # 位置不止一种（<ws>/src/<pkg>/test 或 <ws>/src/<其他层>/<pkg>/test），
    # 所以逐级上溯，找到含 build//install/ 产物的那一层为止。
    here = os.path.dirname(os.path.abspath(__file__))
    for ancestor in Path(here).parents:
        for candidate in (
            ancestor / "build" / "litearm_ros2_control" / "litearm_shm_layout_probe",
            ancestor / "install" / "litearm_ros2_control" / "lib"
            / "litearm_ros2_control" / "litearm_shm_layout_probe",
        ):
            if candidate.exists():
                return str(candidate)

    found = shutil.which("litearm_shm_layout_probe")
    if found:
        return found
    return None


@pytest.fixture(scope="module")
def probe():
    path = _find_probe()
    if path is None:
        pytest.skip("找不到 litearm_shm_layout_probe；请先 colcon build "
                    "或设置 LITEARM_SHM_LAYOUT_PROBE")
    output = subprocess.run([path], check=True, capture_output=True, text=True)
    return json.loads(output.stdout)


@pytest.fixture(scope="module")
def probe_layout(probe):
    return {name: probe[f"{name}_offsets"]
            for name in ("state", "command", "header")}


def test_struct_sizes_match(probe):
    """两侧结构体总大小必须一致。"""
    assert probe["state_size"] == shm_bridge._lib().litearm_shm_state_size()
    assert probe["command_size"] == shm_bridge._lib().litearm_shm_command_size()
    assert probe["header_size"] == shm_bridge._lib().litearm_shm_header_size()
    # 探针（C 侧真实布局）与 ctypes 镜像必须逐项对齐
    assert probe["state_size"] == ctypes.sizeof(LitearmState)
    assert probe["command_size"] == ctypes.sizeof(LitearmCommand)
    assert probe["header_size"] == ctypes.sizeof(LitearmHeader)


@pytest.mark.parametrize("group,fields,mirror", [
    ("state", StateFields, LitearmState),
    ("command", CommandFields, LitearmCommand),
    ("header", HeaderFields, LitearmHeader),
])
def test_field_offsets_match(probe, group, fields, mirror):
    """字段级偏移必须逐项一致——这是本测试的核心。"""
    c_offsets = probe[f"{group}_offsets"]
    assert set(c_offsets) == {name for name, _ in fields}, (
        f"{group} 的字段集合不一致：\n"
        f"  仅 C 侧有: {sorted(set(c_offsets) - {n for n, _ in fields})}\n"
        f"  仅 Python 有: {sorted({n for n, _ in fields} - set(c_offsets))}\n"
        f"C 侧或 Python 侧加了字段但没同步（改布局必须同时递增 "
        f"LITEARM_SHM_LAYOUT_VERSION）")

    mismatches = []
    for name, _ in fields:
        c_offset = c_offsets[name]
        py_offset = getattr(mirror, name).offset
        if c_offset != py_offset:
            mismatches.append(f"  {group}.{name}: C={c_offset} Python={py_offset}")
    assert not mismatches, "字段偏移不一致：\n" + "\n".join(mismatches)


def test_magic_and_version(probe):
    """段头部常量必须两侧一致，否则新旧进程会互相误读对方的内存。"""
    assert probe["magic"] == 0x4C41524D
    # v2：命令块加 acceleration[7]（M·q̈ 前馈用期望加速度）。
    # 改布局必须同步递增 litearm_shm.h 的 LITEARM_SHM_LAYOUT_VERSION 与本断言。
    assert probe["layout_version"] == 2
    assert probe["num_joints"] == shm_bridge.NUM_JOINTS == len(LitearmState().position)


def test_segment_size_covers_blocks(probe):
    """段必须容纳头部 + 两个块（含 8 字节对齐余量）。"""
    minimum = probe["header_size"] + probe["state_size"] + probe["command_size"]
    assert probe["segment_size"] >= minimum
    assert probe["segment_size"] % 8 == 0


def test_daemon_status_codes_match_header():
    """Python 侧 DAEMON_* 码必须与 litearm_shm.h 的定义一致。

    这里做的是"清单完整性"检查：任何一个被内部使用的码，都必须在
    DAEMON_STATUS_TEXT 里有可读文本，否则报错信息会出现裸露数字。
    """
    codes = {
        name: value for name, value in vars(shm_bridge).items()
        if name.startswith("DAEMON_") and isinstance(value, int)
        and not name.startswith("DAEMON_STATUS")
    }
    assert codes, "未找到 DAEMON_* 常量"
    missing = [name for name, value in codes.items()
               if value not in shm_bridge.DAEMON_STATUS_TEXT]
    assert not missing, f"这些码缺少可读文本: {missing}"

    # 与 litearm_shm.h 的取值对齐（顺序不能变）
    expected = {
        "DAEMON_OK": 0,
        "DAEMON_CONNECTING": 1,
        "DAEMON_HOLDING_STALE_COMMAND": 2,
        "DAEMON_HOLDING_ESTOP": 3,
        "DAEMON_HOLDING_MOTOR_FAULT": 4,
        "DAEMON_HOLDING_FEEDBACK_STALE": 5,
        "DAEMON_HOLDING_OVERTEMP": 6,
        "DAEMON_DISABLED": 7,
        "DAEMON_HOLDING_WATCHDOG": 8,
        "DAEMON_SHUTTING_DOWN": 9,
        "DAEMON_HOLDING_BAD_COMMAND": 10,
    }
    assert {k: codes[k] for k in expected} == expected


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
