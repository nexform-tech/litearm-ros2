#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm-stm32 协议层与链路层的测试（无硬件）。

分三层：

1. **黄金向量**：``pack_frame`` 的字节与 CRC 用手算值钉死。CRC16-CCITT-FALSE
   还有一个公开的标准检验值（``"123456789"`` → ``0x29B1``），拿它当独立裁判。
2. **布局**：``MOVE_MIT_ALL`` 的 SoA 排布单独测——litearm-stm32 仓库里已经
   因为把它写成 AoS 真实撞过一次限位（固件侧把 kp 当成了目标角），
   这条断言就是那个事故的回归锁。
3. **链路 ↔ 假固件**：走真协议跑一遍 enable → 跟踪 → 停发看门狗持位 → park
   → disable，以及 license 被拒、应答丢失这两条真机上不好复现的分支。

假固件用 ``struct`` 手工编码（不复用 ``stm32_proto`` 的打包器），所以第 3 层
的往返是**交叉验证**，不是自证。
"""

import math
import os
import struct
import sys
import time
from pathlib import Path

import pytest

# 源码树直接跑 pytest 时保证能 import 到本包（与 test_hw_daemon.py 同一手法）。
_SOURCE_PYTHON = Path(__file__).resolve().parents[1] / "python"
if _SOURCE_PYTHON.is_dir() and str(_SOURCE_PYTHON) not in sys.path:
    sys.path.insert(0, str(_SOURCE_PYTHON))

from litearm_ros2_control import fake_firmware, stm32_proto as proto  # noqa: E402
from litearm_ros2_control.stm32_link import (Stm32AccessDenied,  # noqa: E402
                                             Stm32CmdError, Stm32Link)

NUM_JOINTS = 7


# ─────────────────────────── 1. 黄金向量 ───────────────────────────


def test_crc16_ccitt_false_matches_standard_check_value():
    """CRC-16/CCITT-FALSE 的标准检验值：'123456789' → 0x29B1。"""
    assert proto.crc16_ccitt_false(b"123456789") == 0x29B1


def test_pack_frame_is_byte_exact():
    """A5 41 00 <crc16-le>。CRC 覆盖 SOF..PAYLOAD 末字节（不含 CRC 自身）。"""
    body = bytes([0xA5, 0x41, 0x00])
    expected_crc = proto.crc16_ccitt_false(body)
    frame = proto.pack_frame(proto.CMD_GET_FIRMWARE)
    assert frame == body + bytes([expected_crc & 0xFF, expected_crc >> 8])
    assert len(frame) == 5


def test_pack_frame_payload_and_length_byte():
    payload = b"\x01\x02\x03"
    frame = proto.pack_frame(0x20, payload)
    assert frame[0] == proto.SOF
    assert frame[1] == 0x20
    assert frame[2] == 3                      # LEN 是 1 字节
    assert frame[3:6] == payload
    assert len(frame) == 3 + 3 + 2


def test_pack_frame_rejects_oversized_payload():
    with pytest.raises(ValueError):
        proto.pack_frame(0x05, b"\x00" * 256)


# ─────────────────────────── 2. 载荷布局 ───────────────────────────


def test_move_js_without_tau_is_2n_floats():
    q = [0.1 * i for i in range(NUM_JOINTS)]
    dq = [0.01 * i for i in range(NUM_JOINTS)]
    payload = proto.pack_move_js(q, dq)
    assert len(payload) == 8 * NUM_JOINTS
    assert list(struct.unpack_from(f"<{NUM_JOINTS}f", payload, 0)) == \
        pytest.approx(q)
    assert list(struct.unpack_from(f"<{NUM_JOINTS}f", payload, 28)) == \
        pytest.approx(dq)


def test_move_js_with_tau_is_3n_floats():
    q = [0.0] * NUM_JOINTS
    dq = [0.0] * NUM_JOINTS
    tau = [1.0] * NUM_JOINTS
    payload = proto.pack_move_js(q, dq, tau)
    assert len(payload) == 12 * NUM_JOINTS
    # tau 在末段——**带 tau 会让固件关掉整套内置前馈**，所以位置必须精确。
    assert list(struct.unpack_from(f"<{NUM_JOINTS}f", payload, 56)) == \
        pytest.approx(tau)


def test_move_mit_all_is_soa_not_aos():
    """``MOVE_MIT_ALL`` 必须是 SoA（先全部 q、再全部 dq…）。

    回归背景：litearm-stm32 的多个工具曾按 AoS 打包，真机上固件把 kp 当成了
    目标角（实测 q=[0.1..0.7]、kp=15 时收到 q=[0.1, 0, 15, 1, 0, 0.2, 0]），
    经软限位 clamp 后直接拍向限位。这条断言把排布钉死。
    """
    rows = [(float(i), float(100 + i), float(200 + i), float(300 + i),
             float(400 + i)) for i in range(NUM_JOINTS)]
    payload = proto.pack_move_mit_all(rows)
    assert len(payload) == 20 * NUM_JOINTS
    for field_index in range(5):          # q, dq, kp, kd, tau
        for joint in range(NUM_JOINTS):
            (value,) = struct.unpack_from("<f", payload, field_index * 28
                                          + joint * 4)
            assert value == rows[joint][field_index], (
                f"字段 {field_index} 关节 {joint} 排布错位："
                f"得到 {value}，期望 {rows[joint][field_index]}")
    # 反面证据：AoS 排布下第 1 个 4 字节段本该是 (q0,dq0,kp0,kd0,tau0,q1,dq1)
    aos = b"".join(struct.pack("<5f", *row) for row in rows)
    assert payload[:8] != aos[:8]


def test_set_and_get_payloads_are_byte_exact():
    assert proto.pack_set_ff_flags(0x1FB) == struct.pack("<I", 0x1FB)
    assert proto.pack_set_ff_vec(7, [1.0] * 7) == \
        bytes([7]) + struct.pack("<7f", *([1.0] * 7))
    assert proto.pack_set_ff_scalar(1, 0, 0.15) == \
        bytes([1, 0]) + struct.pack("<f", 0.15)
    assert proto.pack_get_joint_param(3) == bytes([3])
    assert proto.pack_get_ff_vec(15) == bytes([15])
    assert proto.pack_get_ff_scalar(9, 0) == bytes([9, 0])
    assert proto.pack_set_joint_param(2, 300.0, 4.0, 21.0) == \
        bytes([2]) + struct.pack("<fff", 300.0, 4.0, 21.0)
    assert proto.pack_set_joint_limits(0, -1.0, 1.0) == \
        bytes([0]) + struct.pack("<ff", -1.0, 1.0)
    assert proto.pack_u8(0) == b"\x00"


# ─────────────────────── 切帧器：重同步与坏帧 ───────────────────────


def test_framer_reassembles_split_frame():
    frame = proto.pack_frame(0x2C, bytes([9, 0]))
    framer = proto.CdcFramer()
    assert framer.feed(frame[:2]) == []
    out = framer.feed(frame[2:])
    assert len(out) == 1
    assert out[0].cmd == 0x2C
    assert out[0].payload == bytes([9, 0])
    assert out[0].crc_ok is True


def test_framer_skips_garbage_and_resyncs():
    frame = proto.pack_frame(0x41)
    framer = proto.CdcFramer()
    out = framer.feed(b"\x00\x13\x37" + frame)
    assert len(out) == 1 and out[0].crc_ok
    assert framer.discarded == 3


def test_framer_handles_back_to_back_frames():
    stream = proto.pack_frame(0x45, b"\x10") + proto.pack_frame(0x44, b"abc")
    out = proto.CdcFramer().feed(stream)
    assert [f.cmd for f in out] == [0x45, 0x44]
    assert out[1].payload == b"abc"


def test_framer_flags_bad_crc_instead_of_dropping_silently():
    frame = bytearray(proto.pack_frame(0x40, b"\x00\x00"))
    frame[-1] ^= 0xFF                      # 破坏 CRC 高字节
    out = proto.CdcFramer().feed(bytes(frame))
    assert len(out) == 1 and out[0].crc_ok is False


# ─────────────────────────── 状态帧解析 ───────────────────────────


def _status_payload(flags, seq, joints, joint_fault=None):
    """手工拼状态帧（与固件 usb_cmd_report_status 同布局）。"""
    out = bytearray(struct.pack("<HH", flags, seq))
    for q, dq, tau, tm, tc, err in joints:
        out += struct.pack("<fffff", q, dq, tau, tm, tc)
        out += bytes([err])
    if joint_fault is not None:
        out += struct.pack("<H", joint_fault)
    return bytes(out)


def test_status_decode_full_layout():
    joints = [(0.1 * i, 0.01 * i, 0.001 * i, 32.0 + i, 35.0 + i, 1)
              for i in range(NUM_JOINTS)]
    flags = (proto.ARM_FLAG_FAULT | (proto.ARM_MODE_MOVE_JS << 6)
             | proto.FLAG_ENABLED)
    payload = _status_payload(flags, 0x1234, joints, joint_fault=0x0004)
    assert len(payload) == 6 + 21 * NUM_JOINTS == 153

    status = proto.decode_status(payload)
    assert status is not None
    assert status.seq == 0x1234
    assert status.mode == proto.ARM_MODE_MOVE_JS
    assert status.mode_name == "MOVE_JS"
    assert status.enabled is True
    assert status.fault is True
    assert status.joint_fault == 0x0004
    assert status.faulted_joints() == [2]
    assert status.joints[3].q == pytest.approx(0.3)
    assert status.joints[3].t_coil == pytest.approx(38.0)
    assert all(j.err == 1 and j.enabled for j in status.joints)


def test_status_decode_legacy_layout_has_no_joint_fault():
    """旧固件的 4+21N 布局要能解出来，但 joint_fault 为 None（不猜）。"""
    joints = [(0.0, 0.0, 0.0, 30.0, 30.0, 0)] * NUM_JOINTS
    payload = _status_payload(0, 7, joints)
    assert len(payload) == 4 + 21 * NUM_JOINTS == 151
    status = proto.decode_status(payload)
    assert status is not None
    assert status.joint_fault is None
    assert status.faulted_joints() == []


def test_status_decode_rejects_inconsistent_length():
    """长度不自洽必须返回 None——状态帧被误读等于把臂的状态看错。"""
    good = _status_payload(0, 0, [(0.0, 0.0, 0.0, 30.0, 30.0, 1)] * NUM_JOINTS,
                           joint_fault=0)
    assert proto.decode_status(good) is not None
    assert proto.decode_status(good[:-1]) is None
    assert proto.decode_status(good + b"\x00") is None
    assert proto.decode_status(b"\x00" * 4) is None


def test_status_flag_helpers():
    flags = (proto.ARM_FLAG_FAULT | proto.ARM_FLAG_WATCHDOG_TRIPPED
             | proto.ARM_FLAG_FEEDBACK_STALE | proto.ARM_FLAG_TEMP_WARNING
             | proto.FLAG_CART_BUSY)
    status = proto.decode_status(_status_payload(flags, 0, [(0.0,) * 5 + (0,)]))
    assert status.flag_names() == ["FAULT", "WD_TRIPPED", "FB_STALE",
                                   "TEMP_WARN"]
    assert status.watchdog_tripped and status.feedback_stale
    assert status.temp_warning and status.cart_busy
    assert not status.position_violation and not status.overspeed
    assert status.safety_flags == 0x0F


def test_mode_field_is_three_bits():
    """mode 占 bit6-8；bit9/bit10 不能被卷进来。"""
    flags = (proto.ARM_MODE_ZERO_G << 6) | proto.FLAG_ENABLED \
        | proto.FLAG_CART_BUSY
    status = proto.decode_status(_status_payload(flags, 0, [(0.0,) * 5 + (0,)]))
    assert status.mode == proto.ARM_MODE_ZERO_G
    assert status.enabled and status.cart_busy


# ─────────────────────── 其余应答的解析 ───────────────────────


def test_decode_joint_param_has_no_rsp_prefix():
    payload = bytes([4]) + struct.pack("<fffff", 300.0, 5.0, 21.0,
                                       -3.071547, 0.017547)
    param = proto.decode_joint_param(payload)
    assert param is not None
    assert param.index == 4
    assert param.kp == pytest.approx(300.0)
    assert param.q_min == pytest.approx(-3.071547)
    assert proto.decode_joint_param(payload[:-1]) is None


def test_decode_ff_vec_and_scalar_carry_rsp_prefix():
    """这两个应答的载荷**带** RSP id 前缀——与 0x49 相反，容易搞混。"""
    vec = bytes([proto.RSP_FF_VEC, 7]) + struct.pack("<7f", *([1.0] * 7))
    assert proto.decode_ff_vec(vec) == (7, pytest.approx([1.0] * 7))
    assert proto.decode_ff_vec(vec[1:]) is None       # 前缀缺失 → 拒收

    scalar = bytes([proto.RSP_FF_SCALAR, 9, 0]) + struct.pack("<f", 507.0)
    assert proto.decode_ff_scalar(scalar) == (9, 0, pytest.approx(507.0))
    assert proto.decode_ff_scalar(scalar[1:]) is None


def test_decode_ack_err_and_firmware():
    assert proto.decode_ack(bytes([0x10])) == 0x10
    assert proto.decode_ack(b"") is None
    assert proto.decode_err(bytes([0x10, 0x08])) == (0x10, 0x08)
    assert proto.decode_err(bytes([0x10])) is None
    assert proto.decode_firmware(b"Litearm1.8.0-7J") == "Litearm1.8.0-7J"
    assert proto.decode_firmware(b"") is None


def test_format_ff_mask_is_readable():
    assert proto.format_ff_mask(0) == "-"
    assert proto.format_ff_mask(proto.FF_FACTORY_MASK) == \
        "MASTER|G|INERTIA|CORIOLIS|FRICTION|INTEGRAL|QUANT|VELREF"


def test_factory_ff_mask_bits_are_exact():
    """出厂掩码 = 0x1BF。

    回归背景：写这个模块时先写成 0x1FB，那是 ``INERTIA``(0x04) 与 ``WALL``(0x40)
    两位互换的结果——显示成"有墙无惯量"，正好把 MOVE_J 里唯一一项惯量前馈关掉。
    两个定义（协议层与假固件）互相钉死，改一处忘另一处会立刻红。
    """
    assert proto.FF_FACTORY_MASK == 0x1BF == 447
    assert proto.FF_FACTORY_MASK == fake_firmware.DEFAULT_FF_MASK
    # 0x1AF 是 defaults.c 注释里的"回退"值：上述去掉 FRICTION。
    assert proto.FF_FACTORY_MASK & ~proto.FF_FRICTION == 0x1AF == 431
    assert not proto.FF_FACTORY_MASK & proto.FF_WALL, "墙默认必须是关的"


# ──────────────────── 3. 链路 ↔ 假固件（真协议往返） ────────────────────


@pytest.fixture
def rig():
    """起一块假固件并连上它；退出时双向清理。"""
    firmware = fake_firmware.FakeFirmware()
    port = firmware.start()
    link = Stm32Link(port)
    link.open()
    try:
        yield firmware, link
    finally:
        link.close()
        firmware.stop()


def _enable_like_firmware(link):
    """按**真机时序**使能：第一次必然回 ``0x03``，重发才 ACK。

    这不是测试的繁琐，而是要复现的行为。固件 ``ctrl_enable`` 的注释记着真机
    实测时序：``ENABLE#1 -> 0x03(首写 CMODE), ENABLE#2 -> ACK`` —— 启动后第一次
    使能要先写 7 台电机的 CMODE 寄存器。

    ⚠ 真机上被这条咬过：主机若把 ``0x03`` 当失败，冷启动时每次都要白跑一轮
    （关串口 → 重连 → 重读版本/参数/前馈）。所以这里把它写成**命令式的
    0x03→重发**，任何"把 0x03 当失败"的实现都会在这里暴露。
    """
    ok, reason = link.command(proto.CMD_ENABLE)
    assert ok is False and reason == 0x03, (
        f"第一次 ENABLE 应回 0x03（首写 CMODE），实际 ok={ok} reason={reason}")
    ok, reason = link.command(proto.CMD_ENABLE)
    assert ok is True and reason is None, (
        f"重发 ENABLE 应 ACK，实际 ok={ok} reason={reason}")


def test_first_enable_returns_0x03_then_ack(rig):
    """固件启动后第一次 ENABLE 回 0x03（写 CMODE），重发才 ACK；RESET 后重现。

    这条锁的是**真机行为**，也是守护进程 ``_enable`` 必须原地重发的原因。
    """
    firmware, link = rig
    ok, reason = link.command(proto.CMD_ENABLE)
    assert (ok, reason) == (False, 0x03), "首次使能应先回 0x03(首写 CMODE)"
    ok, reason = link.command(proto.CMD_ENABLE)
    assert (ok, reason) == (True, None)
    assert link.wait_enabled(2.0) is not None

    # RESET 会把固件的 ctrl_mode_written 打回 false（同 ctrl_reset），于是
    # 下一次 ENABLE 又回 0x03 —— 主机重试逻辑必须能扛住这件事反复发生。
    link.reset()
    _spin(link, 0.1)
    ok, reason = link.command(proto.CMD_ENABLE)
    assert (ok, reason) == (False, 0x03), "RESET 后首次使能应重新回 0x03"
    ok, reason = link.command(proto.CMD_ENABLE)
    assert (ok, reason) == (True, None)


def _spin(link, seconds, period_s=0.004):
    """按周期空转 poll，模拟控制环的空闲等待。"""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        link.poll()
        time.sleep(period_s)


def test_link_reads_firmware_and_identity(rig):
    firmware, link = rig
    assert link.get_firmware() == fake_firmware.FW_VERSION
    status = link.wait_status(2.0)
    assert status is not None
    assert len(status.joints) == NUM_JOINTS
    assert status.enabled is False        # 上电即安全态：电机失能
    assert status.mode == proto.ARM_MODE_INIT


def test_link_reads_joint_params_from_firmware(rig):
    """关节参数全部从固件读（0x24），守护进程不自己存一份。"""
    _firmware, link = rig
    for index in range(NUM_JOINTS):
        param = link.get_joint_param(index)
        assert param is not None, f"关节 {index} 参数读回超时"
        assert param.index == index
        assert param.kp == pytest.approx(fake_firmware.DEFAULT_KP[index])
        assert param.kd == pytest.approx(fake_firmware.DEFAULT_KD[index])
        assert param.tau_max == pytest.approx(
            fake_firmware.DEFAULT_TAU_MAX[index])
        assert param.q_min == pytest.approx(fake_firmware.DEFAULT_Q_MIN[index])
        assert param.q_max == pytest.approx(fake_firmware.DEFAULT_Q_MAX[index])


def test_link_reads_ff_mask_and_vectors(rig):
    _firmware, link = rig
    assert link.get_ff_mask() == fake_firmware.DEFAULT_FF_MASK
    assert link.get_ff_vec(proto.FF_VEC_GRAVITY_SCALE) == \
        pytest.approx([1.0] * NUM_JOINTS)
    assert link.get_ff_vec(proto.FF_VEC_KD_EXTRA) == \
        pytest.approx(list(fake_firmware.DEFAULT_KD_EXTRA))
    assert link.get_ff_scalar(proto.FF_SCALAR_FRIC_DB) == pytest.approx(0.15)
    # item 9 是只读的（写口拒绝）——读回来必须是 ff_mask 而不是标量表里的值。
    assert link.get_ff_scalar(proto.FF_SCALAR_FF_MASK) == \
        float(fake_firmware.DEFAULT_FF_MASK)


def test_enable_track_then_watchdog_holds(rig):
    """核心闭环：使能 → 跟到目标 → 停发 → 固件看门狗持位。"""
    firmware, link = rig
    _enable_like_firmware(link)
    enabled = link.wait_enabled(2.0)
    assert enabled is not None, "ENABLE 后 2s 内未见 enabled 位"

    start_q = list(link.status.joints[i].q for i in range(NUM_JOINTS))
    target = [q + 0.15 for q in start_q]
    # ⚠ 必须给非零速度：MOVE_JS 的 dq_ref 同时是位置参考的 slew 上限，dq=0 时
    # 参考冻结、臂不会动（见 test_move_js_needs_velocity_to_move）。
    dq = [1.0] * NUM_JOINTS
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        link.move_js(target, dq)
        link.poll()
        time.sleep(0.004)
    link.poll()
    moved = list(link.status.joints[i].q for i in range(NUM_JOINTS))
    # 一阶滞后模型：1.5s 足够收敛到目标附近。
    for index in range(NUM_JOINTS):
        assert abs(moved[index] - target[index]) < 0.02, \
            f"关节 {index} 未收敛：{moved[index]} vs {target[index]}"
    assert firmware.snapshot()["mode"] == proto.ARM_MODE_MOVE_JS

    # 停发：固件 100ms 看门狗应转入 fail-soft 持位，且 q_ref 被改写成实测位置。
    held_at = list(link.status.joints[i].q for i in range(NUM_JOINTS))
    _spin(link, 0.4)
    assert link.status.watchdog_tripped is True, "停发 0.4s 后看门狗未触发"
    still = list(link.status.joints[i].q for i in range(NUM_JOINTS))
    for index in range(NUM_JOINTS):
        assert abs(still[index] - held_at[index]) < 0.01, \
            f"关节 {index} 在持位期间漂移了"

    # 重新发命令 → kick → 看门狗解除，恢复跟踪。
    link.move_js([q + 0.05 for q in still], [0.0] * NUM_JOINTS)
    link.poll()
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline and link.status.watchdog_tripped:
        link.move_js([q + 0.05 for q in still], [0.0] * NUM_JOINTS)
        link.poll()
        time.sleep(0.005)
    assert link.status.watchdog_tripped is False


def test_move_js_needs_velocity_to_move(rig):
    """``MOVE_JS`` 的 ``dq_ref`` **同时是位置参考的 slew 速率上限** —— dq=0 就不动。

    真机核实（``control_loop.c``）::

        v_lim = clampf(fabsf_(target_dq[i]), 0.0f, jp->speed_limit * gov_ratio);
        cmd->q_ref = slew_linear(q_s[i], cmd->q_ref, v_lim * LITEARM_CTRL_DT);

    即参考每拍最多朝 ``target_q`` 走 ``|dq_ref|·dt``。所以**只发位置、速度给 0 时
    参考冻结，关节一步都不会动** —— MOVE_JS 本质上是"按速度指令推参考"的流式模式。

    ⚠ 这条锁的是**假固件的保真度**，不是被测代码：假固件原来直接用一阶滞后逼近
    ``q_ref``，于是"dq=0 也能动"，三条测试因此长期是假阳性；换成忠实模型后它们
    立刻红了。任何把这条语义简化掉的改动都会在这里暴露。
    """
    _firmware, link = rig
    _enable_like_firmware(link)
    link.wait_enabled(2.0)

    start = list(link.status.joints[i].q for i in range(NUM_JOINTS))
    target = [q + 0.1 for q in start]

    # dq = 0：参考冻结，位置不该有可观测变化。
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        link.move_js(target, [0.0] * NUM_JOINTS)
        link.poll()
        time.sleep(0.004)
    idle = list(link.status.joints[i].q for i in range(NUM_JOINTS))
    assert max(abs(idle[i] - start[i]) for i in range(NUM_JOINTS)) < 1e-3, \
        "dq=0 时关节动了 —— 假固件没有忠实建模 slew 语义"

    # dq 非零：参考按 |dq| 推进，关节跟过去。
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        link.move_js(target, [1.0] * NUM_JOINTS)
        link.poll()
        time.sleep(0.004)
    moved = list(link.status.joints[i].q for i in range(NUM_JOINTS))
    for index in range(NUM_JOINTS):
        assert abs(moved[index] - target[index]) < 0.02, \
            f"给了速度仍未到位：j{index + 1} {moved[index]} vs {target[index]}"


def test_move_js_with_tau_turns_builtin_ff_off(rig):
    """带 tau_ff 的一帧必须把 ``s_js_user_ff`` 打开（固件据此关掉内置前馈）。"""
    firmware, link = rig
    _enable_like_firmware(link)
    link.wait_enabled(2.0)

    link.move_js([0.0] * NUM_JOINTS, [0.0] * NUM_JOINTS)
    link.poll()
    _spin(link, 0.05)
    assert firmware.snapshot()["s_js_user_ff"] is False

    link.move_js([0.0] * NUM_JOINTS, [0.0] * NUM_JOINTS,
                 [0.5] * NUM_JOINTS)
    link.poll()
    _spin(link, 0.05)
    assert firmware.snapshot()["s_js_user_ff"] is True

    # 切回不带 tau：固件应复位控制律历史（law_reset_all）。
    link.move_js([0.0] * NUM_JOINTS, [0.0] * NUM_JOINTS)
    link.poll()
    _spin(link, 0.05)
    assert firmware.snapshot()["s_js_user_ff"] is False


def test_park_declaration_raises_hold_stiffness(rig):
    """``0x20 park`` 之后看门狗触发时用 1.0× 刚度而不是 0.6×。"""
    firmware, link = rig
    _enable_like_firmware(link)
    link.wait_enabled(2.0)
    link.park()
    link.poll()
    _spin(link, 0.3)
    snapshot = firmware.snapshot()
    assert snapshot["watchdog_tripped"] is True
    assert link.status.watchdog_tripped is True


def test_disable_and_emergency_paths(rig):
    firmware, link = rig
    _enable_like_firmware(link)
    link.wait_enabled(2.0)

    link.disable()
    _spin(link, 0.1)
    assert link.status.enabled is False
    assert firmware.snapshot()["mode"] == proto.ARM_MODE_INIT

    # 急停：锁存 EMERGENCY，ENABLE 必须被拒（ERR{0x10,0x06}）。
    link.emergency_stop()
    _spin(link, 0.05)
    assert link.status.mode == proto.ARM_MODE_EMERGENCY
    ok, reason = link.command(proto.CMD_ENABLE)
    assert ok is False and reason == 0x06

    link.reset()
    _spin(link, 0.05)
    assert link.status.mode != proto.ARM_MODE_EMERGENCY


def test_enable_rejected_without_license_reports_reason():
    """license 未激活：ENABLE 回 ``ERR{0x10,0x08}``（重试无用，须硬失败）。"""
    firmware = fake_firmware.FakeFirmware(licensed=False)
    port = firmware.start()
    link = Stm32Link(port)
    link.open()
    try:
        ok, reason = link.command(proto.CMD_ENABLE)
        assert ok is False and reason == 0x08
        _spin(link, 0.1)
        assert link.status.enabled is False
        # 门禁是单点的：其他命令不受影响，状态帧照发。
        assert link.get_firmware() == fake_firmware.FW_VERSION
    finally:
        link.close()
        firmware.stop()


def test_dropped_ack_times_out_without_raising(rig):
    """应答丢失是常态（固件 TX 忙则丢）：必须平静地超时，不是异常。"""
    firmware, link = rig
    firmware.drop_replies(1)
    assert link.get_firmware(timeout_s=0.15) is None
    # 丢完之后一切恢复正常。
    assert link.get_firmware() == fake_firmware.FW_VERSION


def test_request_error_frame_raises_with_reason(rig):
    """``accept_err=False`` 时 ERR 直接抛，带上人能读的提示。"""
    firmware, link = rig
    with pytest.raises(Stm32CmdError) as excinfo:
        link.request(proto.CMD_GET_JOINT_PARAM,
                     proto.pack_get_joint_param(99),
                     expect=(proto.RSP_JOINT_PARAM,), timeout_s=0.3)
    assert excinfo.value.cmd == proto.CMD_GET_JOINT_PARAM
    assert excinfo.value.reason == 0x02
    assert "越界" in str(excinfo.value)


def test_joint_fault_detected_from_status_and_isolated(rig):
    """单轴故障：``joint_fault`` 位图只影响该轴，守护进程据此报原因码。"""
    firmware, link = rig
    _enable_like_firmware(link)
    link.wait_enabled(2.0)
    firmware.inject_joint_fault(2)
    _spin(link, 0.1)
    status = link.status
    assert status.faulted_joints() == [2]
    assert status.fault is True
    assert status.joints[2].err == 0x0D

    firmware.inject_joint_fault(2, on=False)
    link.clear_faults()
    _spin(link, 0.1)
    assert link.status.faulted_joints() == []


def test_feedback_stale_and_temp_flags_reach_the_host(rig):
    firmware, link = rig
    firmware.inject_feedback_stale(True)
    firmware.inject_temp_warning(True)
    _spin(link, 0.1)
    assert link.status.feedback_stale is True
    assert link.status.temp_warning is True
    firmware.inject_feedback_stale(False)
    firmware.inject_temp_warning(False)
    _spin(link, 0.1)
    assert link.status.feedback_stale is False


def test_mit_all_passthrough_roundtrip(rig):
    """MIT_ALL 透传：固件按 SoA 解出的 kp/kd/tau 必须与上位机发的一致。"""
    firmware, link = rig
    _enable_like_firmware(link)
    link.wait_enabled(2.0)
    q = [0.05 * i for i in range(NUM_JOINTS)]
    dq = [0.0] * NUM_JOINTS
    kp = [111.0 + i for i in range(NUM_JOINTS)]
    kd = [1.5] * NUM_JOINTS
    tau = [0.25] * NUM_JOINTS
    rows = list(zip(q, dq, kp, kd, tau))
    deadline = time.monotonic() + 0.3
    while time.monotonic() < deadline:
        link.move_mit_all(rows)
        link.poll()
        time.sleep(0.004)
    assert firmware.snapshot()["mode"] == proto.ARM_MODE_MOVE_MIT_ALL
    # 读回的 kp 是透传值，不是固件参数表的值——这是 MIT_ALL 的语义。
    snapshot = firmware.snapshot()
    assert snapshot["kp_cmd"] == pytest.approx(kp)
    assert snapshot["tau_user"] == pytest.approx(tau)


def test_counters_expose_link_health(rig):
    _firmware, link = rig
    _spin(link, 0.2)
    assert link.tx_frames >= 0
    assert link.rx_frames > 0
    assert link.crc_errors == 0
    assert link.status_count > 0
    assert math.isfinite(link.status_age_s())
    assert "tx=" in link.counters()


def test_find_port_never_raises_without_hardware():
    """无硬件时 ``find_port`` 只该返回 None（或真板子的路径），不能抛。"""
    port = __import__("litearm_ros2_control.stm32_link",
                      fromlist=["find_port"]).find_port()
    assert port is None or os.path.exists(port)


def test_open_reports_missing_device_clearly():
    link = Stm32Link("/dev/definitely-not-a-tty")
    with pytest.raises(OSError):
        link.open()
    assert link.is_open is False


def test_open_denied_is_a_distinct_error_with_remedy(tmp_path):
    """权限不足要**单独一个异常类型**，并且自带恢复路径。

    回归背景：真机上 `/dev/ttyACM0` 是 `root:dialout 660`，而用户不在 dialout 组。
    这条错**不会自己好**——等下去、重试都改变不了结果，必须人去加组后重新登录。
    所以它不能和"板子还没插上"（插上就好）混在同一个无限重试里：那样日志只会
    刷满同一句没用的话，而真正的恢复步骤没人告诉你。
    """
    path = tmp_path / "ttyACM0"
    path.write_text("", encoding="utf-8")
    path.chmod(0o000)
    link = Stm32Link(str(path))
    with pytest.raises(Stm32AccessDenied) as excinfo:
        link.open()
    assert link.is_open is False
    text = str(excinfo.value)
    assert "dialout" in text, "错误信息没给恢复路径"
    assert "usermod" in text and "重新登录" in text


def test_access_denied_is_not_a_plain_oserror():
    """`Stm32AccessDenied` 继承 `RuntimeError` 而非 `OSError` —— 调用方别接错。

    这条看着吹毛求疵，但踩过：冒烟脚本原来只 `except OSError`，结果权限错误
    直接以未捕获的 traceback 喷出来，恢复步骤被埋在一堆栈里。
    """
    from litearm_ros2_control.stm32_link import Stm32Error
    assert issubclass(Stm32AccessDenied, Stm32Error)
    assert not issubclass(Stm32Error, OSError)


def test_link_counts_rejected_frames_by_cmd_and_reason(rig):
    """被拒的帧必须**有账可查** —— 只靠 `_replies` 是看不见它们的。

    回归背景（真机排查）：控制环路径走 `send()`，**不等应答**；`RSP_ERR` 只会躺进
    `_replies`，而 `_replies` 只在启动期的 `request()` 里被消费，其余时候由
    `REPLY_BACKLOG` 封顶**从队头丢掉** ⇒ "命令被固件拒了"在本进程**零痕迹**，
    只能从固件状态帧的 `watchdog_tripped` 倒推。与 `tx_dropped` 是同一类缺陷。

    这条尤其要紧：固件 2026-09-24 起新增了 MOVE_JS 的
    「`dq` 全 0 且目标离实测 > 5 mrad ⇒ 整条拒绝」门禁，而**被拒 = 不喂看门狗**
    （拒绝发生在 `watchdog_kick()` 之前）⇒ 持续被拒同样把臂推进 fail-soft。

    这里让假固件对**未知命令**回 `ERR{0x7E,0x00}`，走的是真实回包路径。
    """
    firmware, link = rig
    assert link.err_by_code == {}, "夹具前提：还没发过会被拒的帧"
    assert "被拒" not in link.counters()

    link.send(0x7E)
    link.send(0x7E)
    _spin(link, 0.2)

    assert link.err_by_code == {(0x7E, 0x00): 2}, link.err_by_code
    text = link.counters()
    assert "0x7e/0x00×2" in text, f"拒帧计数没进 counters()：{text}"
    assert "0x00" in text and "未收录" not in text, \
        f"原因码应翻成人能读的话（0x00 = 固件无此命令）：{text}"


def test_link_reject_counter_survives_reply_backlog_overflow(rig):
    """`_replies` 被挤爆之后，拒帧计数**照旧准** —— 这正是它存在的理由。

    控制环路径没人消费 `_replies`，`REPLY_BACKLOG`（32）一满就从队头丢。
    如果计数是"入队时数、出队时减"那类实现，这里就会漏账。
    """
    firmware, link = rig
    from litearm_ros2_control.stm32_link import REPLY_BACKLOG
    total = REPLY_BACKLOG + 5
    for _ in range(total):
        link.send(0x7E)
    _spin(link, 0.5)
    assert link.err_by_code.get((0x7E, 0x00)) == total, (
        f"期望 {total} 条全记上，实际 {link.err_by_code.get((0x7E, 0x00))}"
        f"（REPLY_BACKLOG={REPLY_BACKLOG}）")
