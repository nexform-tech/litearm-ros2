#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""joint_trajectory_controller 与 URDF 的配置契约（期望值来自固件参数表）。

为什么期望值要来自固件
----------------------
换底层之后 PD 增益与跟踪判据都归固件所有：守护进程启动时用 ``0x24`` 把
``mit_kp/mit_kd/tau_max/限位`` 读回来（见 hw_daemon.py 的启动日志）。所以
ROS 侧对"刚度是多少、多紧算跟不上"这些问题**不该自己另有一套答案**。

本文件的三条契约：

1. **URDF 的 ``default_kp/default_kd`` 必须与固件的 ``mit_kp/mit_kd`` 逐值一致。**
   默认通道下真正生效的是固件那份（ROS 侧的 kp/kd 根本不转发）；URDF 那份只在
   ``--mit-passthrough`` 回退通道下写进命令帧。两处不一致时默认通道照常工作，
   而切到回退通道会突然换一套刚度——那正是最难查的一类问题。

2. **JTC 的跟踪/到位容差不得比固件自己的判据更紧。**
   JTC 一旦发现 ``|参考位置 − 实测位置|`` 超过 ``constraints.<joint>.trajectory``，
   就会 **中止整条轨迹**（``State tolerances failed``，``error_code=-4``），
   而不是只记一条警告；而固件的 ``safety_check`` 用 ``following_error`` 判定
   跟随超差（触发后置 ``joint_fault`` 并失能该轴）。比固件更紧 = **先于固件**
   误判失控。
   本工程踩过的坑：最初按"URDF 限位量级"随手设成 0.05 rad——比工程自身判据紧
   5~7 倍，真机上正常运动被误判为失控而中止，现象（轨迹跑一半停住 + -4）看起来
   像链路故障，不像配置问题。

3. **命令接口集合与默认通道一致**（position + velocity，不要 acceleration/effort）。

固件参数表在哪
--------------
它是**另一个仓库**（litearm-stm32），不在本工作区里，所以按约定位置找、允许
``LITEARM_STM32_ROOT`` 覆盖。找不到就 ``skip``；但**找到了却解析不出来要硬失败**
——那种情况下交叉校验已经名存实亡，静默跳过比没有测试更糟。
"""

import os
import re
import sys

import pytest
import yaml

NUM_JOINTS = 7

# ── 没有固件锚点的两个容差：显式常量 + 说明 ──────────────────────────────
# stopped_velocity_tolerance / goal_time 是 JTC 自己的概念，固件侧没有对应判据，
# 所以它们**不是交叉校验，而是回归护栏**：防止有人"顺手收紧"到真机误报超时。
# 取值来自真机整定（litearm.yaml 的 safety.stopped_velocity_rad_s / goal_timeout_s）。
JTC_STOPPED_VELOCITY_FLOOR = 0.03
JTC_GOAL_TIME_FLOOR = 3.0


# ───────────────────── 固件参数表（期望值来源） ─────────────────────


def _firmware_root():
    """定位 litearm-stm32 仓库；找不到返回 ``None``。

    候选（按优先级）：
      1. 环境变量 ``LITEARM_STM32_ROOT``
      2. ``~/wkspace/litearm-stm32``（本机约定位置）
      3. 工作区同级的 ``litearm-stm32``（布局不同时的兜底）
    """
    candidates = [os.environ.get("LITEARM_STM32_ROOT")]
    home = os.path.expanduser("~")
    candidates.append(os.path.join(home, "wkspace", "litearm-stm32"))
    # <ws>/src/<pkg>/test → 上溯到 <ws>，再找同级/上一级的 litearm-stm32
    here = os.path.dirname(os.path.abspath(__file__))
    workspace = os.path.dirname(os.path.dirname(os.path.dirname(here)))
    candidates.append(os.path.join(os.path.dirname(workspace), "litearm-stm32"))
    for path in candidates:
        if path and os.path.isdir(path):
            return path
    return None


def _float_field(block, name):
    """从一块 C 初始化文本里取 ``.name=1.0f`` 的数值。

    前导 ``\\.`` 是有意义的：它让 ``.tau_max`` 不会误匹配 ``.wall_tau_max``。
    """
    match = re.search(rf"\.{name}=(-?[\d.]+(?:[eE][-+]?\d+)?)f", block)
    assert match, f"固件参数表里找不到 .{name}=（解析器或源码结构变了）"
    return float(match.group(1))


def _parse_joint_table(text):
    """从 ``defaults.c`` 的文本里抽出 7 关节参数表。

    按 ``.can_id=0x`` 切块，再用 ``q_min=JL_QMIN_Jn`` 认关节号——**不能用
    can_id 认**：单关节台架表（``#if LITEARM_BENCH_1J``）的 can_id 也是 0x01，
    会把台架那组参数混进来；而台架表的 q_min 是字面量（-12.0f），拿这个判据
    天然把它排除掉。
    """
    joints = {}
    for block in text.split(".can_id=0x")[1:]:
        match = re.search(r"q_min=JL_QMIN_J(\d)", block)
        if not match:
            continue
        joints[int(match.group(1)) - 1] = {
            "kp": _float_field(block, "mit_kp"),
            "kd": _float_field(block, "mit_kd"),
            "tau_max": _float_field(block, "tau_max"),
            "following_error": _float_field(block, "following_error"),
        }
    return joints


def _load_firmware(root):
    """按给定仓库根读出参数表与编译期常量。

    失败模式是**刻意分开的**：

    * 文件不存在 → ``skip``（"这台机器上没有固件源码"，交叉校验无从谈起）；
    * 文件在场但解析不出 7 个关节 / 找不到常量 → ``fail``
      （"源码结构变了，解析器该跟上了"）——静默跳过等于把交叉校验变成一句空话，
      看起来有一条契约、实际什么都没查，比没有测试更糟。
    """
    defaults_c = os.path.join(root, "User", "litearm", "params", "defaults.c")
    control_c = os.path.join(root, "User", "litearm", "control",
                             "control_loop.c")
    for path in (defaults_c, control_c):
        if not os.path.exists(path):
            pytest.skip(f"litearm-stm32 仓库在场但缺少 {path}")

    with open(defaults_c, encoding="utf-8") as handle:
        joints = _parse_joint_table(handle.read())
    if sorted(joints) != list(range(NUM_JOINTS)):
        pytest.fail(
            f"从 {defaults_c} 解析出 {sorted(joints)} 号关节，期望 0..{NUM_JOINTS - 1}。"
            f"仓库在场却解析不出来 = 交叉校验已经失效，**比没有测试更糟**，"
            f"所以这里硬失败而不是 skip。请更新解析器或确认源码结构。")

    with open(control_c, encoding="utf-8") as handle:
        control_text = handle.read()
    match = re.search(r"#define\s+LITEARM_HT_ERR_TOL\s+([\d.]+)f", control_text)
    if not match:
        pytest.fail(f"{control_c} 里找不到 LITEARM_HT_ERR_TOL（到位交接的残差闸）")

    return {
        "root": root,
        "defaults_c": defaults_c,
        "joints": joints,
        "hold_err_tol": float(match.group(1)),
    }


@pytest.fixture(scope="module")
def firmware():
    """固件参数表 + 编译期安全常量（见 :func:`_load_firmware` 的失败模式）。"""
    root = _firmware_root()
    if root is None:
        pytest.skip("找不到 litearm-stm32 仓库（设 LITEARM_STM32_ROOT 可指定）；"
                    "固件参数表交叉校验只在源码在场时有意义")
    return _load_firmware(root)


@pytest.fixture(scope="module")
def jtc_params():
    path = os.path.join(os.path.dirname(__file__), "..",
                        "config", "litearm_controllers.yaml")
    with open(path, encoding="utf-8") as handle:
        config = yaml.safe_load(handle)
    return path, config["joint_trajectory_controller"]["ros__parameters"]


def _parse_xacro_double_list(xacro_path, arg_name):
    with open(xacro_path, encoding="utf-8") as handle:
        text = handle.read()
    match = re.search(rf'<xacro:arg name="{arg_name}" default="([^"]+)"', text)
    assert match, f"{xacro_path} 里找不到 {arg_name}"
    return [float(v) for v in match.group(1).split(",") if v.strip()]


# ───────────────── 契约 1：URDF 增益 == 固件增益 ─────────────────


def test_default_gains_match_firmware(firmware):
    """URDF 的 default_kp/default_kd 必须与固件参数表逐值一致。

    两个来源彼此独立（改固件不会动 URDF、改 URDF 不会动固件），而整条链路按
    "同源整定"使用它们。任何一侧漂移都会静默改变真机刚度，所以锁死逐值相等。

    回归背景：本用例的前身是 ``test_default_gains_match_pylitearm_control``，
    当时它**一直在报红**：pylitearm 的 ``control.kd`` J7 被改成 1.5，而 URDF 是
    2.5。对照固件 ``defaults.c`` 的 ``mit_kd`` J7 = 2.5 —— 是 pylitearm 那份漂了，
    固件与 ROS 侧一直是对的。换期望源之后这条契约自然一致（并顺带消掉那个红）。
    """
    xacro_path = os.path.join(os.path.dirname(__file__), "..",
                              "urdf", "litearm.ros2_control.xacro")
    problems = []
    for name, key in (("litearm_default_kp", "kp"), ("litearm_default_kd", "kd")):
        expected = [firmware["joints"][i][key] for i in range(NUM_JOINTS)]
        actual = _parse_xacro_double_list(xacro_path, name)
        if actual != expected:
            problems.append(f"{name}={actual} vs 固件 mit_{key}={expected}")
    assert not problems, (
        f"{xacro_path} 的默认 MIT 增益与固件参数表（{firmware['defaults_c']}）"
        f"不一致：\n  " + "\n  ".join(problems)
        + "\n默认通道下真正生效的是固件那份；URDF 这份只在 --mit-passthrough "
          "回退通道下生效。两侧必须同步。")


# ───────────── 契约 2：JTC 容差不得比固件判据更紧 ─────────────


def test_jtc_trajectory_tolerance_not_tighter_than_firmware(firmware, jtc_params):
    """JTC 的 trajectory 容差 >= 固件的 following_error。

    单向不变式：放松（往安全方向改）不会失败，收紧会。
    """
    path, params = jtc_params
    constraints = params.get("constraints", {})
    problems = []
    for index in range(NUM_JOINTS):
        joint = f"joint{index + 1}"
        limit = constraints.get(joint, {}).get("trajectory")
        if limit is None:
            problems.append(f"{joint}: 未设置 trajectory 容差")
            continue
        reference = firmware["joints"][index]["following_error"]
        if float(limit) < reference:
            problems.append(
                f"{joint}: JTC trajectory={limit} < 固件 following_error={reference}")
    assert not problems, (
        f"{path} 的跟踪容差比固件 safety_check 的判据更紧——真机上正常运动会"
        f"**先于固件**被 JTC 误判为失控并中止整条轨迹（error_code=-4）：\n  "
        + "\n  ".join(problems)
        + f"\n期望值来源：{firmware['defaults_c']} 的 joint[].following_error。")


def test_jtc_goal_tolerance_not_tighter_than_firmware_handoff(firmware, jtc_params):
    """JTC 的 goal 容差 >= 固件到位交接的残差闸（``LITEARM_HT_ERR_TOL``）。

    这两者必须在**同一量级**：固件在参考跑完后按这个残差闸决定"可以升刚持位了"
    （``control_loop.c`` 的 hold-handoff），而 JTC 用 goal 容差决定"到底到位没有"。
    JTC 明显更紧时，会出现"固件已经宣布到位并升刚、JTC 还在等"的错位——表现为
    到位判定超时，而臂其实早就稳住了。
    """
    path, params = jtc_params
    constraints = params.get("constraints", {})
    reference = firmware["hold_err_tol"]
    problems = []
    for index in range(NUM_JOINTS):
        joint = f"joint{index + 1}"
        goal = constraints.get(joint, {}).get("goal")
        if goal is None:
            problems.append(f"{joint}: 未设置 goal 容差")
        elif float(goal) < reference:
            problems.append(
                f"{joint}: goal={goal} < 固件 LITEARM_HT_ERR_TOL={reference}")
    assert not problems, (
        f"{path} 的到位容差比固件到位交接的残差闸更紧；固件升刚持位后 JTC 仍会"
        f"判定「未到位」，表现是到位超时而臂其实是稳的：\n  "
        + "\n  ".join(problems))


def test_jtc_stopped_velocity_and_goal_time_stay_loose(jtc_params):
    """stopped_velocity / goal_time 不得被"顺手收紧"。

    ⚠ 这两条**不是交叉校验**：它们是 JTC 自己的概念，固件侧没有对应判据。
    这里比的是真机整定时定下的下限（模块常量），作用是**回归护栏**——
    防止有人为了"看起来更精确"把它们改小，而后果是到位判定频繁误报超时。
    """
    path, params = jtc_params
    constraints = params.get("constraints", {})
    stopped = constraints.get("stopped_velocity_tolerance")
    goal_time = constraints.get("goal_time")
    assert stopped is not None, f"{path} 未设置 stopped_velocity_tolerance"
    assert goal_time is not None, f"{path} 未设置 constraints.goal_time"
    assert float(stopped) >= JTC_STOPPED_VELOCITY_FLOOR, (
        f"{path} 的 stopped_velocity_tolerance={stopped} 比真机整定下限 "
        f"{JTC_STOPPED_VELOCITY_FLOOR} 更紧：判定「已停住」更苛刻，"
        f"到位判定容易被误判为超时")
    assert float(goal_time) >= JTC_GOAL_TIME_FLOOR, (
        f"{path} 的 goal_time={goal_time}s 短于真机整定下限 "
        f"{JTC_GOAL_TIME_FLOOR}s：到位窗口更窄，容易误判超时")


# ───────────── 契约 3：命令接口集合 ─────────────


def test_jtc_command_interfaces_are_position_and_velocity(jtc_params):
    """JTC 只声明 position + velocity 命令接口（含顺序）。

    换底层之前这里是 [position, velocity, acceleration]，理由是"守护进程要用
    命令帧的 acceleration 算 M(q)·q̈ 前馈"。**那个理由已经不成立了**：
    加速度前馈算在固件里，而且只在 MOVE_J 模式下算——MOVE_JS 没有加速度源
    （固件源码注释原文："无加速度源(梯形限幅, 非 S 曲线): M·ddq 不猜"）。
    默认通道不会把 acceleration 转发给固件，所以声明它只会让人以为惯量前馈开着。

    同理不要 effort：默认通道的 MOVE_JS **刻意不带 tau_ff**，带上就会关掉固件
    整套内置前馈（builtin_mode 要求 `MOVE_JS && !s_js_user_ff`）。

    真需要这两条通道（自己算前馈）请用守护进程的 mit-passthrough 开关，
    并把这里改成相应的接口组合。
    """
    path, params = jtc_params
    interfaces = params.get("command_interfaces")
    assert interfaces == ["position", "velocity"], (
        f"{path} 的 joint_trajectory_controller.command_interfaces="
        f"{interfaces}，期望 [position, velocity]：默认的 MOVE_JS 通道只消费"
        f"这两个字段，acceleration/effort 不会被转发到固件")


def test_default_gains_are_in_mit_range(jtc_params):
    """kp/kd 的默认值必须落在达妙 MIT 帧的可表示范围内。

    这两个默认值来自 litearm.ros2_control.xacro，插件会按
    kp∈[0,500]、kd∈[0,5] 钳位并告警；这里提前拦住"配了个根本表达不了的刚度"。
    """
    xacro_path = os.path.join(os.path.dirname(__file__), "..",
                              "urdf", "litearm.ros2_control.xacro")
    for name, high in (("litearm_default_kp", 500.0), ("litearm_default_kd", 5.0)):
        values = _parse_xacro_double_list(xacro_path, name)
        assert len(values) == NUM_JOINTS, f"{name} 不是 {NUM_JOINTS} 个数: {values}"
        bad = [v for v in values if not (0.0 <= v <= high)]
        assert not bad, f"{name} 含超出 MIT 范围的取值（应 ∈[0,{high}]）: {bad}"


# ───────────────────── 期望源加载器自身的回归锁 ─────────────────────


def _load_module_by_path(path, name):
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_moveit_joint_limits_in_sync_with_firmware(firmware):
    """MoveIt 的 ``joint_limits.yaml`` 必须与固件参数表同步（跨包不变量）。

    规划空间（MoveIt）与运行期安全包络（固件）**必须同源**：固件的软限位比 URDF
    收窄 1°，如果 MoveIt 用的是 URDF 原始值，就会规划出界、被固件的位置包络拒绝
    ——表现为"规划成功但执行失败"，而且看起来像链路问题。

    这里不是重写一份渲染逻辑来比（那样两边会一起漂），而是**直接调用生成器**，
    所以工具本身坏了也会在这里暴露。
    """
    tool_path = os.path.join(os.path.dirname(__file__), "..", "..",
                             "litearm_moveit_config", "tools",
                             "gen_from_firmware_config.py")
    config_path = os.path.join(os.path.dirname(__file__), "..", "..",
                               "litearm_moveit_config", "config",
                               "joint_limits.yaml")
    for path in (tool_path, config_path):
        if not os.path.exists(path):
            pytest.skip(f"缺 {path}（本测试需要 litearm_moveit_config 同在工作区）")

    module = _load_module_by_path(tool_path, "gen_from_firmware_config")
    expected = module.render_joint_limits(
        module.load_joints(firmware["root"]))
    with open(config_path, encoding="utf-8") as handle:
        actual = handle.read()
    assert actual == expected, (
        f"{config_path} 与固件参数表不同步。\n"
        f"  请运行：python3 litearm_moveit_config/tools/"
        f"gen_from_firmware_config.py\n"
        f"  （规划空间与固件安全包络不同源时，会出现「规划成功但执行失败」）")


def test_firmware_loader_fails_loudly_when_repo_is_unparsable(tmp_path):
    """仓库在场却解析不出来 → ``fail``，不是 ``skip``。

    这条锁的是**设计决定**：静默跳过会把交叉校验变成一句空话（看起来有 7 条
    契约，实际一条都没查），而缺陷恰恰要等到有人改了固件参数才暴露。
    """
    root = tmp_path / "litearm-stm32"
    (root / "User" / "litearm" / "params").mkdir(parents=True)
    (root / "User" / "litearm" / "control").mkdir(parents=True)
    (root / "User" / "litearm" / "params" / "defaults.c").write_text(
        "/* 结构变了，解析器应当跟上来 */\n", encoding="utf-8")
    (root / "User" / "litearm" / "control" / "control_loop.c").write_text(
        "#define LITEARM_HT_ERR_TOL 0.02f\n", encoding="utf-8")
    with pytest.raises(pytest.fail.Exception) as excinfo:
        _load_firmware(str(root))
    assert "解析出" in str(excinfo.value)
    assert "硬失败" in str(excinfo.value), "错误信息应说清为什么不是 skip"


def test_firmware_loader_skips_only_when_files_are_absent(tmp_path):
    """文件压根不在 → ``skip``（这台机器上没有固件源码，无从校验）。"""
    root = tmp_path / "litearm-stm32"
    root.mkdir()
    with pytest.raises(pytest.skip.Exception) as excinfo:
        _load_firmware(str(root))
    assert "缺少" in str(excinfo.value)


def test_firmware_parser_rejects_bench_table():
    """解析器必须只收 7 关节整臂表，不能把单关节台架表混进来。

    这条是**解析器自身**的回归锁：台架表的 can_id 也是 0x01，靠 can_id 认关节
    会把它那组增益（100.0/3.0）混进来，症状是"URDF 与固件不一致"的假告警。
    """
    root = _firmware_root()
    if root is None:
        pytest.skip("找不到 litearm-stm32 仓库")
    joints = _load_firmware(root)["joints"]
    kp = [joints[i]["kp"] for i in range(NUM_JOINTS)]
    assert kp == [400.0, 400.0, 300.0, 300.0, 50.0, 50.0, 50.0], (
        f"解析出的 kp={kp} 不是整臂表那组；是不是混进了台架表（100.0/3.0）？")


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
