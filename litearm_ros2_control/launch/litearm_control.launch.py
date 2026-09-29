#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_control.launch.py — 起整套 ros2_control 控制栈。

进程拓扑：

    litearm_hw_daemon        ← 独占 USB CDC，持有到 litearm-stm32 固件的链路，
      ↕ POSIX shm（seqlock 双缓冲）   创建共享内存段
    ros2_control_node        ← controller_manager + LitearmSystem 插件
      ├─ joint_state_broadcaster
      └─ joint_trajectory_controller

守护进程先于 ros2_control_node 拉起；插件在 on_configure 里等守护进程就绪，
超时或失败时会带上守护进程给出的具体原因（端口没找到 / license 未激活 /
电机反馈未就绪）。

命令通道默认走固件的 **MOVE_JS 位置模式**：PD 与重力/摩擦/积分/kd_extra 前馈
都由固件算（模型由 URDF 生成、编译进固件）。想自己算前馈就用
``mit_passthrough:=true`` 切到 MIT_ALL 全透传。

CPU 亲和（PREEMPT_RT + GRUB isolcpus=2,3）：两个实时进程默认钉到隔离核 ——
守护进程（100 Hz）→ CPU 2（与 USB 中断同核，配合 rt_env.sh irq-pin），
ros2_control_node（250 Hz，与守护进程 1:1；JSB/JTC 是其进程内线程）→ CPU 3
（控制环独占，不受 USB 中断打扰）；用 daemon_cpu:= / control_cpu:= 覆盖，传空串 = 不绑定。

两种运行方式：

  真机    ros2 launch litearm_ros2_control litearm_control.launch.py
          （板子已上电、USB 已连、**license 已激活**——未激活时 ENABLE 会被拒）

  无硬件  ros2 launch litearm_ros2_control litearm_control.launch.py dry_run:=true
          守护进程在 pty 上起一块假固件，**链路仍走真实协议**；
          用于验证 URDF / 控制器 / 话题 / 协议全链路；一阶运动学模型，
          不能用于整定增益。
"""

import os

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from litearm_ros2_control.ros_env import isolation_hint, parse_bool, ros_isolation_env


def _is_true(value: str) -> bool:
    return parse_bool(value, False)


def _declare_arguments():
    return [
        DeclareLaunchArgument(
            "port", default_value="",
            description="litearm-stm32 的 USB CDC 设备路径；"
                        "留空 = 按 VID:PID 1d50:606f 自动发现"),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true = 无硬件模式（守护进程在 pty 上起一块假固件，"
                        "链路仍走真实协议；一阶运动学模型，不能整定增益）"),
        DeclareLaunchArgument(
            "start_daemon", default_value="true",
            description="false = 复用已在运行的守护进程（联调时常这么用）"),
        DeclareLaunchArgument(
            "hw_config", default_value="",
            description="可选的 litearm_hw.yaml 路径：只放 PC 侧参数"
                        "（端口/频率/超时/策略）。留空 = 只用本文件的参数。"
                        "注意**命令行优先于该文件**"),
        DeclareLaunchArgument(
            "shm_name", default_value="/litearm_hw",
            description="共享内存段名，必须与插件 URDF 参数一致"),
        DeclareLaunchArgument(
            "daemon_rate_hz", default_value="0",
            description="守护进程命令下发频率；0 = 用守护进程默认（250Hz）"),
        DeclareLaunchArgument(
            "daemon_cpu", default_value="2",
            description="守护进程绑定的 CPU 核（taskset -c）；空 = 不绑定。"
                        "默认 2 = 隔离核，与 USB 中断（rt_env.sh irq-pin）同核"),
        DeclareLaunchArgument(
            "control_cpu", default_value="3",
            description="ros2_control_node 绑定的 CPU 核（taskset -c）；空 = 不绑定。"
                        "默认 3 = 隔离核，控制环独占、不受 USB 中断打扰"),
        # 五个前馈开关现在是**三态**：默认空 = 不碰固件（固件自己有出厂掩码），
        # true/false = 显式置位/清位。默认不覆盖的理由见 hw_daemon.py 的
        # 「前馈覆盖」一节：关节级参数的唯一真源在固件，守护进程不该悄悄改它。
        DeclareLaunchArgument(
            "gravity_compensation", default_value="",
            description="覆盖固件 ff_mask 的 FF_G 位：空 = 不碰固件，"
                        "true/false = 显式置位/清位"),
        DeclareLaunchArgument(
            "friction_compensation", default_value="",
            description="覆盖固件 ff_mask 的 FF_FRICTION 位（摩擦 v1/v2/drag）"),
        DeclareLaunchArgument(
            "inertia_compensation", default_value="",
            description="覆盖固件 ff_mask 的 FF_INERTIA|FF_CORIOLIS 位"
                        "（⚠ 默认 MOVE_JS 通道下固件不算惯量项，置位无用）"),
        DeclareLaunchArgument(
            "integral_compensation", default_value="",
            description="覆盖固件 ff_mask 的 FF_INTEGRAL 位（ki·∫e dt）"),
        DeclareLaunchArgument(
            "damping_compensation", default_value="",
            description="覆盖固件 kd_extra 向量：false = 清零，"
                        "true = 恢复出厂 6/6/6/6/0/0/0"),
        DeclareLaunchArgument(
            "mit_passthrough", default_value="false",
            description="true = 切到 MIT_ALL 全透传通道：kp/kd/effort 逐帧生效、"
                        "固件不叠任何自家前馈（默认 MOVE_JS 位置模式）"),
        DeclareLaunchArgument(
            "exit_hold_s", default_value="2.0",
            description="退出前继续下发持位参考的秒数（0 = 直接 park 退出）"),
        DeclareLaunchArgument(
            "disable_on_shutdown", default_value="false",
            description="true = 退出时请求失能电机（臂会失力下坠，务必先支撑）"),
        DeclareLaunchArgument(
            "use_rviz", default_value="false",
            description="是否附带启动 RViz2"),
        DeclareLaunchArgument(
            "ros_domain_id", default_value="42",
            description="本栈使用的 ROS 域（默认刻意避开 0）。要与外部系统对接时设 0"),
        DeclareLaunchArgument(
            "ros_localhost_only", default_value="true",
            description="true = 只在本机发现。跨机串扰会污染 robot_description / "
                        "joint_states / move_action，默认开启隔离"),
        # ── 机器人描述（可选覆盖）──────────────────────────────────────────
        # 默认用本包自带的纯臂描述。要用夹爪时传一份同时含臂与夹爪两个
        # <ros2_control> 块的描述（litearm_manipulation 的 litearm_gripper.urdf.xacro）。
        DeclareLaunchArgument(
            "urdf_xacro", default_value="",
            description="可选的 xacro 路径，覆盖默认的纯臂描述"),
        # ── LiteGrip 夹爪（默认关，保持纯臂行为逐字不变）────────────────
        # ★ 夹爪是 ros2_control 硬件组件（LitegripSystem，litegrip_cpp 的薄壳），
        #   **没有独立守护进程、没有共享内存**。下面这些参数不是"传给某个进程"，
        #   而是**注入 URDF**（xacro 的 litegrip_* arg）—— 组件在 on_configure 里读它们。
        DeclareLaunchArgument(
            "start_gripper", default_value="false",
            description="true = 描述里多一个夹爪 <ros2_control> 块（加载其硬件组件）"
                        "+ 起 gripper_controller"),
        DeclareLaunchArgument(
            "gripper_channel", default_value="can0",
            description="夹爪 CAN 接口（STM32 的 gs_usb 桥；需 Classic CAN 1 Mbit/s）"),
        DeclareLaunchArgument(
            "gripper_dry_run", default_value="",
            description="空 = 跟随臂的 dry_run；也可显式 true/false 单独控制"
                        "（夹爪自己走 SDK 的模拟被控对象，不碰 can0）"),
        DeclareLaunchArgument(
            "gripper_hardware_enable", default_value="false",
            description="夹爪真机总闸；必须与（有效的）dry_run=false 同时成立才会驱动电机"),
        DeclareLaunchArgument(
            "gripper_feedback_velocity_rad_s", default_value="-1.0",
            description="最坏情况反馈速度界（rad/s）；-1 = 没给 ⇒ **真机拒绝发任何运动帧**"
                        "（刻意 fail-closed；正式抓取前必须现场标定）。dry_run 下本 launch 用"
                        "命令速率上限顶替它（推导值，不是编的）"),
        DeclareLaunchArgument(
            "gripper_max_position_error_rad", default_value="0.15",
            description="最坏情况**位置误差界**（rad）：力矩预算拿它当 kp 的分母"
                        "（kp=(预算−kd·v_b)/e_b），驱动侧的滞后守卫在 |目标−实测| 超过它时锁存。"
                        "-1 = 由红线宽度推导（1.23 ⇒ kp 只剩 2.4）"),
        DeclareLaunchArgument(
            "gripper_max_velocity_rad_s", default_value="1.5",
            description="命令轨迹速率上限（rad/s）；只能往下调（SDK 的硬上限就是 1.5）。"
                        "★ urdf/litearm_gripper.urdf.xacro 的 grip_opening_vel 必须与它同步"
                        "（= 本值 × rad_to_mm / 1000），litearm_manipulation 的 "
                        "test_gripper_geometry 逐值核对"),
        DeclareLaunchArgument(
            "gripper_torque_limit_nm", default_value="3.5",
            description="总控制力矩预算（N·m）；只能往下调（DM-J4310-2EC 额定 3.5）。"
                        "⚠ 真爪按抓取负载标定前建议调低"),
        DeclareLaunchArgument(
            "gripper_safety_baseline", default_value="3.5",
            description="安全基线版本（或一份基线文件的显式路径）；未知版本/文件缺失时"
                        "组件**拒绝启动**，而不是退回一个没人确认过的默认值"),
    ]


def _tri_state_flag(name: str, raw: str) -> list:
    """把三态 launch 参数翻成守护进程开关。

    ``""`` = 不传（不碰固件）；``"true"`` → ``--name-compensation``；
    ``"false"`` → ``--no-name-compensation``。
    没有 ``--no-`` 那一半就没法做"退回纯 PD"的 A/B 对照。
    """
    text = raw.strip().lower()
    if not text:
        return []
    prefix = "--" if parse_bool(text, False) else "--no-"
    return [f"{prefix}{name}-compensation"]


def _launch_setup(context, *_args, **_kwargs):
    """在 launch 上下文里解析参数并组装动作。"""
    pkg_share = get_package_share_directory("litearm_ros2_control")
    daemon_exe = os.path.join(get_package_prefix("litearm_ros2_control"),
                              "lib", "litearm_ros2_control", "litearm_hw_daemon")

    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    port = resolve("port").strip()
    shm_name = resolve("shm_name")
    dry_run = _is_true(resolve("dry_run"))
    disable_on_shutdown = _is_true(resolve("disable_on_shutdown"))
    mit_passthrough = _is_true(resolve("mit_passthrough"))
    exit_hold_s = resolve("exit_hold_s").strip()
    rate_hz = resolve("daemon_rate_hz").strip() or "0"
    daemon_cpu = resolve("daemon_cpu").strip()
    control_cpu = resolve("control_cpu").strip()
    start_daemon = _is_true(resolve("start_daemon"))
    start_gripper = _is_true(resolve("start_gripper"))
    domain_id = resolve("ros_domain_id").strip() or "42"
    localhost_only = resolve("ros_localhost_only")
    env = ros_isolation_env(domain_id, localhost_only)
    use_rviz = _is_true(resolve("use_rviz"))

    # ── LiteGrip 夹爪的派生值 ──
    # 这些值最终**注入 URDF**（xacro 的 litegrip_* arg），由硬件组件在 on_configure 读。
    # gripper_dry_run 留空 = 跟随臂：同一次演练里两边都是模拟的；也能单独覆盖成
    # 「臂假体 + 爪真机」（dry_run:=true gripper_dry_run:=false）。
    gripper_dry_raw = resolve("gripper_dry_run").strip()
    gripper_dry = dry_run if not gripper_dry_raw else _is_true(gripper_dry_raw)
    gripper_hw_enable = _is_true(resolve("gripper_hardware_enable"))
    rate_text = resolve("gripper_max_velocity_rad_s").strip() or "1.5"
    bound_text = resolve("gripper_feedback_velocity_rad_s").strip() or "-1.0"
    try:
        bound = float(bound_text)
    except ValueError:
        bound = -1.0
    # 反馈速度界缺省（-1 = 没给）时：SDK 会**拒绝发任何运动帧**（刻意 fail-closed）。
    # dry_run 下可以用「命令速率上限」顶替 —— 模拟被控对象的实测速度就是被命令的速率，
    # 所以那是个**推导值**，不是编的。真机路径**不**顶替，只把话说清楚。
    gripper_notes = []
    effective_bound = bound_text
    if bound <= 0.0 and gripper_dry:
        effective_bound = rate_text
        gripper_notes.append(LogInfo(msg=(
            "[litegrip] dry run：gripper_feedback_velocity_rad_s 没给，"
            f"用命令速率上限 {rate_text} rad/s 代表模拟被控对象的实测速度。")))
    elif bound <= 0.0:
        gripper_notes.append(LogInfo(msg=(
            "[litegrip] ⚠ gripper_feedback_velocity_rad_s 没给 —— 控制环会**拒绝发送任何"
            "运动帧**，夹爪不会动。请在真机上标定它"
            "（让夹爪跑一遍真实工况，取峰值反馈速度再加余量）后传进来。")))

    # ── 展开 xacro 得到含 <ros2_control> 的完整 URDF ──
    # shm_name 与 disable_on_shutdown 必须与守护进程侧一致，所以走同一份
    # launch 参数注入，避免"启动参数改了但 URDF 里还是旧的"这类隐性错配。
    #
    # ★ urdf_xacro 默认是本包自带的**纯臂**描述。要用夹爪时由调用方传一份
    #   同时含臂与夹爪两个 <ros2_control> 块的描述（litearm_manipulation 的
    #   litearm_gripper.urdf.xacro）—— 这样本包**不必反向依赖**操纵包的布局，
    #   「描述由谁组装」这件事留在调用方。
    urdf_xacro = resolve("urdf_xacro").strip() or os.path.join(
        pkg_share, "urdf", "litearm.urdf.xacro")
    # ★★★ robot_description 必须显式声明成 str。
    #   launch 对"没写类型的参数"会拿 YAML 去推断值：URDF 是长文本，里面任何一行
    #   XML 注释带 "……: ……"（例如 <!-- ★ Loaded as a plugin: ... -->）就会让
    #   yaml.safe_load 抛 ScannerError，报成 "Failed to convert '<整份 URDF>'"。
    #   那行注释是谁写的不重要 —— 这里把类型钉死，URDF 内容就再也不会被 YAML 解读。
    robot_description = ParameterValue(xacro.process_file(
        urdf_xacro,
        mappings={
            "litearm_shm_name": shm_name,
            "litearm_disable_on_shutdown": "true" if disable_on_shutdown else "false",
            # 夹爪：只在调用方传了含夹爪的 xacro 时才有对应的 arg；纯臂描述时这些
            # mapping 无人使用（xacro 不报错）。gripper:=off 也不会真加载组件 ——
            # 组件块是描述文件里带来的，不是这里开关出来的。
            "litegrip_dry_run": "true" if gripper_dry else "false",
            "litegrip_hardware_enable": "true" if gripper_hw_enable else "false",
            "litegrip_channel": resolve("gripper_channel").strip() or "can0",
            "litegrip_max_feedback_velocity_rad_s": effective_bound,
            "litegrip_max_position_error_rad":
                resolve("gripper_max_position_error_rad").strip() or "-1.0",
            "litegrip_max_velocity_rad_s": rate_text,
            "litegrip_torque_limit_nm": resolve("gripper_torque_limit_nm").strip() or "3.5",
            "litegrip_safety_baseline": resolve("gripper_safety_baseline").strip() or "3.5",
        },
    ).toxml(), value_type=str)

    # ── 硬件守护进程 ──
    # 关节级参数（kp/kd/tau_max/限位/前馈）不在这里传：它们的唯一真源是固件，
    # 守护进程启动时用 0x24 / 0x2B / 0x2C 读回。这里只传 PC 侧概念，
    # 外加"要不要显式覆盖固件前馈"这类策略开关。
    daemon_cmd = [daemon_exe, "--shm-name", shm_name]
    hw_config = resolve("hw_config").strip()
    if hw_config:
        daemon_cmd += ["--hw-config", hw_config]
    if port:
        daemon_cmd += ["--port", port]
    if dry_run:
        daemon_cmd += ["--dry-run"]
    if mit_passthrough:
        daemon_cmd += ["--mit-passthrough"]
    for name in ("gravity", "friction", "inertia", "integral", "damping"):
        daemon_cmd += _tri_state_flag(name, resolve(f"{name}_compensation"))
    if exit_hold_s:
        daemon_cmd += ["--exit-hold-s", exit_hold_s]
    if rate_hz != "0":
        daemon_cmd += ["--rate-hz", rate_hz]
    daemon = ExecuteProcess(
        cmd=daemon_cmd, output="screen", additional_env=env,
        # 钉到隔离核（taskset -c）：250 Hz 周期不被其它任务抢 CPU、不被迁移。
        prefix=f"taskset -c {daemon_cpu}" if daemon_cpu else None,
        condition=IfCondition(LaunchConfiguration("start_daemon")))

    # ── LiteGrip 夹爪的控制器配置（可选，start_gripper:=true）──
    # ★ 夹爪**没有**独立进程：它的硬件组件就是 ros2_control_node 进程里的一个插件
    #   （litegrip_ros2_control/LitegripSystem，litegrip_cpp 的薄壳），参数已在上面
    #   注入 URDF。这里只需要把夹爪自己的**控制器声明**也喂给同一个 controller_manager。
    #   ⚠ 千万不要在这里加一个"夹爪守护进程"：那条路（Python + seqlock 共享内存 +
    #     litegrip_hw_daemon）已随 C++ 重写整体删除，那个可执行文件不存在。
    gripper_controllers_file = None
    if start_gripper:
        gripper_share = get_package_share_directory("litegrip_ros2_control")
        gripper_controllers_file = os.path.join(
            gripper_share, "config", "litegrip_controllers.yaml")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        additional_env=env,
        parameters=[{"robot_description": robot_description}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        additional_env=env,
        # 钉到隔离核：250 Hz 控制环与 JSB/JTC（同进程线程）都留在本核。
        prefix=f"taskset -c {control_cpu}" if control_cpu else None,
        parameters=[
            {"robot_description": robot_description},
            os.path.join(pkg_share, "config", "litearm_controllers.yaml"),
        ] + ([gripper_controllers_file] if gripper_controllers_file else []),
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen",
            additional_env=env,
        )

    spawn_jsb = spawner("joint_state_broadcaster")
    spawn_jtc = spawner("joint_trajectory_controller")
    # 夹爪控制器（start_gripper:=true 才有）。它认领的是 gripper_opening_joint，
    # 与臂的关节**不同名**，所以不存在接口争用，可以和臂的控制器同时激活。
    # 名字刻意叫 gripper_controller：与 MoveIt 侧 moveit_controllers.yaml 里那个
    # 同名 controller 对上（JTC 提供同名 action），上层迁移时 MoveIt 侧零改动。
    spawn_gripper = (spawner("gripper_controller") if start_gripper else None)
    # 伺服控制器**加载但不激活**（--inactive）：它和 JTC 都认领 joint1..7 的
    # position+velocity，同一时刻只能有一个激活（ros2_control 的接口独占），
    # 所以先挂在那里，真要用时用 servo_switch.py 显式交接。
    # 不激活时它不认领任何接口，对现有链路零影响。
    spawn_servo_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["litearm_servo_controller", "--inactive",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
        output="screen",
        additional_env=env,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz", "litearm_control.rviz")],
        output="log",
        additional_env=env,
        # RobotModel 显示走 /robot_description 话题（配置里设了 Transient Local
        # 以匹配 RSP 的锁存发布）；这里的参数是给需要它的插件兜底。
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    banner_target = ("dry-run（pty 假固件，链路走真实协议）" if dry_run
                     else f"真机，端口={port or '(自动发现 1d50:606f)'}")
    banner_channel = ("MIT_ALL 全透传" if mit_passthrough
                      else "MOVE_JS 位置模式（固件算 PD + 内置前馈）")
    # 夹爪模式写进横幅：两个开关（dry_run / hardware_enable）必须一眼可见 ——
    # 它们决定了"这夹爪到底会不会动真硬件"。
    banner_gripper = (
        "未加载（纯臂描述）" if not start_gripper
        else ("mock（SDK 模拟被控对象，不碰 can0）" if gripper_dry
              else ("真机 + hardware_enable=true（会驱动电机）" if gripper_hw_enable
                    else "真机但 hardware_enable=false（只读反馈，绝不使能）")))
    actions = [
        LogInfo(msg=(
            "──────── litearm 控制栈 ────────\n"
            f"  {banner_target}\n"
            f"  命令通道：{banner_channel}\n"
            f"  夹爪：{banner_gripper}\n"
            f"  CPU 亲和：守护进程 → {daemon_cpu or '不绑定'}，"
            f"ros2_control_node → {control_cpu or '不绑定'}\n"
            "  ⚠ 本 launch 已锁 ROS 域；想让 ros2 命令行看到本栈，请在\n"
            "    你自己的终端里执行：\n"
            f"        {isolation_hint(domain_id, localhost_only)}\n"
            "────────────────────────────────")),
        daemon,
        # 夹爪相关提示（dry-run 下顶替反馈速度界 / 真机缺反馈速度界的警告），紧跟在横幅后
        *gripper_notes,
        robot_state_publisher,
        rviz,
        # 守护进程要开端口、读固件参数、等使能（反馈齐了才加磁，最多约 5s）；
        # 先让它跑起来，免得插件的 on_configure 一上来就撞在"还没有心跳"上。
        TimerAction(period=1.5, actions=[ros2_control_node]),
        # 等 ros2_control_node 真正就绪再 spawn，避免 spawner 空等超时。
        RegisterEventHandler(OnProcessStart(target_action=ros2_control_node,
                                            on_start=[spawn_jsb])),
        # JSB 起来后再起 JTC：JTC 依赖 state_interfaces 已被广播。
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb,
                                           on_exit=[spawn_jtc])),
        # 伺服控制器跟在 JTC 之后加载（不激活）：
        # spawner 内部是 load+configure(+activate)，多个 spawner 并行容易撞在
        # 同一份参数/插件加载上，串起来最省心。--inactive 不认领接口，
        # 所以不影响 JTC 已经拿到的那些。
        RegisterEventHandler(OnProcessExit(target_action=spawn_jtc,
                                           on_exit=[spawn_servo_inactive])),
        # 夹爪控制器最后挂（同样串行）。它认领 gripper_opening_joint ——
        # 与臂的关节不同名，不争接口，所以可以和臂的控制器同时激活。
        *([RegisterEventHandler(OnProcessExit(target_action=spawn_servo_inactive,
                                              on_exit=[spawn_gripper]))]
          if spawn_gripper is not None else []),
    ]
    if not start_daemon:
        actions = [a for a in actions if a is not daemon]
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
