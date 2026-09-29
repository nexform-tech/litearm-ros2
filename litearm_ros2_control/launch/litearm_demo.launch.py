#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_demo.launch.py — 不接真实硬件的完整演示。

一条命令起全套并在 RViz 里看到机械臂持续摆动：

    ros2 launch litearm_ros2_control litearm_demo.launch.py

组成：

    litearm_hw_daemon --dry-run    独占"虚拟 CAN"，用一阶运动学模型产生关节反馈
      ↕ POSIX 共享内存（seqlock）
    ros2_control_node              controller_manager + LitearmSystem 插件
      ├─ joint_state_broadcaster
      ├─ joint_trajectory_controller
      └─ demo_motion.py            持续下发摆动轨迹，让链路"看得见"
    robot_state_publisher + RViz2

⚠️ 这个 launch **故意不提供 dry_run 参数**，dry_run 被硬钉为 true。
真实硬件请用 litearm_control.launch.py / litearm_moveit.launch.py ——
把"演示"和"真机"分成两个入口，就不存在看错参数把臂开起来这种事。

⚠️ 一定要在自己的终端里也设同样的 ROS 域
----------------------------------------
本 launch 会把**它启动的所有进程**锁在 ``ros_domain_id``（默认 42）+ 只走本机
（``ros_localhost_only`` 默认 true）。原因是 ROS 2 默认域 0 且组播发现是全网段的：
同网段只要还有另一台机器在跑 ROS 2（本工程实测遇到过一台 twoarm/rail 设备），
就会出现这些症状 ——

* RViz 显示的是**别人的机器人**，而对方网格路径在本机不存在，
  于是界面上"机器人不见了"（日志里是 ``Package [litearm_description] does not exist``）；
* ``/robot_description`` / ``/joint_states`` 被别人的数据覆盖；
* ``/move_action`` 出现**多个 action server**，规划目标被陌生节点的 move_group 接管。

所以本 launch 自带隔离。但**你的终端不会自动跟着变**，想用 ros2 命令行看话题就得自己设：

    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=1

要与外部系统对接时显式关掉隔离：
    ros2 launch litearm_ros2_control litearm_demo.launch.py \\
        ros_domain_id:=0 ros_localhost_only:=false

演示不做什么
------------
dry-run 的关节响应是 ``q ← q + (q_ref − q)·min(1, dt/τ)`` 这种一阶滞后，
**不含惯量、重力、摩擦、力矩限幅**。所以它能验证接口/数据通路/控制器配置，
**不能**用来整定增益、评估跟踪性能或验证安全逻辑（那些要看守护进程的单测
与实际 CAN 报文）。

用法：

    # 默认：无限摆动 + RViz
    ros2 launch litearm_ros2_control litearm_demo.launch.py

    # 无显示环境（SSH / 容器）：关掉 RViz，只看日志与探针
    ros2 launch litearm_ros2_control litearm_demo.launch.py use_rviz:=false

    # 只起栈不摆动，自己用命令行发轨迹
    ros2 launch litearm_ros2_control litearm_demo.launch.py demo_motion:=false

    # 小幅度、快一点、跑 3 轮就退出（CI 用）
    ros2 launch litearm_ros2_control litearm_demo.launch.py \\
        motion_scale:=0.5 motion_period:=8.0 cycles:=3 use_rviz:=false
"""

import os

import xacro
from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from litearm_ros2_control.ros_env import isolation_hint, parse_bool, ros_isolation_env


def _is_true(value: str) -> bool:
    return parse_bool(value, False)


def _declare_arguments():
    return [
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="是否启动 RViz2 查看机械臂"),
        DeclareLaunchArgument("demo_motion", default_value="true",
                              description="是否自动下发摆动轨迹（false = 只起栈）"),
        DeclareLaunchArgument("motion_scale", default_value="1.0",
                              description="摆动幅度缩放"),
        DeclareLaunchArgument("motion_period", default_value="12.0",
                              description="单个摆动周期秒数（调小会让 JTC 跟踪超差而中止）"),
        DeclareLaunchArgument("cycles", default_value="0",
                              description="摆动轮数，0 = 无限直到 Ctrl-C"),
        DeclareLaunchArgument("shm_name", default_value="/litearm_hw",
                              description="共享内存段名（演示不必改）"),
        DeclareLaunchArgument("ros_domain_id", default_value="42",
                              description="本演示使用的 ROS 域。要与外部系统对接时设 0"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = 只在本机发现，避免跨机串扰"),
    ]


def _launch_setup(context, *_args, **_kwargs):
    pkg_share = get_package_share_directory("litearm_ros2_control")
    pkg_prefix = get_package_prefix("litearm_ros2_control")
    daemon_exe = os.path.join(pkg_prefix, "lib", "litearm_ros2_control",
                              "litearm_hw_daemon")
    demo_script = os.path.join(pkg_prefix, "lib", "litearm_ros2_control",
                               "demo_motion.py")

    resolve = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    shm_name = resolve("shm_name")
    run_motion = _is_true(resolve("demo_motion"))
    domain_id = resolve("ros_domain_id").strip() or "42"
    localhost_only = resolve("ros_localhost_only")
    env = ros_isolation_env(domain_id, localhost_only)

    # 展开一次，RSP / controller_manager / RViz 共用同一份描述，
    # 保证三边看到的 <ros2_control> 参数完全一致。
    robot_description = xacro.process_file(
        os.path.join(pkg_share, "urdf", "litearm.urdf.xacro"),
        mappings={"litearm_shm_name": shm_name}).toxml()

    # ── 硬件守护进程：硬钉 --dry-run，演示入口永远不碰 CAN ──
    daemon = ExecuteProcess(
        cmd=[daemon_exe, "--dry-run", "--shm-name", shm_name,
             "--log-level", "INFO"],
        additional_env=env,
        output="screen")

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
        parameters=[
            {"robot_description": robot_description},
            os.path.join(pkg_share, "config", "litearm_controllers.yaml"),
        ],
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager",
                       "--controller-manager-timeout", "60"],
            output="screen",
            additional_env=env)

    spawn_jsb = spawner("joint_state_broadcaster")
    spawn_jtc = spawner("joint_trajectory_controller")

    # ── 摆动演示 ──
    demo_motion = ExecuteProcess(
        cmd=["python3", demo_script,
             "--scale", resolve("motion_scale"),
             "--period", resolve("motion_period"),
             "--cycles", resolve("cycles")],
        additional_env=env,
        output="screen")

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "rviz", "litearm_control.rviz")],
        output="log",
        additional_env=env,
        # robot_description 也作为参数传给 RViz：RobotModel 显示走的是
        # /robot_description 话题（配置里已设 Transient Local 以匹配 RSP 的
        # 锁存发布），这里的参数是对需要它的插件（如 MoveIt）的兜底。
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(LaunchConfiguration("use_rviz")))

    actions = [
        LogInfo(msg=(
            "──────── litearm 无硬件演示 ────────\n"
            "  模式：dry-run（守护进程不发 CAN 帧，关节响应为一阶运动学模型）\n"
            "  ⚠ 本 launch 已锁 ROS 域；想让 ros2 命令行看到本演示的话题，\n"
            "    请在你自己的终端里执行：\n"
            f"        {isolation_hint(domain_id, localhost_only)}\n"
            "  这不验证动力学/增益/安全逻辑，只看接口与数据通路\n"
            "  真机请用：ros2 launch litearm_ros2_control litearm_control.launch.py\n"
            "────────────────────────────────────")),
        daemon,
        robot_state_publisher,
        rviz,
        # 守护进程连接（dry-run 下很快，但保持与控制栈 launch 一致的时序）
        TimerAction(period=1.5, actions=[ros2_control_node]),
        RegisterEventHandler(OnProcessStart(target_action=ros2_control_node,
                                            on_start=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb,
                                           on_exit=[spawn_jtc])),
    ]
    if run_motion:
        # spawner 退出即代表 JTC 已 activate，此时 action server 才存在
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=spawn_jtc, on_exit=[demo_motion])))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
