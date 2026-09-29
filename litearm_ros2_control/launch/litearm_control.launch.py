#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_control.launch.py — bring up the whole ros2_control stack.

Process topology:

    litearm_hw_daemon        ← owns USB CDC, holds the link to litearm-stm32
      ↕ POSIX shm (seqlock, double-buffered)   firmware; creates the shm segment
    ros2_control_node        ← controller_manager + LitearmSystem plugin
      ├─ joint_state_broadcaster
      └─ joint_trajectory_controller

The daemon is brought up before ros2_control_node; the plugin waits for the
daemon to become ready inside on_configure and, on timeout or failure, reports
the daemon's own specific reason (port not found / license not activated /
motor feedback not ready).

The command channel defaults to the firmware's **MOVE_JS position mode**: both
the PD loop and the gravity/friction/integral/kd_extra feedforward are computed
by the firmware (the model is generated from the URDF and compiled into the
firmware). Use ``mit_passthrough:=true`` to switch to full MIT_ALL passthrough
if you want to compute the feedforward yourself.

CPU affinity (PREEMPT_RT + GRUB isolcpus=2,3): both real-time processes are
pinned to isolated cores by default — the daemon (250 Hz) → CPU 2 (same core as
the USB interrupt, together with rt_env.sh irq-pin), and ros2_control_node
(250 Hz, 1:1 with the daemon; JSB/JTC are threads inside its process) → CPU 3
(control loop runs alone, undisturbed by USB interrupts); override with
daemon_cpu:= / control_cpu:=, an empty string = no pinning.

Two ways to run it:

  Real HW  ros2 launch litearm_ros2_control litearm_control.launch.py
           (board powered, USB connected, **license activated** — ENABLE is
           rejected while it is not activated)

  Dry run  ros2 launch litearm_ros2_control litearm_control.launch.py
           dry_run:=true — the daemon starts a fake firmware on a pty, **the
           link still speaks the real protocol**; used to exercise the URDF /
           controllers / topics / protocol end to end; first-order kinematics
           model, so it must not be used to tune gains.
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

from litearm_ros2_control.ros_env import isolation_hint, parse_bool, ros_isolation_env


def _is_true(value: str) -> bool:
    return parse_bool(value, False)


def _declare_arguments():
    return [
        DeclareLaunchArgument(
            "port", default_value="",
            description="USB CDC device path of the litearm-stm32; "
                        "empty = auto-discover by VID:PID 1d50:606f"),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true = no-hardware mode (the daemon starts a fake "
                        "firmware on a pty, the link still speaks the real "
                        "protocol; first-order kinematics model, no gain "
                        "tuning)"),
        DeclareLaunchArgument(
            "start_daemon", default_value="true",
            description="false = reuse an already running daemon (common "
                        "during integration debugging)"),
        DeclareLaunchArgument(
            "hw_config", default_value="",
            description="optional path to litearm_hw.yaml: PC-side parameters "
                        "only (port/rate/timeouts/policy). Empty = use only "
                        "this file's parameters. Note that **the command line "
                        "takes precedence over that file**"),
        DeclareLaunchArgument(
            "shm_name", default_value="/litearm_hw",
            description="shared memory segment name; must match the plugin's "
                        "URDF parameter"),
        DeclareLaunchArgument(
            "daemon_rate_hz", default_value="0",
            description="daemon command publish rate; 0 = use the daemon "
                        "default (250Hz)"),
        DeclareLaunchArgument(
            "daemon_cpu", default_value="2",
            description="CPU core the daemon is pinned to (taskset -c); "
                        "empty = no pinning. Default 2 = an isolated core, "
                        "same core as the USB interrupt (rt_env.sh irq-pin)"),
        DeclareLaunchArgument(
            "control_cpu", default_value="3",
            description="CPU core ros2_control_node is pinned to (taskset -c); "
                        "empty = no pinning. Default 3 = an isolated core, "
                        "dedicated to the control loop, undisturbed by USB "
                        "interrupts"),
        # The five feedforward switches are now **tri-state**: empty (the
        # default) = leave the firmware alone (it has its own factory mask),
        # true/false = explicitly set/clear the bit. For why we do not
        # override by default see the "feedforward override" section of
        # hw_daemon.py: the single source of truth for joint-level parameters
        # is the firmware, and the daemon should not silently change it.
        DeclareLaunchArgument(
            "gravity_compensation", default_value="",
            description="override the FF_G bit of the firmware ff_mask: "
                        "empty = leave the firmware alone, true/false = "
                        "explicitly set/clear"),
        DeclareLaunchArgument(
            "friction_compensation", default_value="",
            description="override the FF_FRICTION bit of the firmware ff_mask "
                        "(friction v1/v2/drag)"),
        DeclareLaunchArgument(
            "inertia_compensation", default_value="",
            description="override the FF_INERTIA|FF_CORIOLIS bits of the "
                        "firmware ff_mask (⚠ on the default MOVE_JS channel "
                        "the firmware does not compute inertia terms, so "
                        "setting it has no effect)"),
        DeclareLaunchArgument(
            "integral_compensation", default_value="",
            description="override the FF_INTEGRAL bit of the firmware ff_mask "
                        "(ki·∫e dt)"),
        DeclareLaunchArgument(
            "damping_compensation", default_value="",
            description="override the firmware kd_extra vector: false = zero "
                        "it, true = restore the factory 6/6/6/6/0/0/0"),
        DeclareLaunchArgument(
            "mit_passthrough", default_value="false",
            description="true = switch to the MIT_ALL passthrough channel: "
                        "kp/kd/effort take effect per frame and the firmware "
                        "adds none of its own feedforward (default is MOVE_JS "
                        "position mode)"),
        DeclareLaunchArgument(
            "exit_hold_s", default_value="2.0",
            description="seconds to keep publishing the hold reference before "
                        "exiting (0 = park and exit straight away)"),
        DeclareLaunchArgument(
            "disable_on_shutdown", default_value="false",
            description="true = request motor disable on exit (the arm goes "
                        "limp and drops; support it first)"),
        DeclareLaunchArgument(
            "use_rviz", default_value="false",
            description="also start RViz2"),
        DeclareLaunchArgument(
            "ros_domain_id", default_value="42",
            description="ROS domain used by this stack (0 is deliberately "
                        "avoided). Set it to 0 when interfacing with external "
                        "systems"),
        DeclareLaunchArgument(
            "ros_localhost_only", default_value="true",
            description="true = localhost discovery only. Cross-machine "
                        "crosstalk corrupts robot_description / joint_states "
                        "/ move_action, so isolation is on by default"),
        # ── robot description (optional override) ───────────────────────────
        # Defaults to the arm-only description shipped with this package. To
        # use the gripper, pass a description that carries both the arm and
        # the gripper <ros2_control> blocks (litearm_manipulation's
        # litearm_gripper.urdf.xacro).
        DeclareLaunchArgument(
            "urdf_xacro", default_value="",
            description="optional xacro path that overrides the default "
                        "arm-only description"),
        # ── LiteGrip gripper (off by default, arm-only behaviour stays word
        #    for word the same) ────────────────────────────────────────────
        DeclareLaunchArgument(
            "start_gripper", default_value="false",
            description="true = start the gripper hardware daemon + load its "
                        "hardware component + start gripper_controller"),
        DeclareLaunchArgument(
            "gripper_shm_name", default_value="/litegrip_hw",
            description="gripper shared memory segment name; must match "
                        "shm_name in the URDF plugin parameters"),
        DeclareLaunchArgument(
            "gripper_channel", default_value="can0",
            description="gripper CAN interface (the STM32's gs_usb bridge; "
                        "1 Mbit/s required)"),
        DeclareLaunchArgument(
            "gripper_dry_run", default_value="",
            description="empty = follow the arm's dry_run; may also be set "
                        "explicitly to true/false on its own"),
        DeclareLaunchArgument(
            "gripper_hardware_enable", default_value="false",
            description="master switch for real gripper hardware; motors are "
                        "only driven when this and dry_run=false both hold"),
        DeclareLaunchArgument(
            "gripper_sdk_path", default_value="",
            description="path to the LiteGrip SDK copy; empty = use the copy "
                        "in the workspace <ws>/src/litegrip/sdk"),
        DeclareLaunchArgument(
            "gripper_feedback_velocity_rad_s", default_value="-1.0",
            description="worst-case feedback velocity bound (rad/s); "
                        "-1 = not given ⇒ the real hardware refuses to send "
                        "any motion frame"),
        DeclareLaunchArgument(
            "gripper_exit_hold_s", default_value="2.0",
            description="seconds the gripper daemon keeps holding position "
                        "before it exits"),
    ]


def _tri_state_flag(name: str, raw: str) -> list:
    """Turn a tri-state launch argument into the daemon switch.

    ``""`` = pass nothing (leave the firmware alone); ``"true"`` →
    ``--name-compensation``; ``"false"`` → ``--no-name-compensation``.
    Without the ``--no-`` half there is no way to A/B against "revert to
    plain PD".
    """
    text = raw.strip().lower()
    if not text:
        return []
    prefix = "--" if parse_bool(text, False) else "--no-"
    return [f"{prefix}{name}-compensation"]


def _launch_setup(context, *_args, **_kwargs):
    """Resolve the arguments in the launch context and assemble the actions."""
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

    # ── expand the xacro to get the full URDF with <ros2_control> ──
    # shm_name and disable_on_shutdown must match the daemon side, so they are
    # injected through the same launch arguments to avoid silent mismatches
    # such as "the launch argument changed but the URDF is still the old one".
    #
    # ★ urdf_xacro defaults to the **arm-only** description shipped with this
    #   package. To use the gripper, the caller passes a description carrying
    #   both the arm and the gripper <ros2_control> blocks
    #   (litearm_manipulation's litearm_gripper.urdf.xacro) — that way this
    #   package **does not have to depend on** the manipulation package's
    #   layout, and "who assembles the description" stays with the caller.
    urdf_xacro = resolve("urdf_xacro").strip() or os.path.join(
        pkg_share, "urdf", "litearm.urdf.xacro")
    robot_description = xacro.process_file(
        urdf_xacro,
        mappings={
            "litearm_shm_name": shm_name,
            "litearm_disable_on_shutdown": "true" if disable_on_shutdown else "false",
        },
    ).toxml()

    # ── hardware daemon ──
    # Joint-level parameters (kp/kd/tau_max/limits/feedforward) are not passed
    # here: their single source of truth is the firmware and the daemon reads
    # them back with 0x24 / 0x2B / 0x2C at startup. Only PC-side concepts are
    # passed here, plus policy switches such as "should the firmware
    # feedforward be explicitly overridden".
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
        # Pin to an isolated core (taskset -c): the 250 Hz cycle is not
        # preempted by other tasks and not migrated.
        prefix=f"taskset -c {daemon_cpu}" if daemon_cpu else None,
        condition=IfCondition(LaunchConfiguration("start_daemon")))

    # ── LiteGrip gripper hardware daemon (optional, start_gripper:=true) ──
    # It is a **separate process** from the arm daemon: the arm owns
    # /dev/ttyACM0 (the CDC command port) and the gripper owns can0 (the if0
    # gs_usb bridge on the same STM32 board) — they do not contend.
    # It must come up before ros2_control_node: it is the **owner** of the
    # shared memory segment and the plugin's on_configure waits for it (on
    # timeout you get actionable troubleshooting hints).
    gripper_daemon = None
    gripper_controllers_file = None
    if start_gripper:
        gripper_share = get_package_share_directory("litegrip_ros2_control")
        gripper_controllers_file = os.path.join(
            gripper_share, "config", "litegrip_controllers.yaml")
        # The SDK copy lives in <ws>/src/litegrip/sdk and <ws> = prefix/../..
        # (prefix being <ws>/install/<pkg>) — an existing convention in this
        # project, see manipulation.yaml.
        # ⚠ Do not walk up from the share/ directory: it has a different
        # number of levels than prefix, so it is easy to go up one too many.
        gripper_sdk = resolve("gripper_sdk_path").strip() or os.path.normpath(
            os.path.join(get_package_prefix("litegrip_ros2_control"),
                         os.pardir, os.pardir, "src", "litegrip", "sdk"))
        # dry_run **follows the arm** by default (both sides are fake in the
        # same dry run), but it can be overridden on its own: while the arm
        # runs against the fake firmware, the gripper may still be talking to
        # the real SDK / real can0.
        gripper_dry_raw = resolve("gripper_dry_run").strip()
        gripper_dry = dry_run if not gripper_dry_raw else _is_true(gripper_dry_raw)
        gripper_cmd = [
            os.path.join(get_package_prefix("litegrip_ros2_control"), "lib",
                         "litegrip_ros2_control", "litegrip_hw_daemon"),
            "--shm-name", resolve("gripper_shm_name"),
            "--channel", resolve("gripper_channel"),
            "--exit-hold-s", resolve("gripper_exit_hold_s").strip() or "2.0",
            "--max-feedback-velocity-rad-s",
            resolve("gripper_feedback_velocity_rad_s"),
            "--sdk-path", gripper_sdk,
            "--dry-run" if gripper_dry else "--no-dry-run",
            ("--hardware-enable" if _is_true(resolve("gripper_hardware_enable"))
             else "--no-hardware-enable"),
        ]
        gripper_daemon = ExecuteProcess(cmd=gripper_cmd, output="screen",
                                        additional_env=env)

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
        # Pin to an isolated core: both the 250 Hz control loop and JSB/JTC
        # (threads in the same process) stay on this core.
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
    # Gripper controller (only with start_gripper:=true). It claims
    # gripper_opening_joint, which shares **no name** with any arm joint, so
    # there is no interface contention and it can be active at the same time
    # as the arm controllers.
    # It is deliberately named gripper_controller: it lines up with the
    # controller of the same name in moveit_controllers.yaml on the MoveIt
    # side (JTC provides an action of the same name), so migrating the upper
    # layer costs zero changes on the MoveIt side.
    spawn_gripper = (spawner("gripper_controller") if start_gripper else None)
    # The servo controller is **loaded but not activated** (--inactive): it
    # and JTC both claim position+velocity on joint1..7 and only one of them
    # can be active at a time (ros2_control gives exclusive ownership of an
    # interface), so it just sits there and you hand over explicitly with
    # servo_switch.py when you really want it.
    # While inactive it claims no interfaces and has zero effect on the
    # existing chain.
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
        # The RobotModel display goes through the /robot_description topic
        # (the config sets Transient Local to match RSP's latched publisher);
        # the parameter here is a fallback for plugins that need it.
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    banner_target = ("dry-run (fake firmware on a pty, the link speaks the "
                     "real protocol)" if dry_run
                     else f"real hardware, port={port or '(auto-discover 1d50:606f)'}")
    banner_channel = ("MIT_ALL passthrough" if mit_passthrough
                      else "MOVE_JS position mode (firmware runs PD + its own "
                           "feedforward)")
    actions = [
        LogInfo(msg=(
            "──────── litearm control stack ────────\n"
            f"  {banner_target}\n"
            f"  command channel: {banner_channel}\n"
            f"  CPU affinity: daemon → {daemon_cpu or 'unpinned'}, "
            f"ros2_control_node → {control_cpu or 'unpinned'}\n"
            "  ⚠ this launch has locked the ROS domain; to make the ros2\n"
            "    command line see this stack, run this in your own terminal:\n"
            f"        {isolation_hint(domain_id, localhost_only)}\n"
            "────────────────────────────────")),
        daemon,
        # The gripper daemon follows the arm daemon: it **must come before
        # ros2_control_node** — it owns the gripper shared memory segment and
        # the plugin's on_configure waits for it.
        *([gripper_daemon] if gripper_daemon is not None else []),
        robot_state_publisher,
        rviz,
        # The daemon has to open the port, read the firmware parameters and
        # wait for enable (the magnets are only energised once every feedback
        # is in, up to ~5s); let it get going first so the plugin's
        # on_configure does not immediately trip over "no heartbeat yet".
        TimerAction(period=1.5, actions=[ros2_control_node]),
        # Wait until ros2_control_node is really up before spawning, so the
        # spawner does not sit there until it times out.
        RegisterEventHandler(OnProcessStart(target_action=ros2_control_node,
                                            on_start=[spawn_jsb])),
        # Start JTC after JSB is up: JTC depends on the state_interfaces
        # already being broadcast.
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb,
                                           on_exit=[spawn_jtc])),
        # The servo controller is loaded after JTC (not activated): a spawner
        # internally does load+configure(+activate), and several spawners in
        # parallel tend to collide on the same parameter/plugin load, so
        # chaining them is the least trouble. --inactive claims no interfaces,
        # so it does not affect the ones JTC already holds.
        RegisterEventHandler(OnProcessExit(target_action=spawn_jtc,
                                           on_exit=[spawn_servo_inactive])),
        # The gripper controller comes last (also serial). It claims
        # gripper_opening_joint — no shared name with any arm joint and no
        # interface contention, so it can be active at the same time as the
        # arm controllers.
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
