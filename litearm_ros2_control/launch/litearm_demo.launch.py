#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""litearm_demo.launch.py — a complete demo with no real hardware.

One command brings up the whole stack and shows the arm swinging continuously
in RViz:

    ros2 launch litearm_ros2_control litearm_demo.launch.py

What it is made of:

    litearm_hw_daemon --dry-run    owns the "virtual CAN" and produces joint
      ↕ POSIX shared memory (seqlock)   feedback with a first-order model
    ros2_control_node              controller_manager + LitearmSystem plugin
      ├─ joint_state_broadcaster
      ├─ joint_trajectory_controller
      └─ demo_motion.py            keeps publishing a swing trajectory so the
                                   chain is "visible"
    robot_state_publisher + RViz2

⚠️ This launch **deliberately has no dry_run argument**; dry_run is hard-wired
to true. For real hardware use litearm_control.launch.py /
litearm_moveit.launch.py — separating "demo" from "real hardware" into two
entry points means nobody powers up the arm by misreading an argument.

⚠️ You must set the same ROS domain in your own terminal as well
----------------------------------------------------------------
This launch locks **every process it starts** to ``ros_domain_id`` (42 by
default) + localhost only (``ros_localhost_only`` defaults to true). The reason
is that ROS 2 defaults to domain 0 and multicast discovery spans the whole
subnet: as long as another machine on that subnet is running ROS 2 (this
project actually ran into a twoarm/rail device in testing), you get symptoms
like —

* RViz shows **somebody else's robot**, and their mesh paths do not exist on
  this machine, so on screen "the robot has vanished" (the log says
  ``Package [litearm_description] does not exist``);
* ``/robot_description`` / ``/joint_states`` get overwritten by someone else's
  data;
* ``/move_action`` shows **multiple action servers**, and planning goals are
  taken over by the move_group of some unfamiliar node.

So this launch isolates itself. But **your terminal will not follow along
automatically**; to look at the topics with the ros2 command line you have to
set it yourself:

    export ROS_DOMAIN_ID=42
    export ROS_LOCALHOST_ONLY=1

To interface with an external system, turn the isolation off explicitly:
    ros2 launch litearm_ros2_control litearm_demo.launch.py \\
        ros_domain_id:=0 ros_localhost_only:=false

What the demo does not do
-------------------------
The dry-run joint response is a first-order lag, ``q ← q + (q_ref − q)·min(1, dt/τ)``,
**with no inertia, gravity, friction or torque limits**. So it verifies the
interfaces / data path / controller configuration, but it **must not** be used
to tune gains, judge tracking performance or validate safety logic (for those,
look at the daemon's unit tests and the actual CAN frames).

Usage:

    # default: infinite swinging + RViz
    ros2 launch litearm_ros2_control litearm_demo.launch.py

    # headless environment (SSH / container): no RViz, just logs and probes
    ros2 launch litearm_ros2_control litearm_demo.launch.py use_rviz:=false

    # bring up the stack without swinging, publish trajectories yourself
    ros2 launch litearm_ros2_control litearm_demo.launch.py demo_motion:=false

    # small amplitude, a bit faster, exits after 3 cycles (for CI)
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
                              description="start RViz2 to view the arm"),
        DeclareLaunchArgument("demo_motion", default_value="true",
                              description="automatically publish the swing "
                                          "trajectory (false = stack only)"),
        DeclareLaunchArgument("motion_scale", default_value="1.0",
                              description="swing amplitude scale factor"),
        DeclareLaunchArgument("motion_period", default_value="12.0",
                              description="seconds per swing cycle (a smaller "
                                          "value makes JTC abort on tracking "
                                          "error)"),
        DeclareLaunchArgument("cycles", default_value="0",
                              description="number of swing cycles, 0 = forever "
                                          "until Ctrl-C"),
        DeclareLaunchArgument("shm_name", default_value="/litearm_hw",
                              description="shared memory segment name (no need "
                                          "to change it for the demo)"),
        DeclareLaunchArgument("ros_domain_id", default_value="42",
                              description="ROS domain used by this demo. Set it "
                                          "to 0 when interfacing with external "
                                          "systems"),
        DeclareLaunchArgument("ros_localhost_only", default_value="true",
                              description="true = localhost discovery only, "
                                          "avoids cross-machine crosstalk"),
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

    # Expand it once so RSP / controller_manager / RViz share the same
    # description, guaranteeing that all three see exactly the same
    # <ros2_control> parameters.
    robot_description = xacro.process_file(
        os.path.join(pkg_share, "urdf", "litearm.urdf.xacro"),
        mappings={"litearm_shm_name": shm_name}).toxml()

    # ── hardware daemon: --dry-run is hard-wired, the demo entry point never
    #    touches CAN ──
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

    # ── swing demo ──
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
        # robot_description is also passed to RViz as a parameter: the
        # RobotModel display goes through the /robot_description topic (the
        # config already sets Transient Local to match RSP's latched
        # publisher), and the parameter here is a fallback for plugins that
        # need it (such as MoveIt).
        parameters=[{"robot_description": robot_description}],
        condition=IfCondition(LaunchConfiguration("use_rviz")))

    actions = [
        LogInfo(msg=(
            "──────── litearm no-hardware demo ────────\n"
            "  mode: dry-run (the daemon sends no CAN frames; the joint\n"
            "  response is a first-order kinematics model)\n"
            "  ⚠ this launch has locked the ROS domain; to make the ros2\n"
            "    command line see this demo's topics, run this in your own\n"
            "    terminal:\n"
            f"        {isolation_hint(domain_id, localhost_only)}\n"
            "  this validates interfaces and the data path only, not\n"
            "  dynamics/gains/safety logic\n"
            "  for real hardware use: ros2 launch litearm_ros2_control "
            "litearm_control.launch.py\n"
            "────────────────────────────────────")),
        daemon,
        robot_state_publisher,
        rviz,
        # The daemon connects (fast under dry-run, but keep the same timing as
        # the control stack launch)
        TimerAction(period=1.5, actions=[ros2_control_node]),
        RegisterEventHandler(OnProcessStart(target_action=ros2_control_node,
                                            on_start=[spawn_jsb])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_jsb,
                                           on_exit=[spawn_jtc])),
    ]
    if run_motion:
        # The spawner exiting means JTC is activated, and only then does the
        # action server exist
        actions.append(RegisterEventHandler(
            OnProcessExit(target_action=spawn_jtc, on_exit=[demo_motion])))
    return actions


def generate_launch_description():
    return LaunchDescription(_declare_arguments() +
                             [OpaqueFunction(function=_launch_setup)])
