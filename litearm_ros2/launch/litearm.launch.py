"""Launch the LiteArm ROS2 bridge node.

Usage:
    ros2 launch litearm_ros2 litearm.launch.py endpoint:=tcp/192.168.31.237:7447
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "endpoint", default_value="tcp/127.0.0.1:7447",
            description="litearm-python Zenoh endpoint, e.g. tcp/192.168.31.237:7447"),
        DeclareLaunchArgument("arm_id", default_value="armA",
                              description="Arm id registered on the server"),
        DeclareLaunchArgument("loop_hz", default_value="50.0",
                              description="State publish rate in Hz"),
        DeclareLaunchArgument("cmd_speed", default_value="0.5",
                              description="Default joint move speed (0..1)"),
        DeclareLaunchArgument("joint_names", default_value="joint0 joint1 joint2 joint3 joint4 joint5 joint6",
                              description="Space-separated joint names"),
        DeclareLaunchArgument("base_frame", default_value="base_link",
                              description="Robot base frame name"),
        DeclareLaunchArgument("tcp_frame", default_value="tool0",
                              description="TCP (tool) frame name"),
        Node(
            package="litearm_ros2",
            executable="litearm_node",
            name="litearm_node",
            output="screen",
            parameters=[{
                "endpoint": LaunchConfiguration("endpoint"),
                "arm_id": LaunchConfiguration("arm_id"),
                "loop_hz": LaunchConfiguration("loop_hz"),
                "cmd_speed": LaunchConfiguration("cmd_speed"),
                "joint_names": LaunchConfiguration("joint_names"),
                "base_frame": LaunchConfiguration("base_frame"),
                "tcp_frame": LaunchConfiguration("tcp_frame"),
            }],
        ),
    ])
