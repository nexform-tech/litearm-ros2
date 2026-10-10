"""Start litearm_driver and bring it to the active state.

The node runs in the `litearm` namespace, so its relative service names resolve to
`/litearm/enable`, `/litearm/clear_faults` and so on. Only one process can own the arm's
serial port, so do not run this together with the ros2_control stack.

Use it when you want to diagnose, tune or license an arm without a controller_manager:
run `ros2 launch litearm_driver litearm_driver.launch.py`, then call services such as
`ros2 service call /litearm/get_status litearm_msgs/srv/GetStatus "{timeout: 0.5}"`.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    autostart = LaunchConfiguration("autostart")
    config = LaunchConfiguration("config")

    node = LifecycleNode(
        package="litearm_driver",
        executable="litearm_driver",
        name="driver",
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=[config],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        ),
        condition=IfCondition(autostart),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="litearm",
            description="Namespace of the driver node; the services live directly in it.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Configure and activate the node after it starts. Set false to "
                        "drive the lifecycle by hand.",
        ),
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(
                _share_directory(), "config", "litearm_driver.yaml"),
            description="Parameter file for the driver.",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Log level of the driver node.",
        ),
        node,
        RegisterEventHandler(
            OnProcessStart(target_action=node, on_start=[configure]),
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                goal_state="inactive",
                entities=[activate],
            ),
        ),
    ])


def _share_directory():
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory("litearm_driver")
