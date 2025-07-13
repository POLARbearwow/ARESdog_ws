"""robot_bringup.launch.py

Launches the essential nodes required for the quadruped: the USB bridge (handles
low-level motor/IMU communication) and the model-based high-level controller
running ONNX inference.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description that starts all required nodes."""

    # ------------------------------------------------------------------
    #  Group: Low-level USB communication
    # ------------------------------------------------------------------

    usb_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ares_usb"),
                "launch",
                "comm_bringup.launch.py",  # corrected filename
            ])
        )
    )

    # ------------------------------------------------------------------
    #  WebSocket bridge for remote_control (rosbridge_server)
    # ------------------------------------------------------------------
    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": 9090}],
    )

    # High-level ONNX controller node from *robot_controller*
    controller_node = Node(
        package="rl_controller",
        executable="rl_controller_node",
        name="rl_controller",
        output="screen",
    )

    return LaunchDescription([
        usb_bridge_launch,
        controller_node,
        rosbridge_node,
    ]) 