from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ares_usb_comm',
            executable='usb_bridge_node',
            name='usb_bridge_node',
            output='screen'
        )
    ]) 