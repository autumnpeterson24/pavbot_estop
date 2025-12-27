"""
Launch file for ros node!
You don't technically need this but it is an easy way to get the node up and running super quickly and be able to set the parameters the way you want them!

Run instructions:
    ros2 launch pav_estop estop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pav_estop',
            executable='estop_node',
            name='estop_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'baud': 115200,
                'timeout_ms': 500
            }]
        )
    ])
