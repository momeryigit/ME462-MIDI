from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_comm',
            namespace='serial',
            executable='serial_node',
            name='serial_node'
        )
    ])