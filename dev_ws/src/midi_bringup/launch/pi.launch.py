from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument(
            'lidar',
            default_value='False',  # Default value set to 'False' as a string
            description='Launch lidar node: true or false')
    )
    
    lidar = LaunchConfiguration('lidar')
    
    serial_node = Node(
        package="serial_comm",
        executable="serial_node",
        parameters=[{
            'baudrate': 115200,
            'port': '/dev/ttyACM0'
        }],
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ydlidar_ros2_driver'),
                'launch/ydlidar_launch_view.py'
            )
        ]),
        condition=IfCondition(PythonExpression(["'", lidar, "'.lower() == 'true'"]))
    )
    
    ld.add_action(serial_node)
    ld.add_action(lidar_launch)
    return ld

