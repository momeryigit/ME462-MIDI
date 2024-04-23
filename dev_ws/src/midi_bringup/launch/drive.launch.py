from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')
    
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('joy_vel', default_value='cmd_vel'))
    ld.add_action(DeclareLaunchArgument('joy_config', default_value='xbox'))
    ld.add_action(DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'))
    ld.add_action(DeclareLaunchArgument('config_filepath', default_value=os.path.join(
        get_package_share_directory('midi_bringup'), 'config', 'xbox.config.yaml'
    )))

    drive_node = Node(
        package="robot_drive",
        executable="drive_node",
    )
    state_publisher_node = Node(
        package="state_publisher",
        executable="state_publisher",
    )
    # include another launch file
    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('state_publisher'),
                'launch/demo_launch.py'))
    )

    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.3,
            'autorepeat_rate': 10.0,
            'sticky_buttons': True,
        }])
    teleop_joy_node = Node(
        package='teleop_twist_joy', executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[config_filepath],
        remappings={('/cmd_vel', LaunchConfiguration('joy_vel'))},
    )
    
    urdf_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('midibot_description'),
                'launch/publishers.launch.py'))
    )
    #Run rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('state_publisher'), 'config/rviz.rviz')],
        parameters=[{'use_sim_time': False}]
    )
    
    ld.add_action(drive_node)
    ld.add_action(state_publisher_node)
    ld.add_action(joy_node)
    ld.add_action(teleop_joy_node)
    ld.add_action(rviz_node)
    ld.add_action(urdf_publisher)
    return ld

