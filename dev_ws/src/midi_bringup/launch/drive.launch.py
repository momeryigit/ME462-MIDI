from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    config_filepath = LaunchConfiguration('config_filepath')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('joy_config', default_value='xbox'))
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
    teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('teleop_twist_joy'),
                'launch/teleop-launch.py')),
            launch_arguments={'joy_config': joy_config,
                              'config_filepath': config_filepath
                              }.items()
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
    ld.add_action(teleop_joy)
    ld.add_action(rviz_node)
    ld.add_action(urdf_publisher)
    return ld


    urdfv4export_description_dir = get_package_share_directory('URDFv4export_description')

    rvizconfig_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=urdfv4export_description_dir + '/launch/urdf.rviz',
        description='Path to the rviz configuration file'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        rvizconfig_arg,
        rviz_node
    ])