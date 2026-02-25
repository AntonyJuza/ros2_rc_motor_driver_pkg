from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('humanoid_motor_control').find('humanoid_motor_control')
    
    # Config file path
    config_file = PathJoinSubstitution([
        FindPackageShare('humanoid_motor_control'),
        'config',
        'motor_control.yaml'
    ])
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Cytron motor controller node
    cytron_node = Node(
        package='humanoid_motor_control',
        executable='cytron_controller_node',
        name='cytron_controller',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    # Encoder reader node
    encoder_node = Node(
        package='humanoid_motor_control',
        executable='encoder_node',
        name='encoder_reader',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    # Differential drive controller node
    diff_drive_node = Node(
        package='humanoid_motor_control',
        executable='diff_drive_controller',
        name='diff_drive_controller',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        cytron_node,
        encoder_node,
        diff_drive_node,
    ])