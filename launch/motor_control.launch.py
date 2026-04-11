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
    
    # Magnetic encoder node from magnetic_encoder_pkg
    magnetic_encoder_node = Node(
        package='magnetic_encoder_pkg',
        executable='magnetic_encoder_node',
        name='magnetic_encoder_node',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare('magnetic_encoder_pkg'),
            'config',
            'magnetic_encoder_params.yaml'
        ])]
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
        magnetic_encoder_node,
        diff_drive_node,
    ])