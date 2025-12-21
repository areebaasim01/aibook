#!/usr/bin/env python3
"""
Robot Launch File
Module 1: The Robotic Nervous System

This launch file starts the robot visualization in RViz
along with the robot state publisher.

Usage:
    ros2 launch my_robot_pkg robot_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate the launch description"""
    
    # Declare the URDF file argument
    urdf_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='bipedal_robot.urdf',
        description='Name of the URDF file'
    )
    
    # Get the URDF file path
    # In a real package, this would use get_package_share_directory
    urdf_file = LaunchConfiguration('urdf_file')
    
    # For standalone testing, use current directory
    # urdf_path = os.path.join(os.getcwd(), urdf_file)
    
    # Robot description from URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher - publishes TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint State Publisher GUI - allows manual joint control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', 'default.rviz']  # Optional: custom RViz config
    )
    
    return LaunchDescription([
        urdf_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])
