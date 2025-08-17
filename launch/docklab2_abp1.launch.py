#!/usr/bin/env python

# Code for launching Air Bearing Platform 1 (ABP1)
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='docklab2',
            namespace='abp1',
            executable='relay_control6',
            name='relay_server'
        ),
        Node(
            package='docklab2',
            namespace='abp1',
            executable='GRASP_control',
            name='GRASP_node',
            parameters=[PathJoinSubstitution([
                FindPackageShare('docklab2'), 'config', 'params.yaml'])
            ],
        )
    ])
