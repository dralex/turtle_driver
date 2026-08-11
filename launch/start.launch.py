#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtle_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node'
    )
    driver_node = Node(
        package='turtle_driver',
        executable='driver',
        name='driver'
    )
    return LaunchDescription([
        turtle_node, driver_node
    ])
