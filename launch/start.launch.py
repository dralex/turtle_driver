#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The driver is not renamed here: it is called the way it names itself, so it has the same
# name whether it is started by this file or by ros2 run, and one block of the parameter
# file applies to it in both cases.


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('turtle_driver'),
                                  'config', 'default_params.yaml')
    params_file = LaunchConfiguration('params_file')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='the parameter file of the turtle driver')

    turtle_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node'
    )
    driver_node = Node(
        package='turtle_driver',
        executable='driver',
        parameters=[params_file]
    )

    return LaunchDescription([declare_params_file, turtle_node, driver_node])
