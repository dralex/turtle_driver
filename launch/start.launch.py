#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
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

    # The simulator draws its window with Qt, which needs a display and refuses to start
    # without one. On a machine with no display - a test runner, a container, a robot -
    # the offscreen platform of Qt runs the simulator with everything but the window, and
    # the turtle moves the same way. The default is the interactive one: asking for the
    # window is what a user of a workstation expects.
    headless = LaunchConfiguration('headless')
    declare_headless = DeclareLaunchArgument(
        'headless', default_value='false',
        description='run the simulator without a display')
    set_offscreen = SetEnvironmentVariable(
        'QT_QPA_PLATFORM', 'offscreen', condition=IfCondition(headless))

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

    return LaunchDescription([declare_params_file, declare_headless, set_offscreen,
                              turtle_node, driver_node])
