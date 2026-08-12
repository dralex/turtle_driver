# -----------------------------------------------------------------------------
# The ROS2 Turtle Driver
# -----------------------------------------------------------------------------
#
# The fixtures of the driver node tests
#
# Copyright (C) 2026 Alexey Fedoseev <aleksey@fedoseev.net>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public
# License as published by the Free Software Foundation; either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see https://www.gnu.org/licenses/
#
# -----------------------------------------------------------------------------

# The fixtures come from the test utilities of the framework, so the node tests of this
# package and the integration tests share one vocabulary. They are imported here to be
# visible to pytest.

from hsm_test_utils import (isolated_domain, node_factory,  # noqa: F401
                            storage_directory)

from geometry_msgs.msg import PoseStamped
from turtlesim.msg import Pose

GOAL_TOPIC = '/goal_pose'
TWIST_TOPIC = '/cmd_vel'
ODOM_TOPIC = '/odom'
MESSAGES_TOPIC = '/hsm_ros_msg'
STOP_MESSAGE_FRAME_ID = '__CANCEL_NAV__'


def turtle_pose(x=0.0, y=0.0, theta=0.0, linear=0.0, angular=0.0):
    # the pose the simulator reports for its turtle
    msg = Pose()
    msg.x = float(x)
    msg.y = float(y)
    msg.theta = float(theta)
    msg.linear_velocity = float(linear)
    msg.angular_velocity = float(angular)
    return msg


def goal(x=0.0, y=0.0, frame_id=''):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    return msg


def cancellation():
    return goal(frame_id=STOP_MESSAGE_FRAME_ID)
