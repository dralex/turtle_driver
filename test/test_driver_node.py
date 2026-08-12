# -----------------------------------------------------------------------------
# The ROS2 Turtle Driver
# -----------------------------------------------------------------------------
#
# The ROS2 contract of the turtle driver node
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

# The driver has no services: it is driven by the topics of the simulator and of the API.
# The times of the control loop are the parameters of the node, so the tests shrink them
# instead of waiting for the real ones.

import math

import pytest

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from conftest import GOAL_TOPIC, ODOM_TOPIC, TWIST_TOPIC, cancellation, goal, turtle_pose
from hsm_interfaces.msg import SimpleMessage
from turtle_driver.turtle_driver import TurtleDriver

COLLISION_DETECTED = SimpleMessage.MSG_NAVIGATION_COLLISION_DETECTED

pytestmark = pytest.mark.node


def fast():
    # a control loop fast enough for a test, and a goal declared unreachable quickly
    return {'control_period': 0.02, 'no_progress_limit': 0.3}


def pose_topic(name='turtle1'):
    return '/{}/pose'.format(name)


def command_topic(name='turtle1'):
    return '/{}/cmd_vel'.format(name)


def test_the_pose_is_published_as_odometry(node_factory):
    ctx = node_factory(TurtleDriver, **fast())
    odometry = ctx.probe.record(ODOM_TOPIC, Odometry)
    ctx.probe.publish(pose_topic(), turtle_pose(x=3.0, y=4.0, linear=0.5, angular=0.25))
    assert ctx.probe.wait_for(lambda: len(odometry) >= 1)
    first = odometry[0]
    assert (first.pose.pose.position.x, first.pose.pose.position.y) == (3.0, 4.0)
    assert first.twist.twist.linear.x == pytest.approx(0.5)
    assert first.twist.twist.angular.z == pytest.approx(0.25)


def test_the_heading_becomes_a_quaternion(node_factory):
    ctx = node_factory(TurtleDriver, **fast())
    odometry = ctx.probe.record(ODOM_TOPIC, Odometry)
    ctx.probe.publish(pose_topic(), turtle_pose(theta=math.pi / 2.0))
    assert ctx.probe.wait_for(lambda: len(odometry) >= 1)
    orientation = odometry[0].pose.pose.orientation
    # a quarter turn around the vertical axis
    assert orientation.z == pytest.approx(math.sin(math.pi / 4.0))
    assert orientation.w == pytest.approx(math.cos(math.pi / 4.0))


def test_the_odometry_frames_are_parameters(node_factory):
    ctx = node_factory(TurtleDriver, odom_frame='map', **fast())
    odometry = ctx.probe.record(ODOM_TOPIC, Odometry)
    ctx.probe.publish(pose_topic(), turtle_pose())
    assert ctx.probe.wait_for(lambda: len(odometry) >= 1)
    assert odometry[0].header.frame_id == 'map'
    assert odometry[0].child_frame_id == 'turtle1'


def test_the_turtle_name_moves_the_topics(node_factory):
    ctx = node_factory(TurtleDriver, turtle_name='robot2', **fast())
    odometry = ctx.probe.record(ODOM_TOPIC, Odometry)
    # the driver listens to the turtle it was given and not to the default one
    ctx.probe.publish(pose_topic('robot2'), turtle_pose(x=1.0))
    assert ctx.probe.wait_for(lambda: len(odometry) >= 1)
    assert odometry[0].child_frame_id == 'robot2'
    assert odometry[0].pose.pose.position.x == 1.0


def test_the_velocity_is_passed_to_the_turtle(node_factory):
    ctx = node_factory(TurtleDriver, **fast())
    commands = ctx.probe.record(command_topic(), Twist)
    driving = Twist()
    driving.linear.x = 0.4
    ctx.probe.publish(TWIST_TOPIC, driving)
    assert ctx.probe.wait_for(lambda: any(c.linear.x == pytest.approx(0.4) for c in commands))


def test_the_velocity_is_repeated(node_factory):
    ctx = node_factory(TurtleDriver, **fast())
    commands = ctx.probe.record(command_topic(), Twist)
    driving = Twist()
    driving.linear.x = 0.4
    ctx.probe.publish(TWIST_TOPIC, driving)
    # the simulator stops a turtle which is not commanded, so the driver keeps sending
    # the last velocity of the diagram on every turn of its loop
    assert ctx.probe.wait_for(
        lambda: len([c for c in commands if c.linear.x == pytest.approx(0.4)]) >= 3)


def test_a_goal_makes_the_driver_steer(node_factory):
    ctx = node_factory(TurtleDriver, **fast())
    commands = ctx.probe.record(command_topic(), Twist)
    ctx.probe.publish(pose_topic(), turtle_pose(x=0.0, y=0.0))
    ctx.probe.publish(GOAL_TOPIC, goal(x=5.0, y=0.0))
    assert ctx.probe.wait_for(lambda: any(c.linear.x > 0.0 for c in commands))


def test_the_commanded_speed_is_limited(node_factory):
    ctx = node_factory(TurtleDriver, linear_limit=0.5, linear_k=10.0, **fast())
    commands = ctx.probe.record(command_topic(), Twist)
    ctx.probe.publish(pose_topic(), turtle_pose(x=0.0, y=0.0))
    ctx.probe.publish(GOAL_TOPIC, goal(x=9.0, y=0.0))
    assert ctx.probe.wait_for(lambda: any(c.linear.x > 0.0 for c in commands))
    assert max(c.linear.x for c in commands) <= 0.5


def test_the_arrival_stops_the_turtle(node_factory):
    ctx = node_factory(TurtleDriver, arrival_tolerance=0.5, **fast())
    commands = ctx.probe.record(command_topic(), Twist)
    ctx.probe.publish(pose_topic(), turtle_pose(x=1.0, y=0.0))
    ctx.probe.publish(GOAL_TOPIC, goal(x=1.2, y=0.0))
    assert ctx.probe.wait_for(lambda: any(c == Twist() for c in commands))


def test_the_arrival_reports_no_event(node_factory):
    ctx = node_factory(TurtleDriver, arrival_tolerance=0.5, **fast())
    ctx.probe.publish(pose_topic(), turtle_pose(x=1.0, y=0.0))
    ctx.probe.publish(GOAL_TOPIC, goal(x=1.2, y=0.0))
    # the driver is a platform adapter: the arrival is reported by the navigation module
    # of the API, so that a platform without such a driver reports it the same way
    ctx.probe.spin(0.3)
    assert ctx.probe.event_codes() == []


def test_a_goal_which_is_not_approached_is_reported(node_factory):
    ctx = node_factory(TurtleDriver, **fast())
    # the turtle does not move, so the distance to the goal never decreases
    ctx.probe.publish(pose_topic(), turtle_pose(x=0.0, y=0.0))
    ctx.probe.publish(GOAL_TOPIC, goal(x=5.0, y=0.0))
    ctx.probe.wait_for_event(COLLISION_DETECTED, timeout=3.0)


def test_the_cancellation_stops_the_turtle(node_factory):
    ctx = node_factory(TurtleDriver, **fast())
    commands = ctx.probe.record(command_topic(), Twist)
    ctx.probe.publish(pose_topic(), turtle_pose(x=0.0, y=0.0))
    ctx.probe.publish(GOAL_TOPIC, goal(x=5.0, y=0.0))
    assert ctx.probe.wait_for(lambda: any(c.linear.x > 0.0 for c in commands))
    ctx.probe.publish(GOAL_TOPIC, cancellation())
    assert ctx.probe.wait_for(lambda: commands[-1] == Twist())


def test_a_cancelled_goal_is_not_reported_unreachable(node_factory):
    ctx = node_factory(TurtleDriver, **fast())
    ctx.probe.publish(pose_topic(), turtle_pose(x=0.0, y=0.0))
    ctx.probe.publish(GOAL_TOPIC, goal(x=5.0, y=0.0))
    ctx.probe.publish(GOAL_TOPIC, cancellation())
    # the goal is gone, so the driver has nothing to fail to approach
    ctx.probe.spin(0.6)
    assert ctx.probe.count_events(COLLISION_DETECTED) == 0
