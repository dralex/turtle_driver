# -----------------------------------------------------------------------------
# The ROS2 Turtle Driver
# -----------------------------------------------------------------------------
#
# Simple Turtle Driver node
# Based on the code from this online course: https://stepik.org/course/221157/
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

import rclpy
import rclpy.node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from turtle_driver.parameters import declare
from hsm_interfaces.msg import SimpleMessage
import math
from transforms3d.euler import euler2quat

TURTLE_ID = 'turtle1'
ODOM_TOPIC = '/odom'
TWIST_TOPIC = '/cmd_vel'
MESSAGES_TOPIC = '/hsm_ros_msg'
GOAL_TOPIC = '/goal_pose'
ODOM_FRAME = 'world'
TIMER_PERIOD = 0.1
MSG_QUEUE_LEN = 10
STOP_MESSAGE_FRAME_ID = '__CANCEL_NAV__'


class TurtleDriver(rclpy.node.Node):
    def __init__(self):
        rclpy.node.Node.__init__(self, 'turtle_driver')
        # the turtle name is not a topic: it is the name the simulator gives its turtle,
        # and the two turtle topics are built from it
        self.__turtle_name = declare(
            self, 'turtle_name', TURTLE_ID,
            'the name of the turtle of the simulator to drive')
        self.__odom_frame = declare(
            self, 'odom_frame', ODOM_FRAME,
            'the frame the published odometry is expressed in')
        # the control law: the turtle is driven to the goal by a proportional controller
        self.__linear_k = declare(
            self, 'linear_k', 1.0,
            'the proportional gain of the linear speed')
        self.__angular_k = declare(
            self, 'angular_k', 4.0,
            'the proportional gain of the angular speed')
        self.__linear_limit = declare(
            self, 'linear_limit', 2.0,
            'the largest linear speed the driver commands')
        self.__angular_limit = declare(
            self, 'angular_limit', 2.0,
            'the largest angular speed the driver commands')
        self.__arrival_tolerance = declare(
            self, 'arrival_tolerance', 0.01,
            'the distance to the goal at which the driver stops the turtle')
        self.__progress_epsilon = declare(
            self, 'progress_epsilon', 0.001,
            'the decrease of the distance to the goal which counts as progress')
        self.__no_progress_limit = declare(
            self, 'no_progress_limit', 2.0,
            'the time (s) without progress after which a collision is reported')
        control_period = declare(
            self, 'control_period', TIMER_PERIOD,
            'the period (s) of the control loop')
        queue_length = declare(
            self, 'message_queue_length', MSG_QUEUE_LEN,
            'the length of the ROS2 message queues')

        turtle_topic = '/{}/cmd_vel'.format(self.__turtle_name)
        pose_topic = '/{}/pose'.format(self.__turtle_name)
        self.__odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, queue_length)
        self.__twist_publisher = self.create_publisher(Twist, turtle_topic, queue_length)
        self.__msg_publisher = self.create_publisher(SimpleMessage, MESSAGES_TOPIC, queue_length)
        self.__twist_subscriber = self.create_subscription(Twist, TWIST_TOPIC,
                                                           self.__twist_callback, queue_length)
        self.__pose_subscriber = self.create_subscription(Pose, pose_topic,
                                                          self.__pose_callback, queue_length)
        self.__goal_subscriber = self.create_subscription(PoseStamped, GOAL_TOPIC,
                                                          self.__goal_callback, queue_length)
        self.__timer = self.create_timer(control_period, self.__move_turtle)
        self.get_logger().info('Turtle driver started ({})'.format(self.__turtle_name))

        self.__current_pose = Pose()
        self.__current_twist = Twist()

        self.__set_goal(None, None)
        self.__stop()

    def __send_message(self, code):
        msg = SimpleMessage()
        msg.code = code
        self.__msg_publisher.publish(msg)

    def __goal_callback(self, msg):
        if msg.header.frame_id == STOP_MESSAGE_FRAME_ID:
            self.__set_goal(None, None)
            self.__stop()
        else:
            self.__set_goal(msg.pose.position.x, msg.pose.position.y)
            self.get_logger().info('New goal: ({}, {})'.format(self.__x_goal, self.__y_goal))
            dx = self.__x_goal - self.__current_pose.x
            dy = self.__y_goal - self.__current_pose.y
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < self.__arrival_tolerance:
                self.__goal_reached()
                return

    def __set_goal(self, x, y):
        self.__x_goal = x
        self.__y_goal = y

        self.__last_distance = float('inf')
        self.__last_progress_time = self.get_clock().now().nanoseconds

    def __stop(self):
        self.get_logger().info('Stop')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.__twist_publisher.publish(twist)

    def __goal_reached(self):
        # the arrival is detected here to stop the turtle, but MOVE_COMPLETED is reported
        # by the navigation module of the API: the driver is a platform adapter and the
        # HSM events have to be raised the same way on the platforms without such driver
        self.get_logger().info('Goal ({}, {}) reached!'.format(self.__x_goal, self.__y_goal))
        self.__set_goal(None, None)
        self.__stop()

    def __pose_callback(self, msg):
        self.__current_pose = msg

        # Publish the turtle odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.__odom_frame
        odom.child_frame_id = self.__turtle_name
        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.position.z = 0.0
        q = euler2quat(0, 0, msg.theta)
        odom.pose.pose.orientation.w = q[0]
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.twist.twist.linear.x = float(msg.linear_velocity)
        odom.twist.twist.angular.z = float(msg.angular_velocity)
        self.__odom_publisher.publish(odom)

    def __move_turtle(self):
        if self.__current_twist != Twist():
            self.__twist_publisher.publish(self.__current_twist)
            # do not break the goal logic here - the HSM programmer should control this by herself

        if self.__x_goal is None:
            return

        dx = self.__x_goal - self.__current_pose.x
        dy = self.__y_goal - self.__current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        if distance < self.__arrival_tolerance:
            self.__goal_reached()
            return

        current_time = self.get_clock().now().nanoseconds
        if distance < self.__last_distance - self.__progress_epsilon:
            self.__last_progress_time = current_time
            self.__last_distance = distance
        else:
            time_no_progress = (current_time - self.__last_progress_time) / 1e9
            if time_no_progress > self.__no_progress_limit:
                self.get_logger().warn('Goal ({}, {}) is unreachable!'.format(self.__x_goal, self.__y_goal))
                self.__set_goal(None, None)
                self.__stop()
                self.__send_message(SimpleMessage.MSG_NAVIGATION_COLLISION_DETECTED)
                return

        desired_angle = math.atan2(dy, dx)
        angle_error = desired_angle - self.__current_pose.theta

        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()
        twist.linear.x = self.__linear_k * distance
        if twist.linear.x > self.__linear_limit:
            twist.linear.x = self.__linear_limit

        twist.angular.z = self.__angular_k * angle_error
        if twist.angular.z > self.__angular_limit:
            twist.angular.z = self.__angular_limit
        elif twist.angular.z < -self.__angular_limit:
            twist.angular.z = -self.__angular_limit

        self.__twist_publisher.publish(twist)

    def __twist_callback(self, msg):
        self.__twist_publisher.publish(msg)
        self.__current_twist = msg


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDriver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
