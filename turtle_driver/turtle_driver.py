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
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see https://www.gnu.org/licenses/
#
# -----------------------------------------------------------------------------

import rclpy
import rclpy.node
from geometry_msgs.msg import Twist, PoseStamped, Pose
import math

TURTLE_TOPIC = '/turtle1/cmd_vel'
POSE_TOPIC = '/turtle1/pose'
GOAL_TOPIC = '/goal_topic'
TIMER_PERIOD = 0.1
MSG_QUEUE_LEN = 10
STOP_MESSAGE_FRAME_ID = '__CANCEL_NAV__'

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
     
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians

class TurtleDriver(rclpy.node.Node):
    def __init__(self):
        rclpy.node.Node.__init__(self, 'turtle_driver')
        self.__twist_publisher = self.create_publisher(Twist, TURTLE_TOPIC, MSG_QUEUE_LEN)
        self.__pose_subscriber = self.create_subscription(Pose, POSE_TOPIC, self.__pose_callback, MSG_QUEUE_LEN)
        self.__goal_subscriber = self.create_subscription(PoseStamped, GOAL_TOPIC, self.__goal_callback, MSG_QUEUE_LEN)
        self.__timer = self.create_timer(TIMER_PERIOD, self.__move_turtle)
        self.get_logger().info('Turtle driver started')

        self.__current_pose = Pose()
        self.__current_theta = 0.0
        self.__linear_k = 1.0
        self.__angular_k = 4.0
        self.__arrival_tolerance = 0.01
        self.__no_progress_limit = 2.0

        self.__set_goal(None, None)
        self.__stop()

    def __goal_callback(self, msg):
        if msg.header.frame_id == STOP_MESSAGE_FRAME_ID:
            self.__set_goal(None, None)
            self.__stop()
        else:
            self.__set_goal(msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info('New goal: ({}, {})'.format(self.__x_goal, self.__y_goal))
        
    def __set_goal(self, x, y):
        self.__x_goal = x
        self.__y_goal = y

        self.__reached = False
        self.__unreachable = False

        self.__last_distance = float('inf')
        self.__last_progress_time = self.get_clock().now().nanoseconds

    def __stop(self):
        self.get_logger().info('Stop')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.__twist_publisher.publish(twist)
        
    def __pose_callback(self, msg):
        self.get_logger().info('New pose: {}'.format(msg))
        self.__current_pose = msg
        orientation_q = msg.pose.orientation
        orientation_list = (orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        roll, pitch, yaw = euler_from_quaternion(*orientation_list)
        self.__current_theta = yaw

    def __move_turtle(self):
        if self.__x_goal is None or self.__reached or self.__unreachable:
            return

        self.get_logger().info('Moving: ({}, {}), {}'.format(self.__current_pose.position.x,
                                                             self.__current_pose.position.y,
                                                             self.__current_theta))
        
        dx = self.__x_goal - self.__current_pose.position.x
        dy = self.__y_goal - self.__current_pose.position.y
        distance = math.sqrt(dx*dx + dy*dy)
        if distance < self.__arrival_tolerance:
            self.__stop()
            self.__reached = True
            self.get_logger().info('Goal reached!')
            return

        current_time = self.get_clock().now().nanoseconds
        if distance < self.__last_distance - 0.001:
            self.__last_progress_time = current_time
            self.__last_distance = distance
        else:
            time_no_progress = (current_time - self.__last_progress_time) / 1e9
            if time_no_progress > self.__no_progress_limit:
                self.__stop()
                self.__unreachable = True
                self.get_logger().warn('Goal is unreachable!')
                return

        desired_angle = math.atan2(dy, dx)
        angle_error = desired_angle - self.__current_theta

        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        twist = Twist()
        twist.linear.x = self.__linear_k * distance
        if twist.linear.x > 2.0:
            twist.linear.x = 2.0

        twist.angular.z = self.__angular_k * angle_error
        if twist.angular.z > 2.0:
            twist.angular.z = 2.0
        elif twist.angular.z < -2.0:
            twist.angular.z = -2.0

        self.__twist_publisher.publish(twist)        
        self.get_logger().info('Twist: {}'.format(twist))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
