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
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from hsm_interfaces.msg import SimpleMessage
import math
from transforms3d.euler import euler2quat

TURTLE_ID = 'turtle1'
TURTLE_TOPIC = '/{}/cmd_vel'.format(TURTLE_ID)
POSE_TOPIC = '/{}/pose'.format(TURTLE_ID)
ODOM_TOPIC = '/odom'
MESSAGES_TOPIC = '/hsm_ros_msg'
GOAL_TOPIC = '/goal_pose'
TIMER_PERIOD = 0.1
MSG_QUEUE_LEN = 10
STOP_MESSAGE_FRAME_ID = '__CANCEL_NAV__'

class TurtleDriver(rclpy.node.Node):
    def __init__(self):
        rclpy.node.Node.__init__(self, 'turtle_driver')
        self.__odom_publisher = self.create_publisher(Odometry, ODOM_TOPIC, MSG_QUEUE_LEN)
        self.__twist_publisher = self.create_publisher(Twist, TURTLE_TOPIC, MSG_QUEUE_LEN)
        self.__msg_publisher = self.create_publisher(SimpleMessage, MESSAGES_TOPIC, MSG_QUEUE_LEN)
        self.__pose_subscriber = self.create_subscription(Pose, POSE_TOPIC, self.__pose_callback, MSG_QUEUE_LEN)
        self.__goal_subscriber = self.create_subscription(PoseStamped, GOAL_TOPIC, self.__goal_callback, MSG_QUEUE_LEN)
        self.__timer = self.create_timer(TIMER_PERIOD, self.__move_turtle)
        self.get_logger().info('Turtle driver started')

        self.__current_pose = Pose()
        self.__linear_k = 1.0
        self.__angular_k = 4.0
        self.__arrival_tolerance = 0.01
        self.__no_progress_limit = 2.0

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
        self.get_logger().info('Goal ({}, {}) reached!'.format(self.__x_goal, self.__y_goal))
        self.__set_goal(None, None)
        self.__stop()
        self.__send_message(SimpleMessage.MSG_NAVIGATION_MOVE_COMPLETED)
        
    def __pose_callback(self, msg):
        self.__current_pose = msg

        # Publish the turtle odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = TURTLE_ID
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
        if self.__x_goal is None:
            return
        
        dx = self.__x_goal - self.__current_pose.x
        dy = self.__y_goal - self.__current_pose.y
        distance = math.sqrt(dx*dx + dy*dy)
        if distance < self.__arrival_tolerance:
            self.__goal_reached()
            return

        current_time = self.get_clock().now().nanoseconds
        if distance < self.__last_distance - 0.001:
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
        if twist.linear.x > 2.0:
            twist.linear.x = 2.0

        twist.angular.z = self.__angular_k * angle_error
        if twist.angular.z > 2.0:
            twist.angular.z = 2.0
        elif twist.angular.z < -2.0:
            twist.angular.z = -2.0

        self.__twist_publisher.publish(twist)        

def main(args=None):
    rclpy.init(args=args)
    node = TurtleDriver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
