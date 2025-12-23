#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math


class HeartPIDNode(Node):
    def __init__(self):
        # Node name
        super().__init__('heart_pid_node')

        # Creating subscriber -> Subscribing the node to topic /turtle1/pose to get the current position, velocity
        '''
        face show turtlesim/msg/Pose
        float32 x
        float32 y
        float32 theta

        float32 linear_velocity
        float32 angular_velocity
        '''
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Creating Publisher to publish the x and y angular velocity
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Pose values
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # PID values
        self.kp = 4.0
        self.ki = 0.0
        self.kd = 0.5

        self.prev_error = 0.0
        self.integral = 0.0

        # Heart parameter
        self.t = 0.0

        # Control loop timer (20 Hz)
        self.create_timer(0.05, self.control_loop)


    # Call back function for the subscriber 
    def pose_callback(self, msg: Pose):
        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

    # Defining the heart using parametric equations
    def heart_point(self):
        x = 16 * (math.sin(self.t) ** 3)
        y = (13 * math.cos(self.t) - 5 * math.cos(2 * self.t) - 2 * math.cos(3 * self.t) - math.cos(4 * self.t))

        # Scale + shift to turtlesim window
        x = x / 16 * 4 + 5.5
        y = y / 16 * 4 + 5.5

        return x, y


    def control_loop(self):
        x_d, y_d = self.heart_point()

        dx = x_d - self.x
        dy = y_d - self.y

        distance_error = math.sqrt(dx * dx + dy * dy)
        desired_theta = math.atan2(dy, dx)

        error = desired_theta - self.theta # angled error

        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi

        # PID
        derivative = (error - self.prev_error)/0.05

        pid_ang_vel = (self.kp * error + self.ki * self.integral + self.kd * derivative)

        self.prev_error = error

        cmd = Twist()
        cmd.linear.x = min(1.5, distance_error)
        cmd.angular.z = pid_ang_vel

        self.cmd_pub.publish(cmd)

        self.t += 0.01



def main(args = None):
    rclpy.init(args = args)

    node = HeartPIDNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()