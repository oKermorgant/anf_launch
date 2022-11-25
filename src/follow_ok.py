#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from math import *

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange

class Follower(Node):

    def __init__(self):
        super().__init__('follower')
        self.publisher_ = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.apply_cmd)

        self.Kv = self.declare_parameter("Kv", 2.).value
        self.Kw = self.declare_parameter("Kw", 2.).value
        self.d = self.declare_parameter("d", 1.).value

        # subscribe to my own pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.my_pose,
            10)
        self.me = None

        # subscribe to turtle1
        self.target_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.target_pose,
            10)
        self.target = None

        self.add_on_set_parameters_callback(self.param_change)


    def param_change(self,  params):

        accept = True
        for param in params:
            self.get_logger().warn('Request: {} -> {}'.format(param.name, param.value))
            if param.value < 0:
                accept = False
            else:
                if param.name == 'Kv': self.Kv = param.value
                elif param.name == 'Kw': self.Kw = param.value
                elif param.name == 'd': self.d = param.value

        return SetParametersResult(successful=accept)

    def my_pose(self, pose):
        self.me = pose

    def target_pose(self, pose):
        self.target = pose

    def apply_cmd(self):

        if self.me is None or self.target is None:
            return

        dx = self.target.x - self.me.x
        dy = self.target.y - self.me.y

        theta = atan2(dy,dx) - self.me.theta
        theta = (theta+pi) % (2*pi) - pi

        c,s = cos(self.me.theta), sin(self.me.theta)

        x = dx*c+dy*s

        cmd = Twist()

        cmd.linear.x = self.Kv*(x - self.d)
        cmd.angular.z = self.Kw*theta
        self.publisher_.publish(cmd)


if __name__ == '__main__':
    rclpy.init()

    minimal_publisher = Follower()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

