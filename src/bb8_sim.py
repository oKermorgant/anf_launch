#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from math import sin, cos, pi, atan2
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

a = 2
b = 3
dt = 0.01


def clamp(v):
    return -pi/4 if v < -pi/4 else (pi/4 if v > pi/4 else v)


class BB8(Node):
    def __init__(self):
        super().__init__('control')
        
        self.tf = TransformStamped()
        
        self.tf.child_frame_id = 'bb8/base_link'
        self.tf.header.frame_id = 'map'
        
        self.prev_theta = 0.
        self.t = 0.
        
        self.br = TransformBroadcaster(self)
        
        self.js = JointState()
        self.js.name = ['wheel', 'torso','neck']
        self.js.position = [0.,0.,0.]
        
        self.js_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(dt, self.update)

    def update(self):
        
        self.t += dt
        c, s = cos(.5*self.t), sin(.5*self.t)
        x = (a + b*c)*c
        y = (a + b*c)*s
        vx = -a*s-2*b*c*s
        vy = a*c + b - 2*b*s*s
        theta = atan2(vy, vx)
        
        now_stamp = self.get_clock().now().to_msg()

        self.tf.header.stamp = now_stamp
        self.tf.transform.translation.x = x
        self.tf.transform.translation.y = y
        self.tf.transform.rotation.z = sin(theta/2)
        self.tf.transform.rotation.w = cos(theta/2)
        
        self.br.sendTransform(self.tf)
        
        # compute corresponding js
        # get linear velocity in robot frame
        v = vx*cos(theta) + vy*sin(theta)
        w = (theta-self.prev_theta)/dt
        self.js.position[0] += 3.7*v*dt
        self.js.position[1] = clamp(v*pi/12)
        self.js.position[2] = clamp(w*pi/8)
        self.js.header.stamp = now_stamp
        
        self.prev_theta = theta
        
        self.js_pub.publish(self.js)

    
rclpy.init()

bb8 = BB8()

rclpy.spin(bb8)
bb8.destroy_node()
rclpy.shutdown()
