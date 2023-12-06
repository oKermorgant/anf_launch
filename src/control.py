#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor, FloatingPointRange
from example_interfaces.msg import Float64


class PID(Node):

    def __init__(self):
        super().__init__('control')
        
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.prev = None
        self.sub = self.create_subscription(Float64, 'angle_setpoint',
                                            self.read_setpoint, 1)

        Kp = ParameterDescriptor(
            name = 'Kp',
            floating_point_range = [FloatingPointRange(
                from_value = 0.,
                to_value = 2.)])
        self.Kp = self.declare_parameter(Kp.name, 0.2, Kp).value
        
        self.add_on_set_parameters_callback(self.cb_params)

    def timer_callback(self):
        '''
        This function is called by the timer
        '''
        self.get_logger().info(f'Kp is {self.Kp}, setpoint is {self.prev}')

    def cb_params(self, params):
        '''
        This function is called each time we ask to change the parameters of this node
        '''
        
        accept = False
        for param in params:
            self.get_logger().warn(f'Request: {param.name} -> {param.value}')
            if param.name == 'Kp' and param.value > 0:
                self.Kp = param.value
                accept = True
                    
        return SetParametersResult(successful=accept)

    def read_setpoint(self, msg: Float64):
        '''
        This function is called when we receive an incoming message
        '''

        if self.prev != msg.data:
            self.get_logger().info(f'Got new setpoint: {msg.data}')
            self.prev = msg.data


rclpy.init()
node = PID()
rclpy.spin(node)
node.destroy_node()
