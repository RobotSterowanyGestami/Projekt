#!/usr/bin/env python3
import rclpy
import time
import pigpio
from rclpy.node import Node

from std_msgs.msg import Float32

MAX_SPEED = 0.22


class MotorDriver(Node):

    def __init__(self):
        super().__init__('hal_motor_driver')
        self.left_motor_subscriber = self.create_subscription(Float32, 'post_pid_left_motor_speed', self.left_motor_speed_callback, 10)
        self.right_motor_subscriber = self.create_subscription(Float32, 'post_pid_right_motor_speed', self.right_motor_speed_callback, 10)
        self.PWM_PIN_L = 13
        self.PWM_PIN_R = 12
        self.speed_L = 0
        self.MotorA_L = 8
        self.MotorB_L = 7
        self.speed_R = 0
        self.MotorA_R = 20
        self.MotorB_R = 21
        self.pi = pigpio.pi()

    def left_motor_speed_callback(self, msg):
        self.speed_L = msg.data
        if self.speed_L >= 0:
            self.pi.write(self.MotorA_L, 1)
            self.pi.write(self.MotorB_L, 0)

        else:
            self.pi.write(self.MotorA_L, 0)
            self.pi.write(self.MotorB_L, 1)
            self.speed_L = -self.speed_L
        self.speed_L = int(self.speed_L / MAX_SPEED * 255)
        self.pi.set_PWM_dutycycle(self.PWM_PIN_L, self.speed_L)
        
    def right_motor_speed_callback(self, msg):
        self.speed_R = msg.data
        if self.speed_R >= 0:
            self.pi.write(self.MotorA_R, 0)
            self.pi.write(self.MotorB_R, 1)

        else:
            self.pi.write(self.MotorA_R, 1)
            self.pi.write(self.MotorB_R, 0)
            self.speed_R = -self.speed_R
        self.speed_R = int(self.speed_R / MAX_SPEED * 255)
        self.pi.set_PWM_dutycycle(self.PWM_PIN_R, self.speed_R)

def main(args=None):
    rclpy.init(args=args)

    node = MotorDriver()
    while(rclpy.ok()):
        rclpy.spin_once(node)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
