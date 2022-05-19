#!/usr/bin/env python3
import rclpy
import time
import pigpio
from rclpy.node import Node

from std_msgs.msg import Int8




class MotorDriver(Node):

    def __init__(self):
        super().__init__('hal_motor_driver')
        #self.publisher_ = self.create_publisher(Int, 'topic', 10)
        self.subscriber_ = self.create_subscription(Int8, 'motor_speed', self.motor_speed_callback, 10)
        self.PWM_PIN = 13
        self.speed = 0
        self.MotorA = 8
        self.MotorB = 7
        self.pi = pigpio.pi()

    def motor_speed_callback(self, msg):
        self.speed = msg.data
        if self.speed >= 0:
            self.pi.write(self.MotorA, 1)
            self.pi.write(self.MotorB, 0)

        else:
            self.pi.write(self.MotorA, 0)
            self.pi.write(self.MotorB, 1)
            self.speed = -self.speed
        self.speed = int(self.speed / 100 * 255)
        self.pi.set_PWM_dutycycle(self.PWM_PIN, self.speed)

def main(args=None):
    rclpy.init(args=args)

    node = MotorDriver()
    while(rclpy.ok()):
        rclpy.spin_once(node)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
