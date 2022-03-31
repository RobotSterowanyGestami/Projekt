#!/usr/bin/env python3
import rclpy
import time
import RPi.GPIO as GPIO
from rclpy.node import Node

from std_msgs.msg import Int8




class MotorDriver(Node):

    def __init__(self):
        super().__init__('motor_driver')
        #self.publisher_ = self.create_publisher(Int, 'topic', 10)
        self.subscriber_ = self.create_subscription(Int8, 'motor_speed', self.motor_speed_callback, 10)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(4, GPIO.OUT)
        self.pwm = GPIO.PWM(4, 100)
        self.speed = 0
        self.pwm.start(0)
        self.Motor1A = 9
        self.Motor1B = 10
        GPIO.setup(self.Motor1A, GPIO.OUT)
        GPIO.setup(self.Motor1B, GPIO.OUT)

    def motor_speed_callback(self, msg):
        self.speed = msg.data
        if self.speed >= 0:
            GPIO.output(self.Motor1A, GPIO.HIGH)
            GPIO.output(self.Motor1B, GPIO.LOW)
        else:
            GPIO.output(self.Motor1A, GPIO.LOW)
            GPIO.output(self.Motor1B, GPIO.HIGH)
            self.speed = -self.speed
        self.pwm.ChangeDutyCycle(self.speed)
        GPIO.output(4, GPIO.HIGH)


def main(args=None):
    rclpy.init(args=args)

    node = MotorDriver()
    while(rclpy.ok()):
        rclpy.spin_once(node)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
