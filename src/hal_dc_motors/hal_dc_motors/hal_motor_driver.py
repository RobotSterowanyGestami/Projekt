#!/usr/bin/env python3
import rclpy
import time
import RPi.GPIO as GPIO
from rclpy.node import Node

from std_msgs.msg import Int8




class MotorDriver(Node):

    def __init__(self):
        super().__init__('hal_motor_driver')
        #self.publisher_ = self.create_publisher(Int, 'topic', 10)
        self.subscriber_ = self.create_subscription(Int8, 'motor_speed', self.motor_speed_callback, 10)
        GPIO.setmode(GPIO.BCM)
        self.PWM_PIN = 4
        GPIO.setup(self.PWM_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.PWM_PIN, 100)
        self.speed = 0
        self.pwm.start(0)
        self.MotorA = 9
        self.MotorB = 10
        GPIO.setup(self.MotorA, GPIO.OUT)
        GPIO.setup(self.MotorB, GPIO.OUT)

    def motor_speed_callback(self, msg):
        self.speed = msg.data
        if self.speed >= 0:
            GPIO.output(self.MotorA, GPIO.HIGH)
            GPIO.output(self.MotorB, GPIO.LOW)

        else:
            GPIO.output(self.MotorA, GPIO.LOW)
            GPIO.output(self.MotorB, GPIO.HIGH)
            self.speed = -self.speed
        self.pwm.ChangeDutyCycle(self.speed)
        GPIO.output(self.PWM_PIN, GPIO.HIGH)


def main(args=None):
    rclpy.init(args=args)

    node = MotorDriver()
    while(rclpy.ok()):
        rclpy.spin_once(node)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
