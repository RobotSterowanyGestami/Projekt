#!/usr/bin/env python3
import RPi.GPIO as gpio
import time
import sys
import signal

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
import RPi.GPIO as gpio

PIN1=5
PIN2=6
PIN3=23
PIN4=24
OUTPIN=17


class Sensor(Node):

    def __init__(self):
        super().__init__('hal_front_ultrasonic_sensor_driver')
        self.publisher_ = self.create_publisher(Float32, 'front_right_distance', 10)
        timer_period = 0.001 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.echo1=False
        gpio.setmode(gpio.BCM)
        gpio.setup(PIN1,gpio.IN)
        gpio.setup(PIN2,gpio.IN)
        gpio.setup(PIN3,gpio.IN)
        gpio.setup(PIN4,gpio.IN)
        gpio.setup(OUTPIN,gpio.OUT)
        gpio.output(OUTPIN, gpio.LOW)
        gpio.add_event_detect(PIN1,RISING,callback1)
        self.start_time=None
        self.stop_time=None

    def timer_callback(self):
        msg = Float32()
        msg.data = self.i
        self.publisher_.publish(msg)
        self.i += 1

    def callback1(self):
        self.echo1=True

    def run(self):
        while rclpy.ok():
            start_time=
            gpio.output(OUTPIN,gpio.HIGH)
            gpio.output(OUTPIN,gpio.LOW)
            if self.echo1==True:
                




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    while rclpy.ok():
        rclpy.spin_once(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

