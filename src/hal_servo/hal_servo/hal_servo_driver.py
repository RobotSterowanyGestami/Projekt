import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class ServoDriver(Node):
    def __init__(self):
        super().__init__('hal_servo_driver')
        self.subscriber = self.create_subscription(Int8, 'servo_angle', self.servo_angle_callback, 10)
        self.PWM_PIN = 12
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PWM_PIN, GPIO.OUT)
        self.pwm = GPIO.PWM(self.PWM_PIN, 50)
        self.angle = 90
        self.pwm.start(0)
        
    def servo_angle_callback(self, msg):
        self.angle = (msg.data / 180) * 11 + 1.5
        self.pwm.ChangeDutyCycle(self.angle)
        
def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    while(rclpy.ok()):
        rclpy.spin_once(node)
        
    node.destroy_node()
    rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()
        




#servoPIN = 12
#GPIO.setmode(GPIO.BCM)
#GPIO.setup(servoPIN, GPIO.OUT)

#p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
#p.start(0) # Initialization
#try:
#  while True:
#    p.ChangeDutyCycle(1.5)
#    time.sleep(1)
#    p.ChangeDutyCycle(7)
#    time.sleep(1)
#    p.ChangeDutyCycle(12.5)
#    time.sleep(1)
#    p.ChangeDutyCycle(7.5)
#    time.sleep(1)

#except KeyboardInterrupt:
 # p.stop()
  #GPIO.cleanup()
