import pigpio
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

class ServoDriver(Node):
    def __init__(self):
        super().__init__('hal_servo_driver')
        self.subscriber = self.create_subscription(Int16, 'servo_angle', self.servo_angle_callback, 10)
        self.PWM_PIN = 18
        self.angle = 90
        self.pi = pigpio.pi()
        
    def servo_angle_callback(self, msg):
        self.angle = int((msg.data / 180) * 1700 + 600)
        self.pi.set_servo_pulsewidth(self.PWM_PIN, self.angle)
        
def main(args=None):
    rclpy.init(args=args)
    node = ServoDriver()
    while(rclpy.ok()):
        rclpy.spin_once(node)
        
    node.destroy_node()
    rclpy.shutdown()
        
        
if __name__ == '__main__':
    main()



