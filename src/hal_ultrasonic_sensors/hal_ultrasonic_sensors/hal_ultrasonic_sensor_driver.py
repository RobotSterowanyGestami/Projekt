#Libraries
import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER = 17
GPIO_ECHO_1 = 5
GPIO_ECHO_2 = 6
 
#set GPIO direction (IN / OUT)



class UltrasoundSensorDriver(Node):
    
    def __init__(self):
        super().__init__('hal_ultrasound_sensor_driver')
        self.front_right_publisher = self. create_publisher(Float32, 'front_right_distance', 10)
        self.front_left_publisher = self.create_publisher(Float32, 'front_left_distance', 10)
        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(GPIO_ECHO_1, GPIO.IN)
        GPIO.setup(GPIO_ECHO_2, GPIO.IN)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.run)
        self.i = 0
        
    def distance(self, trigger, echo):
        # set Trigger to HIGH
        GPIO.output(trigger, True)
     
        # set Trigger after 0.01ms to LOW
        time.sleep(0.00001)
        GPIO.output(trigger, False)
     
        StartTime = time.time()
        StopTime = time.time()
     
        # save StartTime
        while GPIO.input(echo) == 0:
            StartTime = time.time()
     
        # save time of arrival
        while GPIO.input(echo) == 1:
            StopTime = time.time()
     
        # time difference between start and arrival
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
     
        return distance
    
    def run(self):
        msg = Float32()
        msg.data = self.distance(GPIO_TRIGGER, GPIO_ECHO_1)
        self.front_right_publisher.publish(msg)
        self.get_logger().info('Publishing right: "%f"' % msg.data)
        msg.data = self.distance(GPIO_TRIGGER, GPIO_ECHO_2)
        self.front_left_publisher.publish(msg)
        self.get_logger().info('Publishing left: "%f"' % msg.data)
        
        


def main(args=None):
    rclpy.init(args=args)
    
    ultrasound_sensor_driver = UltrasoundSensorDriver()
    
    while rclpy.ok():
        rclpy.spin_once(ultrasound_sensor_driver)
        time.sleep(1)

        
    ultrasound_sensor_driver.destron_node()
    rclpy.shutdown()
    GPIO.cleanup()
    
 
if __name__ == '__main__':
    main()