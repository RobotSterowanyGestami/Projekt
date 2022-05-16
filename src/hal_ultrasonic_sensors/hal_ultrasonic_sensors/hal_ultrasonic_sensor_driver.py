#Libraries
import RPi.GPIO as GPIO
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
GPIO_TRIGGER_1 = 23
GPIO_ECHO_1 = 24
GPIO_TRIGGER_2 = 5
GPIO_ECHO_2 = 6
GPIO_TRIGGER_3 = 0
GPIO_ECHO_3 = 25
GPIO_TRIGGER_4 = 2
GPIO_ECHO_4 = 3

 
#set GPIO direction (IN / OUT)



class UltrasoundSensorDriver(Node):
    
    def __init__(self, echo, trigger, name):
        super().__init__('hal_ultrasound_sensor_driver')
        self.publisher = self.create_publisher(Float32, name, 10)
        GPIO.setup(trigger, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
        self.trigger = trigger
        self.echo = echo
        timer_period = 0.25
        self.timer = self.create_timer(timer_period, self.run)
        
    def distance(self, trigger, echo):
        GPIO.output(trigger, True)
        time.sleep(0.00001)
        GPIO.output(trigger, False)
     
        StartTime = time.time()
        StopTime = time.time()
     
        while GPIO.input(echo) == 0:
            StartTime = time.time()
     
        while GPIO.input(echo) == 1:
            StopTime = time.time()
     
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
     
        return distance
    
    def run(self):
        msg = Float32()
        msg.data = self.distance(self.trigger, self.echo)        


def main(args=None):
    rclpy.init(args=args)
    
    front_right_ultrasound_sensor_driver = UltrasoundSensorDriver(GPIO_ECHO_1, GPIO_TRIGGER_1, 'front_right_distance')
    rear_right_ultrasound_sensor_driver = UltrasoundSensorDriver(GPIO_ECHO_2, GPIO_TRIGGER_2, 'rear_right_distance')
    rear_left_ultrasound_sensor_driver = UltrasoundSensorDriver(GPIO_ECHO_3, GPIO_TRIGGER_3, 'rear_left_distance')
    front_left_ultrasound_sensor_driver = UltrasoundSensorDriver(GPIO_ECHO_4, GPIO_TRIGGER_4, 'front_left_distance')
    
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(front_right_ultrasound_sensor_driver)
    executor.add_node(rear_right_ultrasound_sensor_driver)
    executor.add_node(rear_left_ultrasound_sensor_driver)
    executor.add_node(front_left_ultrasound_sensor_driver)

    while rclpy.ok():
        executor.spin_once()

        
    front_right_ultrasound_sensor_driver.destroy_node()
    rear_right_ultrasound_sensor_driver.destroy_node()
    rear_left_ultrasound_sensor_driver.destroy_node()
    front_left_ultrasound_sensor_driver.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()
    
 
if __name__ == '__main__':
    main()