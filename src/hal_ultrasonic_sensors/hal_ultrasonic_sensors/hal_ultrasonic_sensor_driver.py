#Libraries
import pigpio
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor
  
#set GPIO Pins
GPIO_TRIGGER_1 = 23
GPIO_ECHO_1 = 24
GPIO_TRIGGER_2 = 5
GPIO_ECHO_2 = 6
GPIO_TRIGGER_3 = 0
GPIO_ECHO_3 = 25
GPIO_TRIGGER_4 = 2
GPIO_ECHO_4 = 3


class UltrasoundSensorDriver(Node):
    
    def __init__(self, echo, trigger, name):
        super().__init__('hal_ultrasound_sensor_driver')
        self.name = name
        self.publisher = self.create_publisher(Float32, self.name, 10)
        self.trigger = trigger
        self.echo = echo
        self.timer_period = 0.25
        self.timer = self.create_timer(self.timer_period, self.run)
        self.pi = pigpio.pi()
        
    def distance(self, trigger, echo):
        self.pi.write(trigger, 1)
        time.sleep(0.00001)
        self.pi.write(trigger, 0)
     
        StartTime = time.time()
        StopTime = time.time()
     
        while self.pi.read(echo) == 0:
            if time.time() - StopTime > self.timer_period:
                break
            StartTime = time.time()
     
        while self.pi.read(echo) == 1:
            StopTime = time.time()
     
        TimeElapsed = StopTime - StartTime
        # multiply with the sonic speed (34300 cm/s)
        # and divide by 2, because there and back
        distance = (TimeElapsed * 34300) / 2
     
        return distance
    
    def run(self):
        msg = Float32()
        msg.data = self.distance(self.trigger, self.echo)
        #msg.data = 0.1
        self.publisher.publish(msg)
        #self.get_logger().info('%s' % self.name)



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