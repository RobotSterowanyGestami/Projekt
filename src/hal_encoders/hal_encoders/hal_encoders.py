import pigpio
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor

Enc_A_1 = 4
Enc_B_1 = 1
Enc_A_2 = 16
Enc_B_2 = 17
num_of_ticks_per_round = 960
wheel = 0.21 #m

class EncoderDriver(Node):
    def __init__(self, gpioA, gpioB, name, direction):
        super().__init__('hal_encoder_driver')
        self.publisher = self.create_publisher(Float32, name, 10)
        self.levA = 0
        self.levB = 0
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.lastGpio = None
        self.pi = pigpio.pi()
        self.counter = 0
        self.name = name
        self.pi.set_mode(self.gpioA, pigpio.PUD_UP)
        self.pi.set_mode(self.gpioB, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(self.gpioB, pigpio.PUD_UP)
        self.callbackA = self.pi.callback(self.gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.callbackB = self.pi.callback(self.gpioB, pigpio.EITHER_EDGE, self._pulse)
        self.StartTime = time.time_ns()
        self.StopTime = time.time_ns()
        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.run)
        self.direction = direction
        
    def _pulse(self, gpio, level, tick):

      if gpio == self.gpioA:
         self.levA = level
      else:
         self.levB = level;

      if gpio != self.lastGpio: # debounce
         self.lastGpio = gpio

         if   gpio == self.gpioA and level == 1:
            if self.levB == 1:
               self.rotation_decode_1(1)
         elif gpio == self.gpioB and level == 1:
            if self.levA == 1:
               self.rotation_decode_1(-1)
        
    def rotation_decode_1(self, way):
        self.StopTime=time.time_ns()
        self.counter += way
        self.StartTime=time.time_ns()
        
    def run(self):
        msg = Float32()
        msg.data = float((self.counter/num_of_ticks_per_round *wheel)/(self.timer_period))
        msg.data = self.direction*msg.data
        self.publisher.publish(msg)
        self.counter = 0


def main(args=None):
    rclpy.init(args=args)
    
    left_node = EncoderDriver(Enc_A_1, Enc_B_1, 'left_encoder', -1)
    right_node = EncoderDriver(Enc_A_2, Enc_B_2, 'right_encoder', 1)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(left_node)
    executor.add_node(right_node)
    
    while(rclpy.ok()):
        executor.spin_once()
    left_node.destroy_node()
    right_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
