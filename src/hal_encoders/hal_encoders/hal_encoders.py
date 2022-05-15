import RPi.GPIO as GPIO
from time import sleep
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class EncoderDriver(Node):
    def __init__(self):
        super().__init__('hal_encoder_driver')
        self.publisher = self.create_publisher(Int8, 'left_motor_speed', 10)
        GPIO.setmode(GPIO.BCM)
        self.Enc_A_1 = 4
        self.Enc_B_1 = 1
        self.Enc_A_2 = 20
        self.Enc_B_2 = 21
        GPIO.setup(self.Enc_A_1, GPIO.IN)
        GPIO.setup(self.Enc_B_1, GPIO.IN)
        GPIO.setup(self.Enc_A_2, GPIO.IN)
        GPIO.setup(self.Enc_B_2, GPIO.IN)
        self.counter_1 = 0
        self.counter_2 = 0
        GPIO.add_event_detect(self.Enc_A_1, GPIO.RISING, callback=rotation_decode_1, bouncetime=5)
        GPIO.add_event_detect(self.Enc_A_2, GPIO.RISING, callback=rotation_decode_2, bouncetime=5)
        
    def rotation_decode_1(self, Enc_A):
        self.counter_1 +=1
        
    def totation_decode_2(self, Enc_A):
        self.counter_2 +=1
        
        
    

def rotation_decode(Enc_A):
    global counter
    sleep(0.002)
    Switch_A = GPIO.input(Enc_A)
    Switch_B = GPIO.input(Enc_B)

    if (Switch_A == 1) and (Switch_B == 0):
        counter += 1
        print("direction -> ", counter)
        while Switch_B == 0:
            Switch_B = GPIO.input(Enc_B)
        while Switch_B == 1:
            Switch_B = GPIO.input(Enc_B)
        return

    elif (Switch_A == 1) and (Switch_B == 1):
        counter -= 1
        print("direction <- ", counter)
        while Switch_A == 1:
            Switch_A = GPIO.input(Enc_A)
        return
    else:
        return


def main(args=None):
    rclpy.init(args=args)
    
    node = EncoderDriver()
    while(rclpy.ok()):
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
