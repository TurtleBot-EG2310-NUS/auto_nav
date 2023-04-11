import RPi.GPIO as GPIO 
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

TOPIC_IR = "line"


ir_pin_one = 20
ir_pin_two = 21

GPIO.setup(ir_pin_one, GPIO.IN)
GPIO.setup(ir_pin_two, GPIO.IN)

ir_one_state = False
ir_two_state = False



class IRPublisher(Node):

    def __init__(self):
        super().__init__('ir_publisher')
        self.publisher_ = self.create_publisher(String, TOPIC_IR, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.ir_callback)
        # self.i = 0

    def ir_callback(self):
        msg = String()

        ir_one_state = GPIO.input(ir_pin_one)
        ir_two_state = GPIO.input(ir_pin_two)
        # print(ir_pin_one)
        # print(ir_pin_two)

        if ir_one_state and ir_two_state:
            msg.data = "b"
            # print(msg)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        elif ir_one_state:
            msg.data = "r"
            # print(msg)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        elif ir_two_state:
            msg.data = "l"
            # print(msg)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
        else:
            msg.data = "n"
            # print(msg)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

            

def main(args=None):
    rclpy.init(args=args)

    ir_publisher = IRPublisher()

    rclpy.spin(ir_publisher)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ir_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()