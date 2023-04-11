import RPi.GPIO as GPIO 
from hx711 import HX711 # load cell
import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

DIVIDER = -200000
TOPIC_WEIGHT = "weight"
TOPIC_IR = "line"

current_weight_state = False
previous_weight_state = False

def check_weight():
    hx711 = HX711(
        dout_pin=6,
        pd_sck_pin=5,
        channel='A',
        gain=128
    )

    hx711.reset()   # Before we start, reset the HX711 (not obligate)
    measures = hx711.get_raw_data(times=5)
    weight= sum(measures)/len(measures)

    print(weight)

    if weight > DIVIDER:
        return False
    else:
        return True
    


# while True:
#     current_weight_state = check_weight()

    
#     if current_weight_state != previous_weight_state:
#         msg = "{ \"data\": " + "\"" + str(current_weight_state).lower() + "\""  + " }"
#         print(msg)
#         # client.publish(WEIGHT_TOPIC, msg, 0)

#     time.sleep(1)
    

#     previous_weight_state = current_weight_state



class WeightPublisher(Node):

    def __init__(self):
        super().__init__('weight_publisher')
        self.publisher_ = self.create_publisher(String, TOPIC_WEIGHT, 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.weight_callback)
        # self.i = 0

    def weight_callback(self):
        msg = String()
        global current_weight_state
        global previous_weight_state

        current_weight_state = check_weight()

        if current_weight_state != previous_weight_state:
            msg.data = "%s" % str(current_weight_state).lower() 
            # print(msg)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

                
        # msg.data = "%s" % str(current_weight_state).lower() 
        # # print(msg)
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)

        previous_weight_state = current_weight_state


            

def main(args=None):
    rclpy.init(args=args)

    weight_publisher = WeightPublisher()

    rclpy.spin(weight_publisher)


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    weight_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()