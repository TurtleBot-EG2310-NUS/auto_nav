import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import scipy.stats
from time import sleep
import numpy as np
import os
import math
import cmath
import time
import RPi.GPIO as GPIO 
from hx711 import HX711 #for the load cell


class FactoryTest(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.laser_range = np.array([])
        self.laser_valid = True

    def scan_callback(self, msg):
        # create numpy array
        self.laser_range = np.array(msg.ranges)

        self.laser_range[self.laser_range==0] = np.Inf
        numinfs = (self.laser_range==np.Inf).sum()
        if numinfs >= 270:
            self.laser_valid = False
        else:
            self.laser_valid = True

    def stopbot(self):
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def clear(self):
        os.system('clear')

    #Dynamixel test
    def dynamixeltest(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        done = False
        while not done:
            self.clear()
            print("Place the bot on the floor with sufficient space for it to move\nthen input Y and observe if the turtlebot moves approximately 10cm front, 10cm back\nand then 90 degrees left and 90 degrees right")
            if input() == 'y':
                #forward
                twist.linear.x = 0.2
                self.publisher_.publish(twist)
                time.sleep(0.5)
                self.stopbot()
                time.sleep(0.5)
                #backward
                twist.linear.x = -0.2
                self.publisher_.publish(twist)
                time.sleep(0.5)
                self.stopbot()
                time.sleep(0.5)
                #leftward
                twist.linear.x = 0.0
                twist.angular.z = 1.0
                self.publisher_.publish(twist)
                time.sleep(np.pi/2)
                self.stopbot()
                time.sleep(0.5)
                #rightward
                twist.linear.x = 0.0
                twist.angular.z = -1.0
                self.publisher_.publish(twist)
                time.sleep(np.pi/2)
                self.stopbot()
                time.sleep(0.5)
                done = True

    #Lidar test
    def lidartest(self):
        done = False
        while not done:
            self.clear()
            print("Ensure that the turtlebot is within 1.5 meters of a wall\nthen input Y to start the lidar test")
            if input() == 'y':
                done = True
        while (self.laser_range.size == 0):
                self.clear()
                print("Spin to get a valid lidar data")
                print(self.laser_range)
                rclpy.spin_once(self)
        if self.laser_valid:
            self.clear()
            print("laserscan data is VALID, lidar is functional")
            time.sleep(1)
        else:
            self.clear()
            print("laserscan data is INVALID, lidar is not functional")
            time.sleep(1)

    #Load cell test
    def weight_detected(self):
        # while True:
        hx711 = HX711(
            dout_pin=6,
            pd_sck_pin=5,
            channel='A',
            gain=64)

        hx711.reset()   # Before we start, reset the HX711 (not obligate)
        measures = hx711.get_raw_data()
        weight= sum(measures)/len(measures)
        return weight<50000 #Condition is true when weight placed
    
    def loadtest(self):
        done = False
        while not done:
            self.clear()
            print("Ensure that no payload is placed within the turtlebot carrier\nthen input Y to start the load cell test")
            if input() == 'y':
                done=True
        while not self.weight_detected:
            print('Waiting for payload')

        print('Payload detected')
        time.sleep(1)


    def test(self):
        try:
            self.dynamixeltest()
            self.lidartest()
            self.loadtest()
                
        except Exception as e:
            print(e)
        
        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)
    factorytest = FactoryTest()
    factorytest.test()
    factorytest.destroy_node()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
        #do some thermal nav node and shooting node here
    rclpy.shutdown()


if __name__ == '__main__':
    main()