# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String #Placeholder for snap-action switch
import numpy as np
import math
import cmath
import time
import pandas as pd
from math import sqrt
from std_srvs.srv import Empty
from geometry_msgs.msg import Point, Quaternion, Twist

# constants
rotatechange = 0.3
speedchange = -0.13
occ_bins = [-1, 0, 100, 101]
distance_to_stop = 0.20
distance_to_stop_dock= 0.6
front_angles = range(-5,6,1)
back_angles = range(175,186,1)
left= 1
right= -1
initial_weight_value= 0

scanfile = 'lidar.txt'
mapfile = 'map.txt'
qos_policy = rclpy.qos.QoSProfile(reliability = rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history = rclpy.qos.HistoryPolicy.KEEP_LAST, depth = 10)

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians

class AutoNav(Node):

    ##############################################################################
    ########Basic initialisastion (Initialisation and callback functions) ########
    ##############################################################################
    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            qos_policy)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # create subscription to track occupancy
        self.occ_subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.occ_callback,
            qos_profile_sensor_data)
        self.occ_subscription  # prevent unused variable warning
        self.occdata = np.array([])
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile= qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])
        self.lri= 5

    
    ######## Created subscribers and callbacks(Table number & docking) ############
        self.tablenumber_subscription= self.create_subscription(
            String,
            'table',
            self.tablenumber_callback,
            qos_profile_sensor_data)
        self.tablenumber_subscription # prevent unused variable warning
        self.chosen_table=0

        self.docking_subscription= self.create_subscription(
            String,
            'dock',
            self.docking_callback,
            qos_profile_sensor_data)
        self.docking_subscription # prevent unused variable warning
        self.docked_yet= False

        self.weight_subscription= self.create_subscription(
            String,
            'weight',
            self.weight_callback,
            qos_profile_sensor_data)
        self.weight_subscription # prevent unused variable warning
        self.weight_placed= False

        self.line_subscription= self.create_subscription(
            String,
            'line',
            self.line_callback,
            qos_profile_sensor_data)
        self.line_subscription # prevent unused variable warning
        self.line_direction= 'a'

        self.table_6_segment= 0

    def tablenumber_callback(self, msg):
        self.chosen_table= msg.data
        # self.get_logger().info('Table number received')
        self.get_logger().info("Table number received is: %s" % self.chosen_table)

    def weight_callback(self, msg):
        # print("called")
        self.weight_placed= msg.data=='true'
        # print(self.weight_placed)
        # print(type(self.weight_placed))

    def docking_callback(self, msg):
        self.docked_yet= (msg.data=='true')
        # print(self.docked_yet)
    
    def line_callback(self, msg):
        self.line_direction= msg.data

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)
        self.current_position= msg.pose.pose.position

    def occ_callback(self, msg):
        # self.get_logger().info('In occ_callback')
        # create numpy array
        msgdata = np.array(msg.data)
        # compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msgdata,occ_bins)
        # calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # log the info
        # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins))

        # make msgdata go from 0 instead of -1, reshape into 2D
        oc2 = msgdata + 1
        # reshape to 2D array using column order
        # self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width,order='F'))
        self.occdata = np.uint8(oc2.reshape(msg.info.height,msg.info.width))
        # print to file
        np.savetxt(mapfile, self.occdata)


    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with 100
        self.laser_range[self.laser_range==0] = np.nan
    

    ####################################################
    ########Basic functions (Move, stop, rotate)########
    ####################################################

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle, turn_direction):
            # self.get_logger().info('In rotatebot')
            # create Twist object
            twist = Twist()
            
            # get current yaw angle
            current_yaw = self.yaw
            # log the info
            self.get_logger().info('Current: %f' % math.degrees(current_yaw))
            # we are going to use complex numbers to avoid problems when the angles go from
            # 360 to 0, or from -180 to 180
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # calculate desired yaw
            target_yaw = current_yaw + math.radians(rot_angle)
            # convert to complex notation
            c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
            self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
            # divide the two complex numbers to get the change in direction
            c_change = c_target_yaw / c_yaw
            # get the sign of the imaginary component to figure out which way we have to turn
            c_change_dir = np.sign(c_change.imag)
            # set linear speed to zero so the TurtleBot rotates on the spot
            twist.linear.x = 0.0
            # set the direction to rotate
            twist.angular.z = c_change_dir * rotatechange * turn_direction
            # start rotation
            self.publisher_.publish(twist)

            # we will use the c_dir_diff variable to see if we can stop rotating
            c_dir_diff = c_change_dir
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
            # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
            # becomes -1.0, and vice versa
            while(c_change_dir * c_dir_diff > 0):
                # allow the callback functions to run
                rclpy.spin_once(self)
                current_yaw = self.yaw
                # convert the current yaw to complex form
                c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
                # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
                # get difference in angle between current and target
                c_change = c_target_yaw / c_yaw
                # get the sign to see if we can stop
                c_dir_diff = np.sign(c_change.imag)
                # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

            self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
            # set the rotation speed to 0
            twist.angular.z = 0.0
            # stop the rotation
            self.publisher_.publish(twist)


    def stopbot(self):
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)
        #self.get_logger().info('Robot has reached destination')

    def speed(self, move_speed):
        # start moving
        twist = Twist()
        twist.linear.x = move_speed
        twist.angular.z = 0.0
        time.sleep(1)
        self.publisher_.publish(twist)


    def move_front(self, dist_limit):
        self.speed(speedchange)
        while True:
            rclpy.spin_once(self)
            # self.get_logger().info('Working')
            while self.laser_range.size==0 or math.isnan(self.laser_range[0]):   
                rclpy.spin_once(self)
                front_distance= self.laser_range[0]   
            front_distance= self.laser_range[0] - 0.10#Lidar offset due to front carrier
            self.get_logger().info('Distance is %f ' %front_distance)

            if front_distance<dist_limit:
                self.stopbot()
                break

    def move_back(self, dist_limit):
        self.speed(-speedchange)

        while True:
            rclpy.spin_once(self)
            # self.get_logger().info('Working')
            while self.laser_range.size==0 or math.isnan(self.laser_range[180]):   
                rclpy.spin_once(self)
                back_distance= self.laser_range[180]   
            back_distance= self.laser_range[180] 
            self.get_logger().info('Distance is %f ' % back_distance)

            if back_distance<dist_limit:
                self.stopbot()
                break

    def waiting_for_pickup(self):
        # time.sleep(2)
        self.get_logger().info('Please pick the can up')
        while self.weight_placed:
            rclpy.spin_once(self)
            # time.sleep(0.5)
        self.get_logger().info('Can has been removed')
        time.sleep(2) #This allows for delay between can being removed and robot moving off, for safety

    def waiting_for_dropoff(self):
        # time.sleep(2)
        self.get_logger().info("Waiting for can to drop")
        while not self.weight_placed:
            rclpy.spin_once(self)
            # time.sleep(0.5)
        self.get_logger().info('Can has been dropped')
        time.sleep(0.5)


    ############################################################
    ######## Table orders (Delivery to specific tables) ########
    ############################################################

    def table1(self):
        if self.laser_range.size != 0:  #This line to ensure movef_front will run first, before rotatebot as scan message has delay to be received
            self.waiting_for_dropoff()
            self.move_back(distance_to_stop)
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(distance_to_stop_dock - 0.1)
            self.docking()

    
    def table2(self):
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.waiting_for_dropoff()
            self.move_back(distance_to_stop) #distance from bucket 1
            self.rotatebot(90, right)
            self.move_back(1.15) #distance from left wall
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(0.50) 
            self.rotatebot(90, left)
            self.move_front(distance_to_stop_dock)
            self.docking()

    
    def table3(self):
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.waiting_for_dropoff()
            self.move_back(0.5)
            self.rotatebot(90, right)
            self.move_back(1.15)
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(0.50)
            self.rotatebot(90, left)
            self.move_front(distance_to_stop_dock)
            self.docking()



    def table4(self):
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.waiting_for_dropoff()
            self.move_back(0.55)
            self.rotatebot(90, right)
            self.move_back(0.42)
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(0.55)
            self.rotatebot(90, left)
            self.move_front(distance_to_stop_dock)
            self.docking()

    
    def table5(self):
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.waiting_for_dropoff()
            self.move_back(1.50)
            self.rotatebot(90, right)
            self.move_back(0.45)
            self.rotatebot(90, left)
            self.move_back(distance_to_stop)
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(0.5)
            self.rotatebot(90, right)
            self.move_front(0.5)
            self.rotatebot(90, left)
            self.docking()

    def table6(self):

        def front():
            rclpy.spin_once(self)
            while self.laser_range.size==0 or math.isnan(self.laser_range[0]):   
                rclpy.spin_once(self)
                front_distance= self.laser_range[0]   
            front_distance= self.laser_range[0] - 0.10#Lidar offset due to front carrier
            return front_distance
        

        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.waiting_for_dropoff()
            self.move_back(0.25)
            self.rotatebot(90, right)
            self.move_back(0.42)
            self.rotatebot(90, left)


            if self.table_6_segment==0:
            #### Code to find the table at the start###
                self.move_back(1.2)
                self.rotatebot(90,right)
                print(front())
                #Check all 3 segments
                if front()<1.7:
                    self.table_6_segment=1
                    self.move_front(distance_to_stop)
                    self.waiting_for_pickup()
                else:
                    self.rotatebot(90,left)
                    self.move_back(0.9)
                    self.rotatebot(90,right)

                    if front()<1.7:
                        self.table_6_segment=2
                        self.move_front(distance_to_stop)
                        self.waiting_for_pickup()

                    else:
                        self.table_6_segment=3
                        self.rotatebot(90,left)
                        self.move_back(0.6)
                        self.rotatebot(90,right)
                        self.move_front(distance_to_stop)
                        self.waiting_for_pickup()
            #############################################

            elif self.table_6_segment==1:
                self.move_back(1.2)
                self.rotatebot(90,right)
                self.move_front(distance_to_stop)
                self.waiting_for_pickup()

            elif self.table_6_segment==2:
                self.move_back(0.9)
                self.rotatebot(90,right)
                self.move_front(distance_to_stop)
                self.waiting_for_pickup()

            elif self.table_6_segment==3:
                self.move_back(0.6)
                self.rotatebot(90,right)
                self.move_front(distance_to_stop)
                self.waiting_for_pickup()

            #Segment to run after can has been picked up
            self.move_back(0.42)
            self.rotatebot(90, left)
            self.move_front(0.3)
            self.rotatebot(90, right)
            self.move_front(0.5)
            self.rotatebot(90, left)
            self.move_front(distance_to_stop_dock)
            self.docking()


    def docking(self):
        twist= Twist()
        flag = False
        old_line_dir = 'n'
        rclpy.spin_once(self)
        rclpy.spin_once(self)

        while self.docked_yet!=True:
            self.get_logger().info(self.line_direction)
            rclpy.spin_once(self)
            if self.line_direction=='b' and old_line_dir != 'b':
                flag = True
                old_line_dir = 'b'
                self.get_logger().info('Dock: Moving forward')
                self.speed(-0.05)
                
                #self.speed(0.0)
            elif self.line_direction=='r' and old_line_dir != 'r':
                flag = True
                old_line_dir = 'r'
                self.get_logger().info('Dock: Turning right')
                twist.linear.x = -0.02
                twist.angular.z = -0.04
                self.publisher_.publish(twist)
            elif self.line_direction=='l' and old_line_dir != 'l':
                flag = True
                old_line_dir = 'l'
                self.get_logger().info('Dock: Turning left')
                twist.linear.x = -0.02
                twist.angular.z = 0.04
                self.publisher_.publish(twist)
            else:
                if flag == False:
                    self.speed(0.0)
                    self.rotatebot(90, right)
                    self.move_front(0.3)
                    self.rotatebot(115, left)
                    while self.line_direction=='n':
                        self.get_logger().info('Moving to docking position')
                        rclpy.spin_once(self)
                        self.speed(-0.02)
                        self.get_logger().info(self.line_direction)
                    flag = True
                    self.get_logger().info('Exited')
                # else: 
                #     self.get_logger().info("Adjusting course")
                #     twist.linear.x = -0.01
                #     twist.angular.z = -0.02
                #     self.publisher_.publish(twist)

                
        #Stop robot once docked
        self.speed(0.0)
        self.chosen_table=0
        self.get_logger().info('Robot has docked')
        self.get_logger().info('Waiting for next input')


    def select_table(self):
        table_number_dict= {'1':self.table1, '2':self.table2, '3':self.table3, \
                            '4':self.table4, '5':self.table5, '6':self.table6}
        relevant_nav_function= table_number_dict.get(self.chosen_table)
        self.waiting_for_dropoff()
        relevant_nav_function()

    def testing(self):
        self.move_back(0.2, 0.5)
        self.get_logger().info('next')
        self.move_back(0.2, 0.7)



    ###################################################################
    ######## Mover functions (Function that is called in main) ########
    ###################################################################
    def mover(self):
        try:
            self.stopbot()
            self.get_logger().info('Waiting for next input')

            while rclpy.ok():
                rclpy.spin_once(self) # allow the callback functions to run
                # self.docking()
                if self.chosen_table!=0:
                    self.get_logger().info('Table gotten')
                    self.select_table()
                    
        except Exception as e:
            self.get_logger().info(e)

        finally:
            self.stopbot()



def main(args=None):
    rclpy.init(args=args)
    auto_nav = AutoNav()
    auto_nav.mover()

    auto_nav.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()