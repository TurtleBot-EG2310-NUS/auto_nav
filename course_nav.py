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
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String #Placeholder for snap-action switch
import numpy as np
import math
import cmath
import time
import pandas as pd

# constants
rotatechange = 0.1
speedchange = -0.05
occ_bins = [-1, 0, 100, 101]
distance_to_stop = 0.6
front_angles = range(-2,3,1)
back_angles = range(178,183,1)
placeholder_value= 5
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
            qos_profile_sensor_data)
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
            qos_profile= qos_policy)
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


    def tablenumber_callback(self, msg):
        self.chosen_table= msg.data
        print('Table number received')
        print("Table number received is:", self.chosen_table)

    def weight_callback(self, msg):
        #print("called")
        self.weight_placed= msg.data=='true'
        #print(self.weight_placed)

    def docking_callback(self, msg):
        self.docked_yet= (msg.data=='true')
        # print(self.docked_yet)
    ##################################################################

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orientation_quat =  msg.pose.pose.orientation
        self.roll, self.pitch, self.yaw = euler_from_quaternion(orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w)


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
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with 100
        self.laser_range[self.laser_range==0] = placeholder_value #np.nan
    

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


    def move_front(self, stop_distance):

        if self.laser_range.size != 0:
            front_dist_range= self.laser_range[front_angles]
            front_dist_range[front_dist_range==placeholder_value] = min(front_dist_range)
            front_dist= sum(front_dist_range)/len(front_dist_range)
        
            while front_dist>stop_distance:
                # allow the callback functions to run
                rclpy.spin_once(self)

                front_dist_range= self.laser_range[front_angles]
                front_dist_range[front_dist_range==placeholder_value] = min(front_dist_range)
                front_dist= sum(front_dist_range)/len(front_dist_range)
                print('Front distance: '+str(front_dist))
                self.speed(speedchange)
            
            self.stopbot()  

    def move_back(self, stop_distance):

        if self.laser_range.size != 0:
            
            back_dist_range= self.laser_range[back_angles]
            back_dist_range[back_dist_range==placeholder_value] = min(back_dist_range)
            back_dist= sum(back_dist_range)/len(back_dist_range)
            while back_dist>stop_distance:
                # allow the callback functions to run
                rclpy.spin_once(self)
                back_dist_range= self.laser_range[back_angles]
                back_dist_range[back_dist_range==placeholder_value] = min(back_dist_range)
                back_dist= sum(back_dist_range)/len(back_dist_range)
                print('Back distance: '+str(back_dist))
                self.speed(-speedchange)
            
            self.stopbot()  

    def waiting_for_pickup(self):
        while self.weight_placed:
            print('Please pick the can up')
        print('Can has been removed')
        time.sleep(2) #This allows for delay between can being removed and robot moving off, for safety

    def waiting_for_dropoff(self):
        while not self.weight_placed:
            print('Waiting for can')
            #print(self.weight_placed)
        print('Can has been dropped')




    ####These functions are just for testiing. Can remove later###
    # def check_back_dist(self):
    #     if self.laser_range.size != 0:
    #         back_dist_range= self.laser_range[back_angles]
    #         back_dist_range[back_dist_range==placeholder_value] = min(back_dist_range)
    #         back_dist= sum(back_dist_range)/len(back_dist_range)
    #         while True:
    #             rclpy.spin_once(self)
    #             back_dist_range= self.laser_range[back_angles]
    #             back_dist_range[back_dist_range==placeholder_value] = min(back_dist_range)
    #             back_dist= sum(back_dist_range)/len(back_dist_range)
    #             print('Back distance: '+str(back_dist))
    #             print('Back distance: '+str(back_dist_range))

    # def check_front_dist(self):
    #     if self.laser_range.size != 0:
    #         front_dist_range= self.laser_range[front_angles]
    #         front_dist_range[front_dist_range==placeholder_value] = min(front_dist_range)
    #         front_dist= sum(front_dist_range)/len(front_dist_range)
    #         while True:
    #             rclpy.spin_once(self)
    #             front_dist_range= self.laser_range[front_angles]
    #             front_dist_range[front_dist_range==placeholder_value] = min(front_dist_range)
    #             front_dist= sum(front_dist_range)/len(front_dist_range)
    #             print('Front distance: '+str(front_dist))

    # def testing(self):
    #     self.speed(speedchange)
    #     print('Moving')
    #     time.sleep(5)
    #     self.stopbot()
    #     print('Reached')
    #     self.chosen_table=0
    
    ############################################################


    ############################################################
    ######## Table orders (Delivery to specific tables) ########
    ############################################################

    def table1(self):
        print('table 1')
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.move_back(distance_to_stop)
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(distance_to_stop)
            self.docking()

    
    def table2(self):
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.move_back(distance_to_stop)
            self.rotatebot(90, right)
            self.move_back(1.3)
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(0.6)
            self.rotatebot(90, left)
            self.move_front(distance_to_stop)
            self.docking()

    
    def table3(self):
        self.table2()


    def table4(self):
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.move_back(distance_to_stop)
            self.rotatebot(90, right)
            self.move_back(0.5)
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(0.6)
            self.rotatebot(90, left)
            self.move_front(distance_to_stop)
            self.docking()

    
    def table5(self):
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.move_back(1.75)
            self.rotatebot(90, right)
            self.move_back(0.5)
            self.rotatebot(90, left)
            self.move_front(0.70)
            self.rotatebot(90, right)
            self.move_back(0.55)
            self.rotatebot(90, left)
            self.move_back(distance_to_stop)
            self.waiting_for_pickup()
            #Return back to docking station
            self.move_front(0.6)
            self.rotatebot(90, right)
            self.move_front(2.2)
            self.rotatebot(90, left)
            self.move_back(distance_to_stop)
            self.rotatebot(90, right)
            self.move_front(distance_to_stop)
            self.rotatebot(90, left)
            self.move_front(distance_to_stop)
            self.docking()

    def table6(self):
        if self.laser_range.size != 0:  #This line to ensure move_front will run first, before rotatebot as scan message has delay to be received
            self.move_back(distance_to_stop)
            self.rotatebot(90,right)
            self.move_back(0.6)
            self.rotatebot(90,left)
            self.move_back(0.9)
            self.rotatebot(90,left)

            #### Code to find the table ###
            if min(self.laser_range[back_angles])<2.0: #Stops in the event table is detected early, doesnt stop in middle of box
                self.move_back(distance_to_stop)
                self.waiting_for_pickup()
                #Return back to docking station
                self.move_front(distance_to_stop)

            else:   # Else moves to middle of the square bounds
                self.move_back(0.9)
                self.waiting_for_pickup()
                #Return back to docking station
                self.move_front(distance_to_stop)
            #######################################

            self.rotatebot(90,right)
            self.move_front(distance_to_stop)
            self.rotatebot(90, right)
            self.move_front(distance_to_stop)
            self.rotatebot(90, left)
            self.move_front(distance_to_stop)
            self.docking()

    def docking(self):
        while self.docked_yet!=True:
            print('Preparing to dock')
            self.speed(speedchange)
        #Stop robot once docked
        self.speed(0.0)
        self.chosen_table=0
        print('Robot has docked')


    def select_table(self):
        table_number_dict= {'1':self.table1, '2':self.table2, '3':self.table3, \
                            '4':self.table4, '5':self.table5, '6':self.table6}
        relevant_nav_function= table_number_dict.get(self.chosen_table)
        self.waiting_for_dropoff()
        relevant_nav_function()




    ###################################################################
    ######## Mover functions (Function that is called in main) ########
    ###################################################################
    def mover(self):
        try:
            self.stopbot()

            while rclpy.ok():
                rclpy.spin_once(self) # allow the callback functions to run
                #print('Waiting for next input')
                # if self.chosen_table!=0:
                #     self.select_table()
                self.waiting_for_dropoff()
                    
        except Exception as e:
            print(e)

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