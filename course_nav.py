import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')

        #Create subscriber to track odometry, aka position
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback, 
            qos_profile_sensor_data)
        self.odom_subscription #prevent unused variable

        #Create subscriber to track Lidar
        self.scan_subscription= self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription #prevent unused variable warning


    def odom_callback(self, msg):
        self.odomtest=1.5

    def scan_callback(self, msg):
        self.scantest=2


    def move(self):
        try:
            rclpy.spin_once(self)

            while rclpy.ok():
                print(self.scantest)
        except Exception as e:
            print(e)



def main(args=None):
    rclpy.init(args=args)
    auto_nav= AutoNav()
    auto_nav.move()
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()





# import rclpy
# from rclpy.node import Node
# from rclpy.qos import qos_profile_sensor_data
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist
# import numpy as np
# import math
# import cmath


# # code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
# def euler_from_quaternion(x, y, z, w):
#     """
#     Convert a quaternion into euler angles (roll, pitch, yaw)
#     roll is rotation around x in radians (counterclockwise)
#     pitch is rotation around y in radians (counterclockwise)
#     yaw is rotation around z in radians (counterclockwise)
#     """
#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + y * y)
#     roll_x = math.atan2(t0, t1)

#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     pitch_y = math.asin(t2)

#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (y * y + z * z)
#     yaw_z = math.atan2(t3, t4)

#     return roll_x, pitch_y, yaw_z  # in radians





# class AutoNav(Node):

#     def __init__(self):
#         super().__init__('auto_nav')

#         #Create publisher for moving turtlebot
#         self.vel_publisher= self.create_publisher(Twist,'cmd_vel',10)

#         #Create subscriber to track odometry, aka position
#         self.odom_subscription = self.create_subscription(
#             Odometry,
#             'odom',
#             self.odom_callback, 
#             qos_profile_sensor_data)
#         #self.odom_subscription #prevent unused variable

#         #Create subscriber to track Lidar
#         self.scan_subscription= self.create_subscription(
#             LaserScan,
#             'scan',
#             self.scan_callback,
#             qos_profile_sensor_data)
#         #self.scan_subscription #prevent unused variable warning




#     def odom_callback(self, msg):
#         self.odomtest=1.5
#         # orientation_quat= msg.pose.pose.orientation
#         # position= msg.pose.pose.position
#         # #Yaw goes from 0->-3.14->3,14->0
#         # self.roll, self.pitch, self.yaw= euler_from_quaternion(orientation_quat.x,orientation_quat.y,orientation_quat.z,orientation_quat.w)
#         # self.yaw+=math.pi #convert from '-180 to 180', to become '0 to 360'
#         # self.locx, self.locy= position.x, position.y


#     def scan_callback(self, msg):
#         self.scantest=2
#         # laser_range= np.array(msg.ranges[:100])
#         # laser_range[laser_range==0] =np.nan #Replace NaN values with 0
#         # self.front= laser_range[0] #Takes front of Lidar, aka 0 degree  
#         # #print(laser_range[0], laser_range[90], laser_range[180], laser_range[270])
#         # #self.get_logger().info('Front distance is %s' % self.front)


    
#     def rotate_left(self):
#         twist= Twist() #create Twist object
#         current_yaw= self.yaw #get current yaw
#         target_yaw = current_yaw + math.radians(90) #calculate desired yaw
#         if target_yaw>2*math.pi:
#             target_yaw -= 2*math.pi

#         while not target_yaw-0.02<=self.yaw<=target_yaw+0.02:
#             rclpy.spin_once(self) #allows callbacks to be called once
#             twist.linear.x= 0.0 
#             twist.angular.z= 0.3
#             self.vel_publisher.publish(twist)
#             #print(self.yaw, target_yaw)

#         twist.angular.z= 0.0 #stops rotation
#         self.vel_publisher.publish(twist)


#     def rotate_right(self):
#         twist= Twist() #create Twist object
#         current_yaw= self.yaw #get current yaw
#         target_yaw = current_yaw - math.radians(90) #calculate desired yaw
#         if target_yaw>2*math.pi:
#             target_yaw -= 2*math.pi

#         while not target_yaw-0.01<=self.yaw<=target_yaw+0.01:
#             rclpy.spin_once(self) #allows callbacks to be called once
#             twist.linear.x= 0.0 
#             twist.angular.z= -0.3
#             self.vel_publisher.publish(twist)
#             #print(self.yaw, target_yaw)

#         twist.angular.z= 0.0 #stops rotation
#         self.vel_publisher.publish(twist)


#     def rotate_back(self):
#         twist= Twist() #create Twist object
#         current_yaw= self.yaw #get current yaw
#         target_yaw = current_yaw + math.radians(180) #calculate desired yaw
#         if target_yaw>2*math.pi:
#             target_yaw -= 2*math.pi

#         while not target_yaw-0.01<=self.yaw<=target_yaw+0.01:
#             rclpy.spin_once(self) #allows callbacks to be called once
#             twist.linear.x= 0.0 
#             twist.angular.z= 0.3
#             self.vel_publisher.publish(twist)
#             #print(self.yaw, target_yaw)

#         twist.angular.z= 0.0 #stops rotation
#         self.vel_publisher.publish(twist)
#         rclpy.shutdown()

    
#     def forward(self, distance):
#         pass
#         # twist= Twist()
#         # while self.front>distance:
#         #     twist.linear.x=0.2
#         #     self.vel_publisher.publish(twist)

#         # twist.linear.x=0.0
#         # self.vel_publisher.publish(twist)

#     def backward(self):
#         twist= Twist()
#         twist.linear.x=-0.2
#         self.vel_publisher.publish(twist)

#     def table_1(self):
#         self.forward(0.1)
        
#         # served=input('Can received?: True/False')
#         # if served=='True':
#         #     self.rotate_back()
#         #     self.forward(0.1)


#     def move(self):
#         try:
#             rclpy.spin_once(self)

#             while rclpy.ok():
#                 #print(self.callback(self.odom_sub, self.scan_sub))
#                 print(self.odomtest)
#                 #self.callback()
#         except Exception as e:
#             print(e)


        


# def main(args=None):
#     rclpy.init(args=args)
#     auto_nav= AutoNav()
#     auto_nav.move()
#     auto_nav.destroy_node()
#     rclpy.shutdown()


# if __name__=='__main__':
#     main()