import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from math import sqrt

class MoveDistance(Node):

    def __init__(self, distance):
        super().__init__('move_distance')
        self.distance = distance
        self.current_distance = 0.0
        self.old_distance = 0.0
        self.speed = 0.1
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.odom_subscription  # prevent unused variable warning

    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        linear_distance = sqrt((current_position.x - self.old_position.x) ** 2 + (current_position.y - self.old_position.y) ** 2)
        self.old_distance += linear_distance
        self.current_distance = self.old_distance
        self.old_position = current_position

    def run(self):
        self.old_position = self.current_position #gets current position
        linear_distance= 0.0

        while rclpy.ok() and self.current_distance < self.distance:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = self.speed
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.get_logger().info('Moving...')
            rclpy.spin_once(self)

        cmd_vel_msg = Twist()
        self.cmd_vel_publisher.publish(cmd_vel_msg)
        self.get_logger().info('Finished moving.')

def main(args=None):
    rclpy.init(args=args)
    move_distance = MoveDistance(distance=0.5)
    move_distance.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
