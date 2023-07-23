#source code for the obstacle avoidance using laser scan
#Mia Hartley 25179600 <25179600@students.lincoln.ac.uk>

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        #subscrtiption to the scan topic
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        #publish twist messages to the cmd_vel topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel = Twist()

    def scan_callback(self, msg):
        min_distance = min(msg.ranges)
        #turtlebot rotates and reverses when they encounter an obstacle
        if min_distance > 0.5:
            self.cmd_vel.linear.x = 0.5
            self.cmd_vel.angular.z = 0.0
        else:
            self.cmd_vel.linear.x = -0.2
            self.cmd_vel.angular.z = 0.5

        self.publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
