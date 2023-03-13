import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

#import everything necessary


"""implement a behaviour that takes the robot forward if there is more than 
50cm space in front of it but make it turn to the left, if there isnt enough space."""

#refernce:
#https://github.com/LCAS/teaching/blob/lcas_humble/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/roamer.py

class Roamer(Node):
    # simple node that subscribes to scan and publishes twist msgs to cmd_vel

    min_distance = 0.5 #0.5m of space away from obstacles
    turn_speed = 0.2 #rad/s turn speed if obstacle is detected
    forward_speed = 0.2 #m/s, speed to go forward if no obstacle spotted
    scan_segment = 60 #degrees, the size of the left and right laser segment that is checked for obstacles
    

    def __init__(self):
        # initialise sub and pub for scan and cmd_vel
        super().__init__('roamer')

        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.callback, 1)
        #Creates subscription of data type LaserScan, to the /scan topic, with a queue size of 1
        # self.callback is run every time it receives a msg

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 1)
        #creates publisher of Twist msg to topic cmd_vel, queue of 1

    #finds smallest value in array, for checking laserscan data
    def min_range(self, range):
        min_range = math.inf
        for v in range:
            if v < min_range:
                min_range = v
        
        return min_range
    
    #callback function 
    def callback(self, data):
        #checks left and right 45 degree segments of laserscan
        min_right = self.min_range(data.ranges[:self.scan_segment])
        min_left = self.min_range(data.ranges[-self.scan_segment:])

        #create twist msg
        twist= Twist()

        #publish twist msg with data needed for current environment
        if min_right < self.min_distance:
            self.get_logger().info('turning left')
            twist.angular.z = -self.turn_speed
        
        elif min_left < self.min_distance:
            self.get_logger().info('turning right')
            twist.angular.z = self.turn_speed

        else:
            self.get_logger().info("going straight")
            twist.linear.x = self.forward_speed
        
        self.twist_pub.publish(twist)
    
def main(args=None):
    # Initilises ROS python subsytem
    try:
        rclpy.init()
        node = Roamer() #create node
        rclpy.spin(node) #node will keep running until stopped
    
    except KeyboardInterrupt:
        print("Interrupted by keyboard")
    
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
