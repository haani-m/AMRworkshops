#!/usr/bin/env python

# An example of TurtleBot 3 subscribe to camera topic, mask colours, find and display contours, and move robot to center the object in image frame
# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
import math

#To do:
#Implement roamer path for it to take regardless of objects of interest
#Implement better masking for points of interest
#Implement callback function using laser data to stop when close to object of interest
#Do this using either distance of objects in front or using circle drawn on objects of interest





class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')
        
        self.turn_vel = 0.0
        self.lin_vel = 0.0

        # publish cmd_vel topic to move the robot
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # create timer to publish cmd_vel topic
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # subscribe to the camera topic
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        #subscribe to laser scan
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 1)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        # Create mask for range of colours (HSV low values, HSV high values)
        #current_frame_mask = cv2.inRange(current_frame_hsv,(70, 0, 50), (150, 255, 255))
        orange_frame_mask = cv2.inRange(current_frame_hsv,(0, 150, 50), (255, 255, 255)) # orange
        lower_red_mask = cv2.inRange(current_frame_hsv,(0, 100, 20), (160, 100, 20)) # red lower mask
        upper_red_mask = cv2.inRange(current_frame_hsv,(10, 255, 255), (179, 255, 255)) # red upper mask
        current_frame_mask = orange_frame_mask

        contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Sort by area (keep only the biggest one)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, contours, 0, (0, 255, 0), 20)
        
        if len(contours) > 0:
            # find the centre of the contour: https://docs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
            M = cv2.moments(contours[0]) # only select the largest controur
            if M['m00'] > 0:
                # find the centroid of the contour
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                #print("Centroid of the biggest area: ({}, {})".format(cx, cy))

                # Draw a circle centered at centroid coordinates
                # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
                cv2.circle(current_frame, (round(cx), round(cy)), 50, (0, 255, 0), -1)
                            
                # find height/width of robot camera image from ros2 topic echo /camera/image_raw height: 1080 width: 1920

                # if center of object is to the left of image center move left
                if cx < 900:
                    self.turn_vel = 0.3
                    self.lin_vel = 0.0
                # else if center of object is to the right of image center move right
                elif cx >= 1200:
                    self.turn_vel = -0.3
                    self.lin_vel = 0.0
                else: # center of object is in a 100 px range in the center of the image so dont turn
                    #print("object in the center of image")
                    self.turn_vel = 0.0
                    self.lin_vel = 0.2
        else:
            print("No Centroid Found")
            # turn until we can see a coloured object
            self.turn_vel = 0.3

        # show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)

    def timer_callback(self):
        #print('entered timer_callback')

        self.tw=Twist() # twist message to publish

        
        self.tw.linear.x = self.lin_vel
        self.tw.angular.z = self.turn_vel

        self.pub_cmd_vel.publish(self.tw)
    
    #finds smallest value in array, for checking laserscan data
    def min_range(self, range):
        min_range = math.inf
        for v in range:
            print(v)
            if v < min_range:
                min_range = v
        
        return min_range

    def laser_callback(self, data):
        min_distance = 0.5
        min_centre = self.min_range(data.ranges[-15:15])

        if min_centre < min_distance:
            self.lin_vel = 0
            print("Less than "+ min_centre +"m away from object in front")
        else:
            print(min_centre ,"m away from object in front")

#look up how to find distance from centroid


def main(args=None):
    print('Starting colour_chaser.py.')

    rclpy.init(args=args)

    colour_chaser = ColourChaser()

    rclpy.spin(colour_chaser)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    colour_chaser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()