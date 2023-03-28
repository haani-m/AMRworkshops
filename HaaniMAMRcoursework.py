'''
Haani Mahmood 250270480
Autonomous Mobile Robotics Coursework
References:

https://github.com/LCAS/teaching/blob/lcas_humble/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/colour_chaser2.py
-Used as the basis for the camera_callback function

https://github.com/LCAS/teaching/blob/lcas_humble/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/roamer.py
-Adapted roaming fuctionality into laserscan_callback function
-Provided min_range function'''

#imports:
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from cv_bridge import CvBridge 
import cv2
import numpy as np
import math


class ObjectSearcher(Node):
    def __init__(self):
        super().__init__('object_searcher')

        #HSV masks for the different coloured objects
        self.red_mask1 = [(160, 255, 50), (180, 255, 255)]
        self.red_mask2 = [(0, 255, 50), (10, 255, 255)] #works for column

        self.green_mask = [(50,100,100),(70,255,255)]
        self.yellow_mask = [(22, 93, 100), (45, 255, 255)]
        self.blue_mask = [(110, 50, 50), (130, 255, 255)]

        self.mask_list = [self.yellow_mask, self.red_mask1, self.red_mask2, self.green_mask,  self.blue_mask]
        
        self.index_of_interest = int

        #flags to determine behaviour
        self.colour_flag = False 
        self.timer_flag = False
        self.start_flag = True
     
        

        self.turn_vel = 0.0
        self.lin_vel = 0.0
        self.min_distance = 0.7 #min distance used for roamer function
        self.object_max_dist = 1.0 # max distance robot can be from object

        
        #publisher and subscribers
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        
        timer_period= 10
        self.timer = self.create_timer(timer_period, self.timer_callback) 
        

        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, 1)

        self.pub_string = self.create_publisher(String, '/msgs', 1)

        self.br = CvBridge()


    #function that finds contours and their moments
    def find_contours(self, lmask, umask):
        
        
        frame_mask = cv2.inRange(self.hsv_frame, lmask, umask)
        contours, hierarchy = cv2.findContours(frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #Sort contours by area and keep the largest
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
        

        #returns needed values if contour is found and its area is large enough
        if len(contours) > 0:
            M = cv2.moments(contours[0])
            area = M['m00']

            if area>100:
                return [M, contours, area]
            
            else:
                return [False, False, False]
            
        else:
            return [False, False, False]


    def camera_callback(self, data):
        #or ((len(self.mask_list ==1)) and (self.mask_list[0] == self.red_mask1 or self.red_mask2))

        if (len(self.mask_list) == 0) :
            print("All Cylinders found, destroying node")
            rclpy.shutdown()


        #convert ROS image data into cv2 image frame
        self.bgr_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        #Convert image to HSV for better colour masking
        self.hsv_frame = cv2.cvtColor(self.bgr_frame, cv2.COLOR_BGR2HSV)

        moments_List = []
        contour_List = []
        carea_List = []

        #find any objects of interest by applying all masks
        for i in (self.mask_list):
            #f= []
            f = self.find_contours(i[0], i[1])

            if f != [False, False, False] and len(f)>0:
                moments_List.append(f[0])
                contour_List.append(f[1])
                carea_List.append(f[2])
            
            elif f == [False, False, False]:
                moments_List.append(0)
                contour_List.append(0)
                carea_List.append(0)
                self.colour_flag = False

            else:
                print("Empty list")

        index = 0

        #find object of interest using height of centroid
        
        for i in moments_List:
            
            l = math.inf
            if i != 0:
                cy = int(i['m01']/i['m00'])
                if cy<l:
                    l=cy
                    self.index_of_interest = index
                    self.colour_flag = True
            
            index +=1


        #checks if object of interest was found in camera
        if self.colour_flag == True:

            M = moments_List[self.index_of_interest]
            A = M['m00']
            
            frame_contours = cv2.drawContours(self.bgr_frame, contour_List[self.index_of_interest], 0, (0, 255, 0), 20)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            self.cy = cy
            
            cv2.circle(self.bgr_frame, (round(cx), round(cy)), 50, (0, 255, 0), -1)

            #chase centroid
            #left
            if cx < 900:
                    self.turn_vel = 0.3
                    self.lin_vel = 0.0
                    
            #right
            elif cx >= 1200:
                    self.turn_vel = -0.3
                    self.lin_vel = 0.0

            
            #centre
            else: 
                    self.turn_vel = 0.0
                    self.lin_vel = 0.2
            
            #if centroid is at a certain height and area of segment is of big enough area, can be assumed object is close enough
            if self.cy > 410 and A> 380000:
                print("Object found")
                self.mask_list.pop(self.index_of_interest)
                self.colour_flag = False


            current_frame_contours_small = cv2.resize(frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
            cv2.imshow("Image window", current_frame_contours_small)
            cv2.waitKey(1) 

        
        else:
            self.colour_flag = 0
            current_frame = cv2.resize(self.bgr_frame, (0,0), fx=0.4, fy=0.4) # reduce image size
            cv2.imshow("Image window", current_frame)
            cv2.waitKey(1)   

        
   
    def min_range(self, range):
        min_range = math.inf
        for v in range:
            if v < min_range:
                min_range = v
        return min_range



    def laser_callback(self, data):
        self.min_left = self.min_range(data.ranges[:60])
        self.min_right = self.min_range(data.ranges[-60:])
        
        self.centre = data.ranges[0]

        if self.timer_flag == False and self.start_flag == False:
            #simple reactive control behaviour
            twist = Twist()
    
            #Publishes velocity commands determined by camera input and/or laserscan data
        
            if self.min_left < self.min_distance:
                self.turn_vel = -0.2
                self.lin_vel = 0.0                
        
            elif self.min_right < self.min_distance:                
                self.turn_vel = 0.2 
                self.lin_vel = 0.0                

            elif ((self.min_range(data.ranges[:70]) < 0.3) and (self.min_range(data.ranges[-70:]) < 0.3) or self.centre < 0.3):    
                self.turn_vel = 3.14
                self.lin_vel = -0.2                
        
            else:                
                self.lin_vel = 0.2         

            twist.linear.x = self.lin_vel
            twist.angular.z = self.turn_vel
            self.pub_cmd_vel.publish(twist)

        elif self.timer_flag == True:
            self.rotate()
            self.timer_flag = False
        
        elif self.start_flag == True:
            self.rotate()
            self.rotate()
            self.rotate()
            self.start_flag = False
        
        #Publishes velocity commands determined by camera input
        if self.colour_flag == True:
            
            twist = Twist()
            twist.angular.z = self.turn_vel
            twist.linear.x = self.lin_vel
            self.pub_cmd_vel.publish(twist)
        
    
    def rotate(self):
        twist = Twist()
        twist.angular.z = 0.5
        self.pub_cmd_vel.publish(twist)
            
    #every t seconds if not chasing colour, the robot will turn in a circle to try and see if it has missed a potential pillar
    #or if it is stuck
    #This is performed in laser callback to prevent overlapping vel publishing
    def timer_callback(self):
        
        if self.timer_flag == True and self.colour_flag == False:
            self.timer_flag = False

        elif self.colour_flag == False:
            self.timer_flag = True



def main(args=None):
    rclpy.init(args=args)

    object_searcher = ObjectSearcher()
    rclpy.spin(object_searcher)

    object_searcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                
            

