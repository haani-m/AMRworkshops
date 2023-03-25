#Haani Mahmood 250270480
#Autonomous Mobile RObitcs Coursework

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
'''
To do:
remove test code that manually picks mask colour

troubleshoot problem that is stopping mask from being changed
troubleshoot red mask chasing segments of orange walls DOUBLE CHECK THIS

OR find alternative way of finding out if object is <1m away
test adding masks (will need to change code a fair bit if continue with this)

work on roamer code, stop it from getting stuck in one place
maybe if no colour then spin
go forward at angle x, maybe random
spin again
and etc.

troubleshoot spin problem

'''

class ObjectSearcher(Node):
    def __init__(self):
        super().__init__('object_searcher')

        #HSV masks for the different coloured objects
        self.red_mask1 = [(160, 255, 50), (180, 255, 255)]
        self.red_mask2 = [(0, 255, 50), (10, 255, 255)] #works for column

        self.green_mask = [(50,100,100),(70,255,255)]
        self.yellow_mask = [(22, 93, 100), (45, 255, 255)]
        self.blue_mask = [(110, 50, 50), (130, 255, 255)]

        self.mask_list = (self.yellow_mask, self.red_mask1, self.green_mask,  self.blue_mask)
        

        #flags to determine behaviour
        self.colour_flag = False 
        

        self.turn_vel = 0.0
        self.lin_vel = 0.0
        self.min_distance = 0.7 #min distance used for roamer function
        self.object_max_dist = 1.0 # max distance robot can be from object
        self.centre_flag = False

        #publisher and subscribers
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        
        #-------------------------------------------------
        timer_period= 10
        self.timer = self.create_timer(timer_period, self.timer_callback) #may be unncecessary
        #!!!!!!!!!!!!!!!!!!!!!REMOVE!!!!!!!!!!!!!!!!!!!!!!!!!

        self.sub_image = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        self.sub_laser = self.create_subscription(LaserScan, '/scan', self.laser_callback, 1)

        self.pub_string = self.create_publisher(String, '/msgs', 1)

        self.br = CvBridge()


    #callback functions
    def camera_callback(self, data):
        

        #convert ROS image data into cv2 image frame
        bgr_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        #Convert image to HSV for better colour masking
        hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)


        self.mask_index = 0
        self.mask_colour = self.mask_list[self.mask_index]

        #apply mask and find contours
        print(self.mask_index)
        frame_mask = cv2.inRange(hsv_frame, self.mask_colour[0], self.mask_colour[1])
        contours, hierarchy = cv2.findContours(frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #Sort contours by area and keep the largest
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        #draw contours
        frame_contours = cv2.drawContours(bgr_frame, contours, -1, (0, 255, 0), 20)
        
        if len(contours) > 0:
            self.colour_flag = True
            M = cv2.moments(contours[0])

            #check contour area
            if M['m00'] > 20:
                #find centroid and draw circle
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                self.cx = cx
                self.cy = cy
                cv2.circle(bgr_frame, (round(cx), round(cy)), 50, (0, 255, 0), -1)
                            
                #determines which way to move depending on the position of the object within the camera frame
                #left
                if cx < 900:
                    self.turn_vel = 0.3
                    self.lin_vel = 0.0
                    self.centre_flag = False
                #right
                elif cx >= 1200:
                    self.turn_vel = -0.3
                    self.lin_vel = 0.0
                    self.centre_flag = False
                #centre
                else: 
                    self.turn_vel = 0.0
                    self.lin_vel = 0.2
                    self.centre_flag = True

        else:
            self.colour_flag = False

        current_frame_contours_small = cv2.resize(frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        cv2.waitKey(1)

  
    
    def min_range(self, range):
        min_range = math.inf
        for v in range:
            if v < min_range:
                min_range = v
        return min_range

    def laser_callback(self, data):

        Testflag1 = False

        forward = 0.2
        turn = 0.2
        if self.colour_flag == False and self.spin_flag == False:
            #checks left, right segments of laser scan
            min_left = self.min_range(data.ranges[:60])
            min_right = self.min_range(data.ranges[-60:])
            

            #create twist msg
            twist= Twist()

            #publish twist msg with data needed for current environment
            if min_left < self.min_distance:
                self.get_logger().info('turning right')
                twist.angular.z = -0.2
        
            elif min_right < self.min_distance:
                self.get_logger().info('turning left')
                twist.angular.z = 0.2

            elif (min_right < self.min_distance) and (min_left < self.min_distance):
                self.get_logger().info('turning around')
                twist.angular.z = 0.4
                twist.linear.x = -0.2
            else:
                self.get_logger().info("going straight")
                twist.linear.x = 0.2
        
            self.pub_cmd_vel.publish(twist)

        elif self.colour_flag == True and Testflag1 == False:

            #min_centre = self.min_range(data.ranges[0])
            min_centre = data.ranges[0]
            #print(data.ranges[0])
            print(min_centre)
            
            self.tw=Twist() # twist message to publish
            self.tw.linear.x = self.lin_vel
            self.tw.angular.z = self.turn_vel
            self.pub_cmd_vel.publish(self.tw)

            #check if object is within desired distance
            if self.centre_flag == True and min_centre < self.object_max_dist:
                self.string = String()
                self.string.data = "Object found!"
                self.pub_string.publish(self.string)
                print("Object found")
                print(self.mask_index)
                try:
                    self.mask_index +=1
                except IndexError:
                    #shutdown if there are no more objects to be found
                    rclpy.shutdown

                print(self.mask_index)
                #reset all flags and values to return to standard behaviour
                self.turn_vel = 0.0
                self.lin_vel = 0.0
                self.centre_flag = False
                self.colour_flag = False


        elif self.colour_flag == True and Testflag1 == True:
            
            opp = 960 - self.cx #distance between middle of camera and centroid
        
            

    def timer_callback(self):
        if self.colour_flag == False:
            self.tw = Twist() 
            self.tw.angular.z =  0.4
            self.pub_cmd_vel.publish(self.tw)
           


def main(args=None):
    rclpy.init(args=args)

    object_searcher = ObjectSearcher()
    rclpy.spin(object_searcher)

    object_searcher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
                
            
        
        
    


'''
References:

https://github.com/LCAS/teaching/blob/lcas_humble/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/colour_chaser2.py
-Used as the basis for the colour detection and "chasing" behaviour

https://github.com/LCAS/teaching/blob/lcas_humble/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/roamer.py
-Used as a basis for the standard roaming behaviour
'''