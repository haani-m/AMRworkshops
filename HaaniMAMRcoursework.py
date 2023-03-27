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
get rid of test flag
clean up camera callback 1 and 2

test circle spinning on timer callback


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

        self.mask_list = [self.yellow_mask, self.red_mask2, self.green_mask,  self.blue_mask]
        
        self.index_of_interest = -1

        #flags to determine behaviour
        self.colour_flag = False 
        self.centre_flag = False

        

        self.turn_vel = 0.0
        self.lin_vel = 0.0
        self.min_distance = 0.7 #min distance used for roamer function
        self.object_max_dist = 1.0 # max distance robot can be from object

        
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

            if area>20:
                return [M, contours, area]
            
            else:
                return [False, False, False]
            
        else:
            return [False, False, False]


    def camera_callback(self, data):
        
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

            else:
                print("Empty list")


        #find object of interest with the biggest area to determine which mask to use
        for i in carea_List:
            largest = 0
            index = 0
            if i > largest:
                largest = i
                self.index_of_interest = index
            
            index += 1

        #checks if object of interest was found in camera
        if self.index_of_interest != -1:

            M = moments_List[self.index_of_interest]
            #draw contour and centroid
            frame_contours = cv2.drawContours(self.bgr_frame, contour_List[self.index_of_interest], 0, (0, 255, 0), 20)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            self.cy = cy
            print("CY:", cy)
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

                    #if self.centre < self.object_max_dist:
                     #   print("Distance:", self.centre, "Y positions:", cy)

            

            '''
            #find way of determining if OOI is close enough
            #temporary value
            elif cy < 500:
                print("Object found")
                self.mask_list.pop(self.index_of_interest)
                self.index_of_interest = -1
            '''
            current_frame_contours_small = cv2.resize(frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
            cv2.imshow("Image window", current_frame_contours_small)
            cv2.waitKey(1) 

        #roaming behaviour is implemented if no OOI are found, found in laserscan callback
        else:
            self.colour_flag = 0
            self.index_of_interest = -1 
            cv2.imshow("Image windows", self.bgr_frame)

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
        self.min_left = self.min_range(data.ranges[:60])
        self.min_right = self.min_range(data.ranges[-60:])
        #self.centre = self.min_range(data.ranges[-10:10])
        self.centre = data.ranges[0]

        if self.index_of_interest == -1:
            #simple reactive control behaviour
            twist = Twist()

            if self.min_left < self.min_distance:
                self.get_logger().info('turning right')
                twist.angular.z = -0.2
        
            elif self.min_right < self.min_distance:
                self.get_logger().info('turning left')
                twist.angular.z = 0.2

            elif (self.min_right < self.min_distance) and (self.min_left < self.min_distance):
                self.get_logger().info('turning around')
                twist.angular.z = 0.4
                twist.linear.x = -0.2
            else:
                self.get_logger().info("going straight")
                twist.linear.x = 0.2
        
            self.pub_cmd_vel.publish(twist)
        
        else:
            
            twist = Twist()
            twist.angular.z = self.turn_vel
            twist.linear.x = self.lin_vel
            self.pub_cmd_vel.publish(twist)

            if self.cy > 410:
                print("Distance:", self.centre, "Y positions:", self.cy)

                self.mask_list.pop(self.index_of_interest)


            
        '''
        Testflag1 = False

        forward = 0.2
        turn = 0.2
        if self.colour_flag == False:
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
            
            opp = 960 - self.cx #distance between middle of camera and centroid'''
        
            
    #every t seconds if not chasing colour, the robot will turn in a circle to try and see if it has missed a potential pillar
    def timer_callback(self):
        if self.index_of_interest == -1:
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