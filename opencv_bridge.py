

#!/usr/bin/env python

# Written for humble
# cv2 image types - http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class OpencvBridge(Node):
    def __init__(self):
        super().__init__('opencv_bridge')
        self.imagesub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        self.br = CvBridge()

    def camera_callback(self, data):
        print("Callback")
        self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window")
        cv2.namedWindow("blur")
        cv2.namedWindow("canny")

        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray_img_small = cv2.resize(gray_img, (0,0), fx=0.2, fy=0.2) 
        print(np.mean(gray_img_small))

        blur_img = cv2.blur(gray_img, (3, 3))
        blur_img_small = cv2.resize(blur_img, (0,0), fx=0.2, fy=0.2) # reduce image size
        cv2.imshow("blur", blur_img_small)

        canny_img = cv2.Canny(gray_img, 10, 200)
        canny_img_small = cv2.resize(canny_img, (0,0), fx=0.2, fy=0.2) # reduce image size
        cv2.imshow("canny", canny_img_small)

        cv_image_small = cv2.resize(cv_image, (0,0), fx=0.2, fy=0.2) # reduce image size
        cv2.imshow("Image window", cv_image_small)
        cv2.waitKey(1)

def main(args=None):
    print('Starting opencv_bridge.py.')

    rclpy.init(args=args)

    opencv_bridge = OpencvBridge()

    rclpy.spin(opencv_bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    opencv_bridge.destroy_node()
    rclpy.shutdown()
    print("Finish")

if __name__ == '__main__':
    main()
