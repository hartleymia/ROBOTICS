#source code for the opencv_bridge node
#Mia Hartley 25179600 <25179600@students.lincoln.ac.uk>

#I have taken this code from the opencv_bridge code given in workshop 3 and edited it
#https://github.com/LCAS/teaching/blob/lcas_humble/cmp3103m_ros2_code_fragments/cmp3103m_ros2_code_fragments/opencv_bridge.py

#in order to identify the coloured objects in the simulation I used a technique called colour thresholding which involves
#selecting a range of colours and setting a binary threshold that marks the pixels within that range as white and the rest as black.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np

class OpencvBridge(Node):
    def __init__(self):
        super().__init__('opencv_bridge')
        self.br = CvBridge()
        
        #subscribe to camera topic
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10) 
        
        #define range of colours to detect in hsv format (these are the HSV values not RGB
        self.colors = {
            "red":([0, 70, 70], [5,255,255]),
            "yellow":([30, 50, 50], [90, 240, 240]),
            "green":([50, 50, 50], [70, 255, 255]),
            "blue":([100, 90, 50], [130, 255, 255])
        }
        
        #define the font for the text message
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 10
        self.font_color = (255, 255, 255) #RGB value, white
        self.line_type = cv2.LINE_AA
        
    def camera_callback(self, data):
     
        # Convert ROS Image message to OpenCV image
        cv_image = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        
        #convert image from bgr8 to hsv format
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        #threshold image for each colour
        mask_images = {}
        for color, (lower, upper) in self.colors.items():
            mask_images[color] = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
        
        #apply masks to original image using cv2.bitwise_and() function
        object_images = {}
        for color, mask_image in mask_images.items():
            object_images[color] = cv2.bitwise_and(cv_image, cv_image, mask=mask_image)
            
            #check if any pixels are detected
            if cv2.countNonZero(mask_image) > 0:
                #draw the text on the image
                cv2.putText(object_images[color], color, (10, 300), self.font, self.font_scale, self.font_color, self.line_type)
        
        #display the object images
        object_images = np.hstack(list(object_images.values()))
        object_image_small = cv2.resize(object_images, (0,0), fx=0.1, fy=0.15) #reduce image size
        cv2.imshow("object window", object_image_small)
        cv2.waitKey(1)
        
        #show camera view of turtlebot
        cv_image_small = cv2.resize(cv_image, (0,0), fx=0.2, fy=0.2) # reduce image size
        cv2.imshow("Image window", cv_image_small)
        cv2.waitKey(1)
        

def main(args=None):
    print('Starting opencv_bridge.py.')

    rclpy.init(args=args)
    opencv_bridge = OpencvBridge()
    rclpy.spin(opencv_bridge)
    # Destroy the node explicitly
    opencv_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 
