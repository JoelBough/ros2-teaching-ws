import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor

class PushBlocks(Node):

    def __init__(self):
        super().__init__('push_blocks')
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.findObjects, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)
        
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)


        self.br = CvBridge()

    #https://robotics.stackexchange.com/questions/24710/follow-wall-and-avoid-obstacles-using-lidar
    def scan_callback(self, scan: LaserScan):
        msg = Twist()


    def findObjects(self, data):
        #convert image message to OpenCV
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        #Convert to HSV
        current_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV) #
        #creates mask for colour range
        current_mask_green = cv2.inRange(current_hsv, (50, 100, 100), (70, 255, 255))
        current_mask_red = cv2.inRange(current_hsv, (160, 100, 100), (10, 255, 255))

        green_block, hierarchy = cv2.findContours(current_mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        #only keep second biggest green block to account for the green on the wall
        green_block = sorted(green_block, key=cv2.contourArea, reverse=True)[:1]
        #draw contour
        current_green_block = cv2.drawContours(current_frame, green_block, 0, (255, 255, 0), 20)

        self.tw = Twist()
        if len(green_block) > 0:
            #https://pyimagesearch.com/2016/02/01/opencv-center-of-contour/
            M = cv2.moments(green_block[0])
            CentreX = int(M["m10"]/M["m00"])#finds centre coordinates
            CentreY = int(M["m01"]/M["m00"])
            cv2.circle(current_frame, (CentreX, CentreY), 7, (255, 255, 255), -1)
            print("x:%s"% CentreX, "y:%s"% CentreY)

            if CentreX < 2*data.width/5:
                self.tw.linear.x = 0.1
                self.tw.angular.z = 0.3
            elif CentreX > 3*data.width/5:
                self.tw.linear.x = 0.1
                self.tw.angular.z = -0.3
            else:
                self.tw.angular.z = 0.0
                self.tw.linear.x = 0.2

        else:
            self.tw.angular.z = 0.3

        self.pub_cmd_vel.publish(self.tw)


    #def moveToObject(self, data):



def main(args = None):

    rclpy.init(args=args)
    push_blocks = PushBlocks()
    try:
        rclpy.spin(push_blocks)
        push_blocks.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Keystroke interrupt')
    finally:
        print('Simulation Terminated')

if __name__ == '__main__':
    main()