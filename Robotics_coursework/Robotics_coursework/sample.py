import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2
import numpy as np
from rclpy.executors import MultiThreadedExecutor


class ColourChaser(Node):
    def __init__(self):
        super().__init__('colour_chaser')

        # publish cmd_vel topic to move the robot
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        # subscribe to the camera topic
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.Laser_callback, 1)
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.distance_to_wall = 999
        self.perpendicular = 0
        self.push_dir = 0
        self.push_start = 0
        self.push_mode = 0
        self.push_rotate = 0
        self.push_counter = 0
        self.mode_2_counter = -1

        self.tracker = ["", "", "", ""]

        self.timer = self.create_timer(1, self.run_step)
        self.counter  = 0

    def camera_callback(self, data):
        #self.get_logger().info("camera_callback")

        cv2.namedWindow("Image window", 1)

        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert image to HSV
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        # Create mask for range of colours (HSV low values, HSV high values)
        #current_frame_mask = cv2.inRange(current_frame_hsv,(70, 0, 50), (150, 255, 255))
        current_frame_mask = cv2.inRange(current_frame_hsv,(60, 150, 50), (255, 255, 255)) 

        contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Sort by area (keep only the biggest one)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]

        # Draw contour(s) (image to draw on, contours, contour number -1 to draw all contours, colour, thickness):
        current_frame_contours = cv2.drawContours(current_frame, contours, 0, (255, 255, 0), 20)

        self.tw=Twist() # twist message to publish
        

        cx = -1
        cy = -1
        
        if len(contours) > 0 and self.push_mode == 0:
            # find the centre of the contour: https://dcv2.dcv2.drawContours()rawContours()ocs.opencv.org/3.4/d8/d23/classcv_1_1Moments.html
            M = cv2.moments(contours[0]) # only select the largest contour
            if M['m00'] > 0:
                # find the centroid of the contour
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                print("Centroid of the biggest area: ({}, {})".format(cx, cy))
        
        if len(contours) > 0 and self.push_mode == 0:
            # Draw a circle centered at centroid coordinates
            # cv2.circle(image, center_coordinates, radius, color, thickness) -1 px will fill the circle
            cv2.circle(current_frame, (round(cx), round(cy)), 5, (0, 255, 0), -1)
                        
            track_list = self.tracker[0] + self.tracker[1] + self.tracker[2] + self.tracker[3]
            in_loop = track_list == 'LRLR' or track_list == 'RLRL'
            print("list: ", track_list)

            # if center of object is to the left of image center move left
            if in_loop == False and cx < 3*data.width / 7 and (self.push_mode != 1 or self.distance_to_wall < 0.3):
                self.tw.angular.z=0.3
                print("Left")
                self.tracker.append("L")
                self.tracker.pop(0)
                self.push_mode = 0
            # else if center of object is to the right of image center move right
            elif in_loop == False and cx >= 4 * data.width / 7 and (self.push_mode != 1 or self.distance_to_wall < 0.3):
                self.tw.angular.z=-0.3
                print("Right")
                self.tracker.append("R")
                self.tracker.pop(0)
                self.push_mode = 0
            else: # center of object is in a 100 px range in the center of the image so dont turn
                print("object in the center of image")
                self.tw.angular.z=0.0
                self.tw.linear.x = 0.3
                self.push_mode = 1
                self.tracker.append("")
                self.tracker.pop(0)
                    
        else:
            print("No Centroid Found")
            self.tracker.append("")
            self.tracker.pop(0)
            if(self.push_mode != 1 or self.distance_to_wall < 0.3):
                if self.push_mode == 1:
                    self.push_mode = 0
                if self.push_mode == 2:
                    if self.push_start - self.distance_to_wall < -1.0 or self.counter - self.mode_2_counter > 5:
                        self.push_mode = 3
                        self.push_dir = 0
                        self.push_rotate = -0.5
                        self.push_counter = self.counter * 1 
                elif self.push_mode == 3:
                    if self.counter-self.push_counter >= 3:
                        self.push_mode = 0
                        self.push_dir = 0
                        self.push_rotate = 0.0
                elif self.counter - self.mode_2_counter < 30 and self.mode_2_counter >= 0:
                    self.push_mode = 0
                    self.push_dir = 0
                    self.push_rotate = -0.3
                else:
                    print("SC:", self.counter, ": M2C:", self.mode_2_counter)
                    self.push_dir = -1
                    self.push_rotate = 0.0
                    self.push_start = self.distance_to_wall * 1
                    self.push_mode = 2
                    self.mode_2_counter = self.counter * 1
                    print("SC:", self.counter, ": M2C:", self.mode_2_counter)
                    

                self.tw.linear.x = 0.5 * self.push_dir
                self.tw.angular.z = self.push_rotate
            elif len(contours) == 0 or cx < 3*data.width / 7 or cx > 4*data.width / 7:
                if self.push_dir == 0:
                    self.push_dir = 1
                    self.push_mode = 1
                    self.push_start = self.distance_to_wall*1
                elif self.push_dir == 1:
                    if self.push_start - self.distance_to_wall > 0.5:
                        self.push_dir = -1
                        self.push_mode = 1
                elif self.push_dir == -1:
                    if self.push_start - self.distance_to_wall <0.2:
                        self.push_dir = 0
                        self.push_mode = 0
                self.tw.angular.z=0.0
                self.tw.linear.x = 0.2 * self.push_dir
                print("Doing nothing")
            # turn until we can see a coloured object
            
        print(": LinX: ", self.tw.linear.x, ": AngZ: ", self.tw.angular.z, ": Dir: ", self.push_dir, ": PS-D:", round(self.push_start - self.distance_to_wall, 2), ": Mode: ", self.push_mode)
        self.pub_cmd_vel.publish(self.tw)

        # show the cv images
        current_frame_contours_small = cv2.resize(current_frame_contours, (0,0), fx=0.4, fy=0.4) # reduce image size
        cv2.imshow("Image window", current_frame_contours_small)
        #cv2.waitKey(1)


    def Laser_callback(self, msg):
        self.min_dist = 999
        self.min_index = -1
        for i in range(len(msg.ranges)):
            if msg.ranges[i] < self.min_dist:
                self.min_dist = msg.ranges[i]
                self.min_index = i

        self.distance_to_wall = self.min_dist

        self.perpendicular = int(100*(self.min_index - int(len(msg.ranges)/2))/int(len(msg.ranges)/2))
       # print("Angle: ", self.perpendicular)
        

        #print("Distance: ", self.distance_to_wall)

    def run_step(self):
        self.counter += 1



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