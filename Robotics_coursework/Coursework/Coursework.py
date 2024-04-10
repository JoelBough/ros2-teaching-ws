import rclpy
from rclpy.node import Node


from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import time
import math


class BullDozer(Node):

    def __init__(self):

        super().__init__('bulldozer')

        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        #subscribe to camera and laser scanner
        self.create_subscription(Image, '/limo/depth_camera_link/image_raw', self.camera_callback, 1)
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 1)

        self.br = CvBridge()

        #image State
        self.contours = []
        self.image_width = 0
        self.largest_block_x = -1
        self.in_line_block_x = -1
        self.in_line_block_area = 0
        self.in_line_block_dx = -1 #number of pixels target block is from the centre


        #laser state
        self.min_distance = -1 
        self.min_index = -1
        self.forward_distance = -1
        self.perpendicular = 0

        #current operation state
        self.current_operation = "start"
        self.approach_start_distance = -1
        self.push_start_distance = -1
        self.rotate_start_time = 0
        self.fine_tune_start_time = 0

        #operational configuration
        self.config_in_line_block_dx_tolerance_pixels = 5  #pixels from centre
        #bounds find the central seventh of the camera output
        self.config_in_line_lower_bound = 0.4 
        self.config_in_line_upper_bound = 0.6
        self.config_target_distance = 0.3#cant be closer than 30cm to a wall
        self.config_push_increment = 0.5
        self.config_rotate_rate = 0.3 #will rotate 0.3 rad/s anti-clockwise
        self.config_rotate_seconds = 1.5
        self.config_fine_tune_rate = 0.04
        self.config_fine_tune_time_limit = 2

        #twist message to publish
        self.tw = Twist()
        self.tw.linear.x = 0.0
        self.tw.angular.z = 0.0
        

    def initialise_operation(self):
        #robot can be in three states, approaching a block, pushing a block or rotating
        if self.current_operation == "approach":
            self.approach_start_distance = self.forward_distance #assigns the distance from the wall to approach start to control distance traveled

        elif self.current_operation == "push":
            self.push_start_distance = self.forward_distance

        elif self.current_operation == "rotate":
            self.rotate_start_time = time.time()


    def refresh_bulldozer(self):
        #used to process the current operation(search > approach > push > reset > rotate)
        prev_operation = self.current_operation
        prev_linear_x = self.tw.linear.x
        prev_angular_z = self.tw.angular.z

        if self.current_operation == "start":
            self.tw.linear.x = 0.0
            self.tw.angular.z = 0.0
            self.current_operation = "search"
            

        elif self.current_operation == "search":
            #find a qualifying block if there is not already one in line
            if self.in_line_block_x == -1:
                #if no block in line, rotate anticlockwise on the spot
                self.tw.linear.x = 0.0
                self.tw.angular.z = 0.3
                self.fine_tune_start_time = 0
            elif (self.fine_tune_start_time ==0 or (time.time() - self.fine_tune_start_time) < self.config_fine_tune_time_limit) and abs(self.in_line_block_dx)>(self.config_in_line_block_dx_tolerance_pixels):
                #turns the block slightly until it is perfectly in line
                self.fine_tune()    #rotates the robot towards the target, faster the further away the targe block is
            else:
                self.fine_tune_start_time = 0
                #once in line with the block, start moving towards it
                self.current_operation = "approach"

        elif self.current_operation == "approach":
            #move towards the identified and central block
            
            if self.in_line_block_x > -1:
                if (self.fine_tune_start_time ==0 or (time.time() - self.fine_tune_start_time) < self.config_fine_tune_time_limit) and abs(self.in_line_block_dx)>(self.config_in_line_block_dx_tolerance_pixels):
                #turns the block slightly until it is perfectly in line
                    self.fine_tune()
                else:
                    self.fine_tune_start_time = 0
                    self.tw.linear.x = 0.3     
                    self.tw.angular.z = 0.0
            else:    #if there are no visible blocks(the block has gone off the screen), move to pushing
                self.current_operation = "push"

        elif self.current_operation == "push":
            push_distance = self.push_start_distance - self.forward_distance 
            if self.min_distance > self.config_target_distance and (push_distance) < self.config_push_increment:
                #if any of the laser inputs detect a distance less than 30cm to wall or we have moved less than 50cm, keep driving forward
                self.tw.linear.x = 0.2
                self.tw.angular.z = 0.0
            else:
                #if we are close to the wall or have pushed the block the push increment
                self.current_operation = "reset"

        elif self.current_operation == "reset":
            if self.forward_distance < self.approach_start_distance:
                #move back to the position the robot was in before the approach started
                self.tw.linear.x = -0.3
                self.tw.angular.z = 0.0
            else:
                #rotate so the previous target block is out of view
                self.current_operation = "rotate"

        elif self.current_operation == "rotate":
            #rotate slightly to get previous block out of view
            if (time.time() - self.rotate_start_time) < self.config_rotate_seconds:
                self.tw.linear.x = 0.0
                self.tw.angular.z = self.config_rotate_rate
            else:
                #once rotation is done, search for a new block
                self.current_operation = "search"

        
        if self.current_operation != prev_operation:
            #stop all previous movement when changing operations
            #initialise the new operation
            self.initialise_operation()
            #execute on new operation
            self.refresh_bulldozer()
        else:
            #if there is no change in operation, maintain movement
            self.pub_cmd_vel.publish(self.tw)

    def contour_x_is_in_line(self, contour_x):
        #checks if contour is between the upper and lower bounds of the centre of the image
        lower_bound = self.config_in_line_lower_bound * self.image_width
        upper_bound = self.config_in_line_upper_bound * self.image_width
        return lower_bound < contour_x < upper_bound #True if contour is in the centre, false if not
    
    def contour_x_distance_from_centre(self, contour_x):
        #measures the number of pixels away from the centre of the image.
        #negative return means left of centre, positive return means to the right
        return contour_x - self.image_width//2
    
    def find_contour_centre_x(self, contour):
        #find centre x from centroid moments
        #taken from Colour chaser workshop 03
        cx = -1
        m = cv2.moments(contour)
        if m['m00'] > 0:
            cx = int(m['m10']/m['m00'])
        return cx

    def fine_tune(self): #slowly calibrate orientation to keep target block in centre of image
        
        if self.fine_tune_start_time == 0:
            self.fine_tune_start_time = time.time()
        self.tw.linear.x = 0.0
        ratio = abs(self.in_line_block_dx)/(self.config_in_line_block_dx_tolerance_pixels)
        if ratio > 5:
            ratio = 5
        direction = (-self.in_line_block_dx / abs(self.in_line_block_dx))
        self.tw.angular.z = self.config_fine_tune_rate * ratio * direction
    def camera_callback(self, data):
        #stores the image from the robot camera and its characteristics

        #converts ROS2 image to OpenCV
        #taken from colour chaser workshop 03
        current_frame = self.br.imgmsg_to_cv2(data, desired_encoding='bgr8')
        current_frame_hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV) # convert to hsv
        current_frame_mask = cv2.inRange(current_frame_hsv, (60, 150, 50), (255, 255, 255))
        contours, hierarchy = cv2.findContours(current_frame_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        contours = sorted(contours, key=cv2.contourArea, reverse = True)#sorts the contours by area

        #find the largest contour and the largest or first in the centre
        in_line_block_x = -1
        in_line_block_dx = -1
        in_line_block_area = 0
        for contour in contours:
            cx = self.find_contour_centre_x(contour)
            if self.contour_x_is_in_line(cx):
                dx = self.contour_x_distance_from_centre(cx)
                if in_line_block_dx == -1 or abs(dx) < in_line_block_dx:
                    in_line_block_x = cx
                    in_line_block_area = cv2.contourArea(contour) * 1
                    in_line_block_dx = dx

        #store results in self
        self.in_line_block_x = in_line_block_x
        self.in_line_block_dx = in_line_block_dx
        self.in_line_block_area = in_line_block_area

        if len(contours) > 0:
            self.largest_block_x = self.find_contour_centre_x(contours[0])
        else:
            self.largest_block_x = -1

        
        #draw the contours onto the current frame 
        current_frame_contours = cv2.drawContours(current_frame, contours, 0, (255, 255, 0), 20)
        #reduce image size and display(Workshop 03)
        current_frame_contours_small = cv2.resize(current_frame_contours, (0, 0), fx = 0.4, fy=0.4)
        cv2.namedWindow("Image window", 1)
        cv2.imshow("Image window", current_frame_contours_small)

        #store the contours in self
        self.contours = contours.copy()
        self.image_width = data.width
        
        #process current operation
        self.refresh_bulldozer()


    def laser_callback(self, msg):
        
        #calculates characteristics of the robot relative to the walls
        min_distance = -1
        min_index = -1
        forward_distance = -1

        #loops through each laser to obtain location information
        for i in range(len(msg.ranges)):
            r = msg.ranges[i] * 1
            if math.isinf(r):
                r = 0
            if i == len(msg.ranges)//2:#records value of middle(forward) laser 
                forward_distance = r
            #if current laser is the closest to the wall so far, record it as it is the closest perpendicular distance to the wall
            if r < min_distance or min_distance < 0:
                min_distance = r
                min_index = i * 1


        #transfer information to self
        self.min_distance = min_distance * 1
        self.min_index = min_index * 1
        self.forward_distance = forward_distance * 1

        #records the degree of the perpendicular laser(0=forward is perpendicular, +/- 100 is first/last laser perpendicular)
        self.perpendicular = int(100*(self.min_index - int(len(msg.ranges)/2))/int(len(msg.ranges)/2))
        #process current operation
        self.refresh_bulldozer()

def main(args = None):

    rclpy.init(args=args)
    bulldozer = BullDozer()
    try:
        rclpy.spin(bulldozer)
        bulldozer.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print('Keystroke interrupt')
    finally:
        print('Simulation Terminated')

if __name__ == '__main__':
    main()