#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import other modules
import numpy as np
import math
import time

class object_detection(object):

    def __init__(self):
        rospy.init_node('object_detection')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        
        # Default vars
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.hsv_img = np.zeros((1920,1080,3), np.uint8)
        self.target_color_bounds = ([0, 255, 100], [0, 255, 255])
        self.status = 0

        # Shutdown hook 
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

        # Blue, Red, Green, Turquoise, Purple, Yellow hsv colour values:
        self.colour_boundaries = {
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Yellow": ([28, 180, 100], [32, 255, 255]),
            "Green":   ([25, 150, 100], [70, 255, 255]),
            "Turquoise":   ([75, 150, 100], [100, 255, 255]),
            "Purple":   ([145, 185, 100], [150, 250, 255])
        }

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
    
    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def rotate_by_degree(self, degree, direction="left"):
        time.sleep(1)
        speed = 0.2
        t = math.radians(degree) / speed
        if direction is "right":
            self.robot_controller.set_move_cmd(0.0, -speed)
        else:
            self.robot_controller.set_move_cmd(0.0, speed)
        self.robot_controller.publish()
        time.sleep(t)

        self.robot_controller.stop()

    def colour_selection(self):
        for colour_name, (lower, upper) in self.colour_boundaries.items():
            lower_bound = np.array(lower)
            upper_bound = np.array(upper)
            mask = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask.any():
                print("SEARCH INITIATED: The target colour is {}".format (colour_name))
                self.target_color_bounds = self.colour_boundaries[colour_name]
                break
        
    def main(self):
        while not self.ctrl_c:
            # Strats here:
            if self.status == 0:
                # rotate 180 degrees
                self.rotate_by_degree(90)
                self.status += 1
            if self.status == 1:
                self.colour_selection()
                self.status += 1
            if self.status == 2:
                self.rotate_by_degree(90, "right")
                self.status +=1
            else:
                break
            
            
            self.robot_controller.publish()
            self.rate.sleep()
        
            
if __name__ == '__main__':
    object_detection = object_detection()
    try:
        object_detection.main()
    except rospy.ROSInterruptException:
        pass