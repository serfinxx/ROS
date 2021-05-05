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

# Import other modules
import numpy as np
import math
import time

class search_and_beaconing(object):

    def __init__(self):
        rospy.init_node('search_and_beaconing')
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        
        # Default vars
        self.robot_controller.set_move_cmd(0.0, 0.0)
        self.hsv_img = np.zeros((1920,1080,3), np.uint8)
        self.target_color_bounds = ([0, 0, 100], [255, 255, 255])
        self.mask = np.zeros((1920,1080,1), np.uint8)
        self.status = 0
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.move_rate = '' # fast, slow or stop
        self.m00 = 0
        self.m00_min = 100000

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

        self.mask = cv2.inRange(self.hsv_img, np.array(self.target_color_bounds[0]), np.array(self.target_color_bounds[1]))

        m = cv2.moments(self.mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)


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
            mask_checker = cv2.inRange(self.hsv_img, lower_bound, upper_bound)
            if mask_checker.any():
                print("SEARCH INITIATED: The target colour is {}.".format (colour_name))
                self.target_color_bounds = self.colour_boundaries[colour_name]
                break
        
    def action_launcher(self):
        while not self.ctrl_c:
            # Strats here:
            if self.status == 0:        # rotate 180 degrees
                self.rotate_by_degree(90)
                self.status += 1
            elif self.status == 1:      # select target colour
                self.colour_selection()
                self.status += 1
            elif self.status == 2:      # turn back
                self.rotate_by_degree(90, "right")
                self.status +=1
            elif self.status == 3:      # move to centre of map
                self.robot_controller.set_move_cmd(0.2, 0.0)
                self.robot_controller.publish()
                time.sleep(5)
                self.status += 1
            elif self.status == 4:      # turn left to start scanning
                self.rotate_by_degree(100)
                self.status += 1
            elif self.status == 5:      # scan
                if self.m00 > self.m00_min:
                # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                    else:
                        self.move_rate = 'slow'
                else:
                    self.move_rate = 'fast'
                    
                if self.move_rate == 'fast':
                    # print("MOVING FAST: I can't see anything at the moment (blob size = {:.0f}), scanning the area...".format(self.m00))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                elif self.move_rate == 'slow':
                    # print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                elif self.move_rate == 'stop':
                    # print("STOPPED: The blob of colour is now dead-ahead at y-position {:.0f} pixels...".format(self.cy))
                    self.robot_controller.stop()
                    self.status += 1
                else:
                    # print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                
                self.robot_controller.publish()
                self.rate.sleep()
            else:
                print("SEARCH COMPLETE: The robot is now facing the target pillar.")
                break
            
        
            
if __name__ == '__main__':
    search_and_beaconing = search_and_beaconing()
    try:
        search_and_beaconing.action_launcher()
    except rospy.ROSInterruptException:
        pass