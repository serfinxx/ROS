#! /usr/bin/python
# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import all the necessary ROS message types:
from sensor_msgs.msg import LaserScan, Image

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
import cv2
from cv_bridge import CvBridge, CvBridgeError
from obstacle_avoidance import *

# Import other modules
import numpy as np
import math

class final_task(object):

    def __init__(self, speed=0.2, look_around_time=6):
        rospy.init_node('final_task')

        # Scan subscriber
        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)


        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        self.oa = ObstacleAvoidance(fdist_thresh=0.55, left_range=70, right_range=70, robot_controller=self.robot_controller, init=False)

        # Robot control vars
        self.robot_controller.set_move_cmd(0.0, 0.0)

        # Satus control vars
        self.status = 0
        self.task_done = False

        # Shutdown hook
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.arc_angles = np.arange(-96, 96)

        self.right_status = False
        self.left_status = False
        self.front_status = False
        self.T_turn = 0

        self.cvbridge_interface = CvBridge()

        r = 20 # Hz
        self.rate = rospy.Rate(r)
        self.look_around_time = r * look_around_time # look around every 3 seconds
        self.look_around_countdown = r * 10 # So that we don't start looking around immediately

        self.target_check = False
        self.found_target = False
        self.task_complete = False
        self.park_stage = 0
        
        self.speed = speed
        self.state = 0 # 0 = looking for beacon, 1 = Found Beacon

        # camera + colour
        self.view_high = False
        self.cam = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.hsv_img = np.zeros((1920, 1080, 3), np.uint8)
        self.target_colour_bounds = ([0, 0, 100], [255, 255, 255])
        self.mask = np.zeros((1920, 1080, 1), np.uint8)
        self.colour_boundaries = {
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Yellow": ([28, 180, 100], [32, 255, 255]),
            "Green":   ([25, 150, 100], [70, 255, 255]),
            "Turquoise":   ([75, 50, 100], [90, 255, 255]),
            "Purple":   ([145, 185, 100], [150, 250, 255])
        }
        self.m00 = 0
        self.m00_min = 10000

        self.width_crop = 800

    def shutdown_ops(self):
        self.robot_controller.stop()
        self.ctrl_c = True

    def scan_callback(self, scan_data):
        self.dead_left90 = scan_data.ranges[90]
        self.dead_right90 = scan_data.ranges[-90]
        self.dead_front = scan_data.ranges[0]

        self.front_arc = min(min(scan_data.ranges[0:10]), min(scan_data.ranges[-10:-1]))
        self.left_arc = min(scan_data.ranges[30:50])
        self.right_arc = min(scan_data.ranges[-50:-30])

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        height, width, channels = cv_img.shape

        crop_width = width - 800
        crop_height = 300
        offset = 300 if self.view_high else 0

        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height - crop_height - offset) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, 0:width]

        self.hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        self.mask = cv2.inRange(self.hsv_img, np.array(self.target_colour_bounds[0]), np.array(self.target_colour_bounds[1]))

        m = cv2.moments(self.mask)
        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)
        self.width = width

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), crop_height / 2), 10, (0, 0, 255), 2)
        
        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def rotate_by_degree(self, degree):
        rospy.sleep(1)
        speed = 0.4
        t = math.radians(abs(degree)) / speed
        if degree < 0:
            self.robot_controller.set_move_cmd(0.0, -speed)
        else:
            self.robot_controller.set_move_cmd(0.0, speed)
        self.robot_controller.publish()
        rospy.sleep(t)

        self.robot_controller.stop()

    def leave_spawn(self):
        # Method to leave spawn
        rospy.sleep(1)
        self.robot_controller.set_move_cmd(0.2, 0.0)
        self.robot_controller.publish()
        rospy.sleep(3)

    def check_right(self):
        # Check distance on the right (0.5)
        if self.dead_right90 < 0.5:
            self.right_status = True
        else:
            self.right_status = False

    def check_left(self):
        # Check distance on the left (0.5)
        if self.dead_left90 < 0.5:
            self.left_status = True
        else:
            self.left_status = False

    def check_front(self):
        # Check distance in front (0.5)
        if self.dead_front < 0.445:
            self.front_status = True
        else:
            self.front_status = False

    def turn_dead_right(self):
        # Turn Dead Right
        self.robot_controller.set_move_cmd(0.0, -0.62)
        self.robot_controller.publish()
        rospy.sleep(2.5)
        while self.dead_left90 > 0.358:
            self.robot_controller.set_move_cmd(0.0, -0.2)
            self.robot_controller.publish()
        self.robot_controller.stop()
        rospy.sleep(0.5)

    def turn_dead_left(self):
        # Turn Dead Left
        self.robot_controller.set_move_cmd(0.0, 0.62)
        self.robot_controller.publish()
        rospy.sleep(2.5)
        while self.dead_right90 > 0.358:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
        self.robot_controller.stop()
        rospy.sleep(0.5)

    def avoid_wall(self):
        # Wall avoidance
        if self.left_arc < 0.32:
            self.robot_controller.set_move_cmd(0.0, -0.3)
            self.robot_controller.publish()

        elif self.right_arc < 0.32:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
        else:
            if self.front_arc > 0.6:
                self.robot_controller.set_move_cmd(0.25, 0.0)
                self.robot_controller.publish()
            else:
                self.robot_controller.set_move_cmd(0.15, 0.0)
                self.robot_controller.publish()

    def colour_selection(self):
        for name, (lower, upper) in self.colour_boundaries.items():
            lb = np.array(lower)
            ub = np.array(upper)
            mask_checker = cv2.inRange(self.hsv_img, lb, ub)

            if mask_checker.any():
                return name, (lower, upper)

    def check_for_target(self):
        selection = self.colour_selection()
        if selection != None:
            _, bounds = selection
            self.found_target = bounds == self.target_colour_bounds
            return True
        return False
    
    def look_around(self):
        print("Look Around")
        angle = 30 # Sweep 30 Deg

        # Check forward
        found_a_col = self.check_for_target()
        if self.found_target:
            return
        # Check left
        self.robot_controller.deg_rotate(-angle)
        found_a_col |= self.check_for_target()
        if self.found_target:
            return
        
        self.robot_controller.deg_rotate(angle)

        # Check right
        self.robot_controller.deg_rotate(angle)
        found_a_col |= self.check_for_target()
        if self.found_target:
            return
        
        self.robot_controller.deg_rotate(-angle)

        if found_a_col:
            self.robot_controller.deg_rotate(-10)
        

    def seek(self):
        self.oa.attempt_avoidance()
        self.robot_controller.set_move_cmd(self.speed, 0.0)
        self.robot_controller.publish()

        if self.target_check:
            self.check_for_target()
        
        self.look_around_countdown -= 1
        if self.look_around_countdown < 1:
            self.target_check = True

    def park_at_target(self):
        if self.park_stage == 0:
            print("TARGET BEACON IDENTIFIED: Beaconing Initiated")
            self.view_high = True
            self.found_target = False
            self.park_stage += 1
            self.robot_controller.set_move_cmd(0.0, 0.0)
            self.robot_controller.publish()
        elif self.park_stage == 1:
            self.check_for_target()
            if self.m00 > self.m00_min:
                self.park_stage += 1
                print("Lock on {}".format(self.cy))
            else:
                self.robot_controller.deg_rotate(5)
        elif self.park_stage == 2:
            rot = ((self.cy / self.width) - (0.5 * (0 if self.cy == 0.0 else 1))) * -1
            self.robot_controller.deg_rotate(rot)

            straight_ahead = abs(rot) < 0.1

            if straight_ahead:
                self.robot_controller.deg_rotate(-10)
                self.park_stage += 1
                self.robot_odom.cache_current_data()

        else:
            self.robot_controller.set_move_cmd(self.speed, 0.0)
            self.robot_controller.publish()

            # at the start of this step cache the yaw
            # when the yaw deviates from that value, wait a bit
            # then take into account the distance covered in that distance
            # then attempt to rotate to make up for that distance 

            if self.m00 == 0.0:
                self.park_stage = 0
                self.found_target = False
            elif abs(self.m00) < 1000000:
                self.oa.attempt_avoidance()

            if (self.robot_odom.yaw != self.robot_odom.cache_yaw):
                print("Curr Y: {} Cached Y: {} M00: {}".format(self.robot_odom.yaw, self.robot_odom.cache_yaw, self.m00))

            # Good M00: 101439000.0, 142878795.0, 94630500.0

            if self.oa.lidar[FRONT] < 0.3:
                print("M00: {}".format(self.m00))
                self.task_complete = True

    def action_launcher(self):
        rospy.sleep(1)
        self.robot_controller.deg_rotate(90)

        target, self.target_colour_bounds = self.colour_selection()

        self.robot_controller.deg_rotate(-90)
        self.robot_controller.publish()

        print("TARGET COLOUR DETECTED: The target beacon is {}".format(target))

        p = True

        self.leave_spawn()   #move forward

        while not self.ctrl_c and not self.task_done:
            if self.found_target or self.view_high:
                self.park_at_target()
            else:
                self.check_left()
                self.check_right()
                self.check_front()
                if self.left_status == True and self.front_status == True:
                    self.robot_controller.stop()
                    self.turn_dead_right()
                elif self.right_status == True and self.front_status == True:
                    self.robot_controller.stop()
                    self.turn_dead_left()
                elif self.front_status == True and self.right_status == False and self.left_status == False:
                    if self.T_turn == 0:
                        self.robot_controller.stop()
                        self.turn_dead_left()
                        self.T_turn += 1
                    elif self.T_turn == 1:
                        self.robot_controller.stop()
                        self.turn_dead_right()
                        self.T_turn += 1
                    elif self.T_turn == 2:
                        self.robot_controller.stop()
                        self.turn_dead_left()
                else:
                    self.avoid_wall()
                self.seek()
            self.rate.sleep()

        self.robot_controller.stop()

        if self.task_complete:
            print("BEACONING COMPLETE: The robot has now reached the target")

if __name__ == '__main__':
    ft = final_task()
    try:
        ft.action_launcher()
    except rospy.ROSInterruptException:
        pass
