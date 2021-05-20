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
        """
            This method is only used at beginning for leaving spawn.
        """
        rospy.sleep(1)
        self.robot_controller.set_move_cmd(0.2, 0.0)
        self.robot_controller.publish()
        rospy.sleep(3)

    def check_right(self):
        """
            Check the distance over 0.5 of right
        """
        if self.dead_right90 < 0.5:
            self.right_status = True
        else:
            self.right_status = False

    def check_left(self):
        """
            Check the distance over 0.5 of left
        """
        if self.dead_left90 < 0.5:
            self.left_status = True
        else:
            self.left_status = False

    def check_front(self):
        """
            Check the distance over 0.5 of front
        """
        if self.dead_front < 0.445:
            self.front_status = True
        else:
            self.front_status = False

    def turn_dead_right(self):
        """
            Turn to dead right direction
        """
        print "turn right to dead dir"
        self.robot_controller.set_move_cmd(0.0, -0.62)
        self.robot_controller.publish()
        rospy.sleep(2.5)
        while self.dead_left90 > 0.358:
            self.robot_controller.set_move_cmd(0.0, -0.2)
            self.robot_controller.publish()
        self.robot_controller.stop()
        print"stopped"
        rospy.sleep(0.5)

    def turn_dead_left(self):
        """
            Turn to dead left direction
        """
        print "turn left to dead dir"
        self.robot_controller.set_move_cmd(0.0, 0.62)
        self.robot_controller.publish()
        rospy.sleep(2.5)
        while self.dead_right90 > 0.358:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
        self.robot_controller.stop()
        print"stopped"
        rospy.sleep(0.5)

    def avoid_wall(self):
        """
            Wall avoidance
        """
        if self.left_arc < 0.32:
            self.robot_controller.set_move_cmd(0.0, -0.3)
            self.robot_controller.publish()
            print("adjusting, turning right")
        elif self.right_arc < 0.32:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
            print("adjusting, turning left")
        else:
            if self.front_arc > 0.6:
                self.robot_controller.set_move_cmd(0.25, 0.0)
                self.robot_controller.publish()
                print("moving fast")
            else:
                self.robot_controller.set_move_cmd(0.15, 0.0)
                self.robot_controller.publish()
                print("moving slow")

    def action_launcher(self):
        self.leave_spawn()   #move forward
        while not self.ctrl_c and not self.task_done:
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


if __name__ == '__main__':
    ft = final_task()
    try:
        ft.action_launcher()
    except rospy.ROSInterruptException:
        pass
