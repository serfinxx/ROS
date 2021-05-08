#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import other modules
import numpy as np
import math

class search_and_beaconing(object):

    def __init__(self):
        rospy.init_node('search_and_beaconing')

        # Camera subscriber
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)

        # Lidar subscriber
        self.lidar_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback)
        self.lidar = {'front_distance': 0.0,
                      'fleft_distance': 0.0,
                      'fright_distance': 0.0}

        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
        
        # Robot control vars
        self.robot_controller.set_move_cmd(0.0, 0.0)

        # Camera vars
        self.hsv_img = np.zeros((1920,1080,3), np.uint8)
        self.target_color_bounds = ([0, 0, 100], [255, 255, 255])
        self.mask = np.zeros((1920,1080,1), np.uint8)
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.move_rate = '' # fast, slow or stop
        self.m00 = 0
        self.m00_min = 100000

        # Lidar vars
        self.raw_data = np.array(tuple())

        # Start zone checker vars
        self.init_x = self.robot_odom.posx
        self.init_y = self.robot_odom.posy

        # Satus control vars
        self.status = 0
        self.task_done = False

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

    def lidar_callback(self, lidar_data):
        """Returns arrays of lidar data"""

        self.raw_data = np.array(lidar_data.ranges)

        # Distance Detection
        self.lidar['front_distance'] = min(
            min(min(self.raw_data[342:359]), min(self.raw_data[0:17])), 10)      # front 36 degrees
        self.lidar['fleft_distance'] = min(
            min(self.raw_data[306:342]), 10)      # front left 36 degrees
        self.lidar['fright_distance'] = min(
            min(self.raw_data[18:54]), 10)      # front right 36 degrees

    def rotate_by_degree(self, degree):
        rospy.sleep(1)
        speed = 0.8
        t = math.radians(abs(degree)) / speed
        if degree < 0:
            self.robot_controller.set_move_cmd(0.0, -speed)
        else:
            self.robot_controller.set_move_cmd(0.0, speed)
        self.robot_controller.publish()
        rospy.sleep(t)

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

    def spawn_zone_checker(self):
        current_x = self.robot_odom.posx
        current_y = self.robot_odom.posy
        distance_to_spawn = np.sqrt(pow(current_x-self.init_x, 2) + pow(current_y-self.init_y, 2))

        
        current_global_yaw = (self.robot_odom.yaw+360)%360
        global_angle_to_spawn = (math.degrees(math.atan2(self.init_x-current_x, self.init_y-current_y))-90+360)%360
        local_angle_to_spawn = int(round(global_angle_to_spawn-current_global_yaw+360)%360)
        
        checker_vision = 0

        if self.lidar['front_distance'] > 0.6:
            checker_vision = 20
        else: 
            checker_vision = 65

        left_lidar_index, right_lidar_index = int(local_angle_to_spawn-checker_vision), int(local_angle_to_spawn+checker_vision)

        checker_distances = []

        if left_lidar_index < 0:
            checker_distances.extend(self.raw_data[(left_lidar_index+360):359])
        else:
            checker_distances.extend(self.raw_data[left_lidar_index:local_angle_to_spawn])

        if right_lidar_index >= 360:
            checker_distances.extend(self.raw_data[0:(right_lidar_index-360)])
        else:
            checker_distances.extend(self.raw_data[local_angle_to_spawn:right_lidar_index])

        torlerance = 0.2

        checker = False

        for dis in checker_distances:
            if abs(dis-distance_to_spawn) < torlerance:
                checker =  True
                break
            else: 
                checker = False

        if distance_to_spawn < 1:
            checker = True

        return checker

    def target_seeker(self):
        if self.m00 > self.m00_min and not self.spawn_zone_checker():
            self.status +=1
            # blob detected
            if self.cy >= 560-100 and self.cy <= 560+100:
                if self.move_rate == 'slow':
                    self.move_rate = 'stop'
            else:
                self.move_rate = 'slow'
        else:
            self.move_rate = 'fast'
            
        if self.move_rate == 'fast':
            self.obstacle_avodance()
        elif self.move_rate == 'slow':
            if self.cy <= 560-100:
                if self.lidar['fleft_distance'] > 0.3 and self.lidar['fright_distance'] < 0.3:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=-0.3)
                elif self.lidar['fleft_distance'] < 0.3 and self.lidar['fright_distance'] > 0.3:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=0.3)
                else:
                    self.robot_controller.set_move_cmd(0.2, 0.15)
            else:
                if self.lidar['fleft_distance'] > 0.3 and self.lidar['fright_distance'] < 0.3:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=-0.3)
                elif self.lidar['fleft_distance'] < 0.3 and self.lidar['fright_distance'] > 0.3:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=0.3)
                else:
                    self.robot_controller.set_move_cmd(0.2, -0.15)
        elif self.move_rate == 'stop':
            if self.lidar['front_distance'] > 0.4:
                self.robot_controller.set_move_cmd(0.2, 0.0)
            else:
                self.robot_controller.set_move_cmd(0.0, 0.0)
                print("BEACONING COMPLETE: The robot has now stopped.")
                self.task_done = True
        else:
            self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
        
        self.robot_controller.publish()
        self.rate.sleep()

    def obstacle_avodance(self):
        if self.lidar['front_distance'] > 0.4:
                if self.lidar['fleft_distance'] > 0.3 and self.lidar['fright_distance'] < 0.3:
                    self.robot_controller.set_move_cmd(linear=0.025, angular=-0.6)
                elif self.lidar['fleft_distance'] < 0.3 and self.lidar['fright_distance'] > 0.3:
                    self.robot_controller.set_move_cmd(linear=0.025, angular=0.6)
                elif self.lidar['fleft_distance'] < 0.3 and self.lidar['fright_distance'] < 0.3:
                    self.robot_controller.set_move_cmd(linear=0.025, angular=-0.6)
                else:
                    self.robot_controller.set_move_cmd(linear=0.12, angular=0)
                    if self.lidar['front_distance'] > 0.9:
                        self.robot_controller.set_move_cmd(linear=0.26, angular=0)
        else:
            if self.lidar['fleft_distance'] > 0.3 and self.lidar['fright_distance'] > 0.3:
                self.robot_controller.set_move_cmd(linear=0.0, angular=-0.6)
            elif self.lidar['fleft_distance'] > 0.3 and self.lidar['fright_distance'] < 0.3:
                self.robot_controller.set_move_cmd(linear=0.0, angular=-0.6)
            elif self.lidar['fleft_distance'] < 0.3 and self.lidar['fright_distance'] > 0.3:
                self.robot_controller.set_move_cmd(linear=0.0, angular=0.6)
            else:
                self.robot_controller.set_move_cmd(linear=0.0, angular=-0.6)
        self.robot_controller.publish()

    def action_launcher(self):
        while not self.ctrl_c:
            # Strats here:
            if self.status == 0:        # rotate 90 degrees
                rospy.sleep(1)
                self.rotate_by_degree(90)
                self.status += 1
            if self.status == 1:      # select target colour
                self.colour_selection()
                self.status += 1
            if self.status == 2:      # turn back
                self.rotate_by_degree(-90)
                self.status +=1
                self.init_x= self.robot_odom.posx
                self.init_y= self.robot_odom.posy
            if self.status == 3:      # move out of start zone
                rospy.sleep(1)
                self.robot_controller.set_move_cmd(0.2, 0.0)
                self.robot_controller.publish()
                rospy.sleep(2)
                self.status += 1        
            else:       # walking & searching
                self.target_seeker()
                if self.status == 5:
                    print("BEACON DETECTED: Beaconing initiated.")
                    self.status +=1
                if self.task_done:
                    break
            
                
            
        
            
if __name__ == '__main__':
    search_and_beaconing = search_and_beaconing()
    try:
        search_and_beaconing.action_launcher()
    except rospy.ROSInterruptException:
        pass