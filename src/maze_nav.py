#! /usr/bin/python
# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import all the necessary ROS message types:
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import other modules
import numpy as np
import math

class maze_navigation(object):

    def __init__(self):
        rospy.init_node('maze_navigation')

        # Lidar subscriber
        self.lidar_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback)
        self.lidar = {'front_distance': 0.0,
                      'fleft_distance': 0.0,
                      'fright_distance': 0.0}

        # Scan subscriber 
        self.scan_subscriber = rospy.Subscriber("/scan",
            LaserScan, self.scan_callback)


        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        # Robot control vars
        self.robot_controller.set_move_cmd(0.0, 0.0)

        # Lidar vars
        self.raw_data = np.array(tuple())

        # Satus control vars
        self.status = 0
        self.task_done = False

        # Shutdown hook 
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(100000)

        # Blue, Red, Green, Turquoise, Purple, Yellow hsv colour values:
        self.colour_boundaries = {
            "Red":    ([0, 185, 100], [10, 255, 255]),
            "Blue":   ([115, 224, 100],   [130, 255, 255]),
            "Yellow": ([28, 180, 100], [32, 255, 255]),
            "Green":   ([25, 150, 100], [70, 255, 255]),
            "Turquoise":   ([75, 50, 100], [90, 255, 255]),
            "Purple":   ([145, 185, 100], [150, 250, 255])
        }

        self.arc_angles = np.arange(-96, 96)

        self.right_status = False
        self.left_status = False
        self.front_status = False
        self.T_turn = 0

    def shutdown_ops(self):
        self.robot_controller.stop()
        self.ctrl_c = True

    def scan_callback(self, scan_data):
        self.dead_left90 = scan_data.ranges[90]
        self.dead_right90 = scan_data.ranges[-90]
        self.dead_front = scan_data.ranges[0]
        left_arc45_55 = scan_data.ranges[30:40]
        left_arc55_60 = scan_data.ranges[40:50]
        left_arc45_60 = np.array(left_arc55_60[::-1] + left_arc45_55[::-1])
        self.left_arc = left_arc45_60.min()

        right_arc45_55 = scan_data.ranges[-40:-30]
        right_arc55_60 = scan_data.ranges[-50:-40]
        right_arc45_60 = np.array(right_arc55_60[::-1] + right_arc45_55[::-1])
        self.right_arc = right_arc45_60.min()
    
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
        speed = 0.4
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
        rospy.sleep(2)
    
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
        rospy.sleep(2)                
    
    def avoid_wall(self):
        """
            Wall avoidance
        """
        if self.left_arc < 0.3:
            self.robot_controller.set_move_cmd(0.0, -0.3)
            self.robot_controller.publish()
            print "adjusting, turning right"
        elif self.right_arc < 0.3:
            self.robot_controller.set_move_cmd(0.0, 0.3)
            self.robot_controller.publish()
            print" adjusting, turning left"
        else:            
            self.robot_controller.set_move_cmd(0.2, 0.0)    
            self.robot_controller.publish()  

    def action_launcher(self):
        self.leave_spawn()   #move forward
        while not self.ctrl_c or not self.task_done:
            self.check_left()
            self.check_right()
            self.check_front()
            if self.left_status == True and self.front_status == True:
                self.robot_controller.stop()
                rospy.sleep(1.5) 
                self.turn_dead_right()
            elif self.right_status == True and self.front_status == True:
                self.robot_controller.stop()
                rospy.sleep(1.5) 
                self.turn_dead_left()
            elif self.front_status == True and self.right_status == False and self.left_status == False:
                if self.T_turn == 0:
                    self.robot_controller.stop()
                    rospy.sleep(1.5)   
                    self.turn_dead_left()
                    self.T_turn += 1
                elif self.T_turn == 1:
                    self.robot_controller.stop()
                    rospy.sleep(1.5) 
                    self.turn_dead_right()
                    self.T_turn += 1
                elif self.T_turn == 2:
                    self.robot_controller.stop()
                    rospy.sleep(1.5) 
                    self.turn_dead_left()
            else:
                self.avoid_wall()


if __name__ == '__main__':
    mn = maze_navigation()
    try:
        mn.action_launcher()
    except rospy.ROSInterruptException:
        pass