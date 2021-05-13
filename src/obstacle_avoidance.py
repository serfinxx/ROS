#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package (copied from other package)
from move_tb3 import MoveTB3

# Import some other useful Python Modules
from math import radians
import datetime as dt
import os
import numpy as np

FRONT = 'front'
FRONT_LEFT = 'fleft'
FRONT_RIGHT = 'fright'

class ObstacleAvoidance(object):
    def __init__(self, front_range = 36, fleft_range = 36, fright_range = 36, fdist_thresh=0.4, rldist_thresh=0.1, robot_controller=None, init=True):
        # Initialise action server
        if init:
            rospy.init_node('obstacle_avoidance')
        # Lidar subscriber
        self.lidar_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.lidar_callback)

        self.lidar = { FRONT: 0.0, FRONT_LEFT: 0.0, FRONT_RIGHT: 0.0 }
        self.raw_data = np.array(tuple())

        f_right = int(front_range / 2)
        f_left = 359 - f_right

        self.r_front = [ (f_left, 359), (0, f_right) ]
        self.r_fleft = (f_left - fleft_range, f_left)
        self.r_fright = (f_right, f_right + fright_range)

        self.fdist_thresh = fdist_thresh
        self.rldist_thresh = rldist_thresh

        # Robot movement and odometry
        if robot_controller is None:
            self.robot_controller = MoveTB3()
        else:
            self.robot_controller = robot_controller

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

    def lidar_callback(self, lidar_data):
        """Returns arrays of lidar data"""

        self.raw_data = np.array(lidar_data.ranges)
        f = self.r_front
        l = self.r_fleft
        r = self.r_fright

        # Distance Detection
        self.lidar[FRONT] = min(
            min(min(self.raw_data[f[0][0]:f[0][1]]), min(self.raw_data[f[1][0]:f[1][1]])), 10)      # front 36 degrees
        
        self.lidar[FRONT_LEFT] = min(
            min(self.raw_data[l[0]:l[1]]), 10)      # front left 36 degrees
        
        self.lidar[FRONT_RIGHT] = min(
            min(self.raw_data[r[0]:r[1]]), 10)      # front right 36 degrees

    def shutdown_ops(self):
        self.robot_controller.stop()
        self.ctrl_c = True

    def attempt_avoidance(self):
        front = self.lidar[FRONT]
        fleft = self.lidar[FRONT_LEFT]
        fright = self.lidar[FRONT_RIGHT]

        t = self.rldist_thresh
        degrees = 0

        # If we're too close to the object right in front of us
        if front < self.fdist_thresh:
            self.robot_controller.stop()
            if fright > fleft:
                degrees = 25
            else:
                degrees = -25
    
            self.robot_controller.deg_rotate(degrees)
        
        #print("F: {}, L: {}, R; {}".format(front, fleft, fright))


        # if front > self.fdist_thresh:
        #     if fright > t and fleft < t:
        #         self.robot_controller.set_move_cmd(linear=0.025, angular=-0.4)
        #     elif fright < t and fleft > t:
        #         self.robot_controller.set_move_cmd(linear=0.025, angular=0.4)
        #     elif fright < t and fleft < t:
        #         self.robot_controller.set_move_cmd(linear=0.025, angular=-0.4)
        #     else:
        #         self.robot_controller.set_move_cmd(linear=0.12, angular=0)
        # else:
        #     if fright > t and fleft > t:
        #         self.robot_controller.set_move_cmd(linear=0.0, angular=-0.4)
        #     elif fright > t and fleft < t:
        #         self.robot_controller.set_move_cmd(linear=0.0, angular=-0.4)
        #     elif fright < t and fleft > t:
        #         self.robot_controller.set_move_cmd(linear=0.0, angular=0.4)
        #     else:
        #         self.robot_controller.set_move_cmd(linear=0.0, angular=-0.4)

        # self.robot_controller.set_move_cmd(linear=linear, angular=angular)
        # self.robot_controller.publish()

    def action_launcher(self):
        # set the robot velocity:
        self.robot_controller.set_move_cmd(linear=0)

        while not rospy.is_shutdown():
            self.attempt_avoidance()

            


if __name__ == '__main__':
    oa = ObstacleAvoidance()
    try:
        oa.action_launcher()
    except rospy.ROSInterruptException:
        pass
