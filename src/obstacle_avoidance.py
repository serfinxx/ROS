#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package (copied from other package)
from move_tb3 import MoveTB3

# Import some other useful Python Modules
from math import radians
import datetime as dt
import os
import numpy as np


class obstacle_avoidance(object):

    def __init__(self):
        # Initialise action server
        rospy.init_node('obstacle_avoidance')
        # Lidar subscriber
        self.lidar_subscriber = rospy.Subscriber(
            '/scan', LaserScan, self.callback_lidar)
        self.lidar = {'front_distance': 0.0,
                      'fleft_distance': 0.0,
                      'fright_distance': 0.0}

        # Robot movement and odometry
        self.robot_controller = MoveTB3()

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)

    def callback_lidar(self, lidar_data):
        """Returns arrays of lidar data"""

        raw_data = np.array(lidar_data.ranges)

        # Distance Detection
        self.lidar['front_distance'] = min(
            min(min(raw_data[342:359]), min(raw_data[0:17])), 10)      # front 36 degrees
        self.lidar['fleft_distance'] = min(
            min(raw_data[306:342]), 10)      # front left 36 degrees
        self.lidar['fright_distance'] = min(
            min(raw_data[18:54]), 10)      # front right 36 degrees


    def shutdown_ops(self):
        self.robot_controller.stop()
        self.ctrl_c = True

    def action_launcher(self):

        # set the robot velocity:
        self.robot_controller.set_move_cmd(linear=0)

        while not rospy.is_shutdown():
            # Action making
            if self.lidar['front_distance'] > 0.3:
                if self.lidar['fleft_distance'] > 0.3 and self.lidar['fright_distance'] < 0.3:
                    self.robot_controller.set_move_cmd(linear=0.025, angular=-0.4)
                elif self.lidar['fleft_distance'] < 0.3 and self.lidar['fright_distance'] > 0.3:
                    self.robot_controller.set_move_cmd(linear=0.025, angular=0.4)
                elif self.lidar['fleft_distance'] < 0.3 and self.lidar['fright_distance'] < 0.3:
                    self.robot_controller.set_move_cmd(linear=0.025, angular=-0.4)
                else:
                    self.robot_controller.set_move_cmd(linear=0.12, angular=0)
            else:
                if self.lidar['fleft_distance'] > 0.3 and self.lidar['fright_distance'] > 0.3:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=-0.4)
                elif self.lidar['fleft_distance'] > 0.3 and self.lidar['fright_distance'] < 0.3:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=-0.4)
                elif self.lidar['fleft_distance'] < 0.3 and self.lidar['fright_distance'] > 0.3:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=0.4)
                else:
                    self.robot_controller.set_move_cmd(linear=0.0, angular=-0.4)
            self.robot_controller.publish()

            


if __name__ == '__main__':
    oa = obstacle_avoidance()
    try:
        oa.action_launcher()
    except rospy.ROSInterruptException:
        pass
