#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import actionlib

# Import all the necessary ROS message types:
from com2009_actions.msg import SearchFeedback, SearchResult, SearchAction
from sensor_msgs.msg import LaserScan

# Import some other modules from within this package (copied from other package)
from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
from math import radians
import datetime as dt
import os
import numpy as np

class obstacle_avoidance_server(object):
    feedback = SearchFeedback() 
    result = SearchResult()

    def __init__(self):
        # Initialise action server
        self.actionserver = actionlib.SimpleActionServer("oa_server", 
            SearchAction, self.action_server_launcher, auto_start=False)
        self.actionserver.start()

        # Lidar subscriber
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'closest': 0.0,
                      'closest angle': 0,
                      'farest angle': 0}

        # Robot movement and odometry
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()
    
    def callback_lidar(self, lidar_data):
        """Returns arrays of lidar data"""

        raw_data = np.array(lidar_data.ranges)

        # Distance Detection
        angle_tolerance = 50
        self.lidar['range'] = min(min(raw_data[:angle_tolerance]),      # closest distance in 60 degree (range)
                               min(raw_data[-angle_tolerance:]))

        self.lidar['closest'] = min(raw_data)       # closest distance in all degrees

        # Angle Detection
        self.lidar['closest angle']=raw_data.argmin()       # angle of closest object in all degrees

        self.lidar['farest angle']=raw_data.argmax()        # angle of farest object in all degrees
    
    def action_server_launcher(self, goal):
        r = rospy.Rate(10)

        success = True
        if goal.fwd_velocity <= 0 or goal.fwd_velocity > 0.26:
            print("Invalid fwd_velocity {:.2f}: 0 < fwd_velocity < 0.26".format(goal.fwd_velocity))
            success = False
        if goal.approach_distance <=0:
            print("approach_distance must be > 0 (requested {:.2f})".format(goal.approach_distance))
            success = False

        if not success:
            self.actionserver.set_aborted()
            return

        print("Request to move at {:.2f} m/s until {:.2f} m away from object".format(
            goal.fwd_velocity, goal.approach_distance))


        # set the robot velocity:
        self.robot_controller.set_move_cmd(linear=goal.fwd_velocity)
        
        # Get the current robot odometry
        ref_x = self.robot_odom.posx
        start_x = self.robot_odom.posx

        ref_y = self.robot_odom.posy
        start_y = self.robot_odom.posy
        distance_travelled = 0.0

        # Before collision
        while self.lidar['range'] > goal.approach_distance:
            if (self.lidar['closest angle'] < 50 or self.lidar['closest angle'] > 310) and self.lidar['range'] < 0.4:
                if self.lidar['closest angle'] > 310:
                    self.robot_controller.set_move_cmd(linear=0.0, angular = 0.3)
                else:
                    self.robot_controller.set_move_cmd(linear=0.0, angular = -0.3)
            else:
                self.robot_controller.set_move_cmd(linear=goal.fwd_velocity, angular = 0)
            self.robot_controller.publish()

            # check if there has been a request to cancel the action mid-way through:
            if self.actionserver.is_preempt_requested():
                rospy.loginfo('Cancelling the movement request.')
                self.actionserver.set_preempted()
                # stop the robot:
                self.robot_controller.stop()
                success = False
                # exit the loop:
                break
            

            # Calculate distance travelled
            distance_travelled = np.sqrt(pow(start_x-ref_x, 2) + pow(start_y-ref_y, 2))
                
            # populate the feedback message and publish it:
            rospy.loginfo('Current distance to object: {:.2f} m'.format(self.lidar['range']))
            rospy.loginfo('Current closest angle to object: {:.2f} degree'.format(self.lidar['closest angle']))
            self.feedback.current_distance_travelled = distance_travelled
            self.actionserver.publish_feedback(self.feedback)

            # update the reference odometry:
            ref_x = self.robot_odom.posx
            ref_y = self.robot_odom.posy

        if success:
            rospy.loginfo('Motion finished successfully.')
            self.result.total_distance_travelled = distance_travelled
            self.result.closest_object_distance = self.lidar['closest']
            self.result.closest_object_angle = self.lidar['closest angle']
            self.actionserver.set_succeeded(self.result)
            self.robot_controller.stop()
            
if __name__ == '__main__':
    rospy.init_node('oa_server')
    obstacle_avoidance_server()
    rospy.spin()
