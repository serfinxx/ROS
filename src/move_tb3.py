#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math

class MoveTB3(object):
    def __init__(self):
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.publisher_rate = rospy.Rate(10) # Hz
        self.vel_cmd = Twist()
    
    def publish(self):
        self.publisher.publish(self.vel_cmd)

    def set_move_cmd(self, linear = 0.0, angular = 0.0):
        self.vel_cmd.linear.x = linear
        self.vel_cmd.angular.z = angular

    def get_move_cmd(self):
        return self.vel_cmd.linear.x, self.vel_cmd.angular.z

        
    def deg_rotate(self, degree):
        # rospy.sleep(1)
        linear, angular = self.get_move_cmd()
        speed = 0.8
        t = math.radians(abs(degree)) / speed
        if degree < 0:
            self.set_move_cmd(0.0, -speed)
        else:
            self.set_move_cmd(0.0, speed)
        
        self.publish()
        rospy.sleep(t)
        self.set_move_cmd(linear, angular)
        self.publish()
    
    def stop(self):
        self.set_move_cmd()
        self.publish()
