#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import degrees

class TB3Odometry(object):
    def odom_cb(self, odom_data):
        orientation = odom_data.pose.pose.orientation
        position = odom_data.pose.pose.position
        (_, _, yaw) = euler_from_quaternion([orientation.x,
            orientation.y, orientation.z, orientation.w],'sxyz')
        
        self.yaw = self.round(degrees(yaw), 4)
        self.posx = self.round(position.x, 4)
        self.posy = self.round(position.y, 4)

        if not self.has_set_inits:
            self.init_posx = self.posx
            self.init_posy = self.posy
        
        # Here, we map out the areas we've been to
        # if we step back into that area we turn around immediately
    
    def __init__(self):
        self.has_set_inits = False
        self.init_posx = 0.0
        self.init_posy = 0.0

        self.posx = 0.0
        self.posy = 0.0
        self.yaw = 0.0
        self.subscriber = rospy.Subscriber('/odom', Odometry, self.odom_cb)

        self.cache_posx = 0.0
        self.cache_posy = 0.0
        self.cache_yaw = 0.0
    
    def round(self, value, precision):
        value = int(value * (10**precision))
        return float(value) / (10**precision)

    def cache_current_data(self):
        self.cache_posx = self.posx
        self.cache_posy = self.posy
        self.cache_yaw = self.yaw
        
