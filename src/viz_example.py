#!/usr/bin/env python2

import rospy
import numpy as np
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from visualization_tools import *

class LinePublisher:
    #the topics to publish and subscribe to
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    WALL_TOPIC = "/wall"
    SIDE = rospy.get_param("wall_follower/side")
    
    def __init__(self):
        #a publisher for our line marker
        self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=1)
        
        #a subscriber to get the laserscan data
        rospy.Subscriber(self.SCAN_TOPIC, numpy_msg(LaserScan), self.laser_callback)
        
    def laser_callback(self, laser_scan):
        # x and y should be points on your detected wall
        # here we are just plotting a parabola as a demo
        # x = np.linspace(-2., 2., num=20)
        # y = np.square(x)
        ranges = np.array(laser_scan.ranges)
        angles = np.array([laser_scan.angle_min+laser_scan.angle_increment*i for i in range(len(ranges))])
        if self.SIDE == -1:
            indices = np.logical_and(angles < np.pi/6.0, angles > -1.0/2*np.pi)
            # indices = np.logical_and(angles < np.pi/3.0, angles > -1.0/2*np.pi)
            # indices = np.logical_and(angles < np.pi/4.0, angles > -1.0/2*np.pi)
            x = np.cos(angles[indices]) * ranges[indices]
            y = np.sin(angles[indices]) * ranges[indices]

            # x_close = (-c*m)/(m**2+1)
            # y_close = m*x_close + c
        # front_ranges = ranges[33:67]
        else:
            indices = np.logical_and(angles < np.pi/2.0, angles > -1.0/6*np.pi)
            # indices = np.logical_and(angles > -np.pi/2.0, angles < 1.0/3*np.pi)
            # x = np.cos(angles[indices]) * ranges[indices]
            # y = np.sin(angles[indices]) * ranges[indices]
            x = np.cos(angles[60:]) * ranges[60:]
            y = np.sin(angles[60:]) * ranges[60:]

        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")


if __name__ == "__main__":
    rospy.init_node("line_publisher")
    line_publisher = LinePublisher()
    rospy.spin()
