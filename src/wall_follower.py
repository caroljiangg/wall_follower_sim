#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = rospy.get_param("wall_follower/side")
    VELOCITY = rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")

    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here
        self.pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.prev_error = 0
        rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.callback)

    # TODO:
    # Write your callback functions here.
    def callback(self, scan):
        ranges = np.array(scan.ranges)
        angles = np.array([scan.angle_min+scan.angle_increment*i for i in range(len(ranges))])
        if self.SIDE == -1:
            indices = (angles > -np.pi/2) & (angles < np.pi/3) & (ranges < 4)
            if np.sum(indices) == 0:
                indices = (angles > (-np.pi/2)) & (angles < (np.pi/3)) 
        else:
            indices = (angles > -np.pi/3) & (angles < np.pi/2) & (ranges < 4)
            if np.sum(indices) == 0:
                indices = (angles > (-np.pi/3)) & (angles < (np.pi/2)) 
        x = np.cos(angles[indices]) * ranges[indices]
        y = np.sin(angles[indices]) * ranges[indices]

        A = np.vstack([x, np.ones(len(x))]).T
        m, c = np.linalg.lstsq(A, y)[0]
        distance = abs(c)/np.sqrt(m**2 + 1)
           
        error = self.SIDE * (distance - self.DESIRED_DISTANCE)
        
        kp = 4
        kd = 1.25

        if np.min(ranges) < 0.2:
            steering_angle = 0.0
        elif error > 1:
            steering_angle = self.SIDE*0.34
        else:
            error_derivative = error - self.prev_error
            angle_error = np.arctan2(m, 1)
            steering_angle = kp*error + kd*error_derivative + angle_error

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.steering_angle_velocity = 0
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.acceleration = 0
        drive_msg.drive.jerk = 0

        self.prev_error = error
        self.pub.publish(drive_msg)

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
