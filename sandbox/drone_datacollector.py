#!/usr/bin/env python3

import sys, time
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion, TwistStamped
from nav_msgs.msg import Odometry
import numpy as np 
import pandas as pd 
import matplotlib.pyplot as plt

class Datacollector:
    
    def __init__(self):
        rospy.init_node("Datacollector", anonymous=True, disable_signals=True)
        self.sub_state_gt = rospy.Subscriber("/autopilot/vicon_reference", Odometry, self.state_gt_cb) # 100hz
        self.sub_pos_px4 = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pos_px4_cb) # 30 hz
        self.sub_vel_px4 = rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, self.vel_px4_cb) # 30hz
        self.sub_ref = rospy.Subscriber("/autopilot/odometry_ref", Odometry, self.reference_cb) # 30 hz

        self.ground_truth_pos = np.empty((4,1))
        self.px4_pos = np.empty((4,1))
        self.px4_vel = np.empty((7,1))
        self.target = np.empty((4,1))

        
    def state_gt_cb(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        t = rospy.get_rostime()
        self.ground_truth_pos = np.column_stack((self.ground_truth_pos, np.array([[x],[y],[z],[t]])))

    def pos_px4_cb(self, data):
        x = data.pose.position.x
        y = data.pose.position.y
        z = data.pose.position.z
        t = rospy.get_rostime()
        self.px4_pos = np.column_stack((self.px4_pos, np.array([[x],[y],[z],[t]])))

    def vel_px4_cb(self, data):
        x = data.twist.linear.x
        y = data.twist.linear.y
        z = data.twist.linear.z
        p = data.twist.angular.x
        q = data.twist.angular.y
        r = data.twist.angular.z
        t = rospy.get_rostime()
        self.px4_vel = np.column_stack((self.px4_vel, np.array([[x],[y],[z],[p],[q],[r],[t]])))

    def reference_cb(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        t = rospy.get_rostime()
        self.target = np.column_stack((self.target, np.array([[x],[y],[z],[t]])))



def main(argv):
    dp = Datacollector()

    try:
        while 1:
            time.sleep(1)
    except KeyboardInterrupt:
        print("saving")
        print(dp.target.shape)
        np.save('ground_truth_pos_1', dp.ground_truth_pos[:,1:])
        np.save('px4_pos_1', dp.px4_pos[:,1:])
        np.save('px4_vel_1', dp.px4_vel[:,1:])
        np.save('target_1', dp.target[:,1:])


if __name__ == '__main__':
    main(sys.argv)