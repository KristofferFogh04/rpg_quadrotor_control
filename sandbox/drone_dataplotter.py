#!/usr/bin/env python2

import sys
import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3, Quaternion
from nav_msgs.msg import Odometry
import numpy as np 
import pandas as pd 

class Dataplotter:
    
    def __init__(self, argv):
        rospy.init_node("dataplotter", anonymous=True)
        self.sub_state_gt = rospy.Subscriber("autopilot/vicon_reference", Odometry self.state_gt_cb)
        self.sub_pos_px4 = rospy.Subscriber("mavros/local_position/pose", PoseStamped self.pos_px4_cb)
        self.sub_vel_px4 = rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped self.vel_px4_cb)
        self.sub_ref = rospy.Subscriber("autopilot/odometry_ref", Odometry self.reference_cb)

        


    def state_gt_cb(self, data):

    def pos_px4_cb(self, data):

    def vel_px4_cb(self, data):


    def reference_cb(self, data):



def main(argv):
    va = ViconAdapter(argv)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)