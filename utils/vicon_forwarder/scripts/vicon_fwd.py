#!/usr/bin/env python2

import sys
import rospy
from geometry_msgs.msg import PoseStamped, Transform, Vector3, Quaternion
from nav_msgs.msg import Odometry

class ViconAdapter:
    
    def __init__(self, argv):
        sub_name = str(argv[1])
        seg_name = str(argv[2])
        rospy.init_node("vicon_forwarder", anonymous=True)
        self.sub = rospy.Subscriber("vicon/" + sub_name + "/" + seg_name, Transform, self.vicon_callback)
        self.pub = rospy.Publisher("autopilot/vicon_pose", PoseStamped, queue_size=10)
        self.pub_msg = PoseStamped()
        self.pub_msg.header.frame_id = 'odom'
        self.seq = 0

    def vicon_callback(self, data):
        self.pub_msg.header.seq = self.seq
        self.seq += 1
        self.pub_msg.header.stamp = rospy.get_rostime()
        self.pub_msg.pose.position.x = data.translation.x
        self.pub_msg.pose.position.y = data.translation.y
        self.pub_msg.pose.position.z = data.translation.z
        self.pub_msg.pose.orientation = data.rotation
        self.pub.publish(self.pub_msg)

def main(argv):
    va = ViconAdapter(argv)

    rospy.spin()


if __name__ == '__main__':
    main(sys.argv)