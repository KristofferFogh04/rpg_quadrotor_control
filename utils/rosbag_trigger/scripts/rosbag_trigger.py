#!/usr/bin/env python2

import sys, time
import rospy
import subprocess, shlex
from nav_msgs.msg import Odometry

class RosbagTrigger:
    
    def __init__(self):
        rospy.init_node("RosbagTrigger", anonymous=True)
        self.sub_ref = rospy.Subscriber("autopilot/odometry_ref", Odometry, self.reference_cb) # 30 hz
        self.first = True
        self.command = "rosbag record -o circular1.bag --all --split --size=1024"
        self.command = shlex.split(self.command)
        self.watchdog = 0

    def reference_cb(self,data):
        self.watchdog += 1
        if self.first == True:
            print("Recording rosbag")
            self.first = False
            self.rosbag_proc = subprocess.Popen(self.command)


def main(argv):
    rt = RosbagTrigger()
    old_watchdog = 0
    while True:
        old_watchdog = rt.watchdog
        time.sleep(0.5)
        if rt.watchdog == old_watchdog and rt.first == False:
            rt.rosbag_proc.send_signal(subprocess.signal.SIGINT)
            print("Finished recording rosbag")
            rt.first = True


if __name__ == '__main__':
    main(sys.argv)