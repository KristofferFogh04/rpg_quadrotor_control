#!/usr/bin/env python2

PKG = 'px4'

import rospy
import math
import mavros
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, SetModeRequest, CommandBool, CommandBoolRequest
import time
from std_msgs.msg import Header
from threading import Thread

# Constants
STATE_RATE = 20

class mavrosInterface():

    def __init__(self):
        self.current_state = State()
        self.local_pose = None
        self.local_vel = None
        self.keepalive = False
        self.setup()

    def setup(self):

        try:

            # Initialize ROS node
            rospy.init_node('px4keepalive', disable_signals=True)

            #Set up publishers and subscribers
            self.state_rate = rospy.Rate(STATE_RATE)
            self.sub_state = rospy.Subscriber('mavros/state', State,  self._state_callback)
            self.autopilot_sub = rospy.Subscriber("px4_keepalive", Header,  self.autopilot_callback)

            # Connect with flight controller px4:
            while not rospy.is_shutdown() and not self.current_state.connected:
                rospy.Rate(20)
            print("Successfully connected to Flight Controller PX4")
        except KeyboardInterrupt:
            rospy.signal_shutdown("shutting down")


    def _state_callback(self, data):
        self.current_state = data


    def autopilot_callback(self, data):
        if data.frame_id == "arm":
            self.arm(True)

            self.set_mode("OFFBOARD")

            self.keepalive = True

        elif data.frame_id == "disarm":

            self.set_mode("AUTO.LAND")

            while self.current_state.system_status != 3: # 3 is MAV_STATE_ACTIVE
                self.state_rate.sleep()

            self.arm(false)
            self.keepalive = false         


    def set_mode(self, mode):
        if not self.current_state.connected:
            print "No FCU connection"
        
        elif self.current_state.mode == mode:
            pass
        
        else:
            # Request mode change with ros service
            rospy.wait_for_service("mavros/set_mode")
            set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

            req = SetModeRequest()
            req.custom_mode = mode

            while not rospy.is_shutdown() and (self.current_state.mode != req.custom_mode):
                
                try:
                    # request 
                    set_mode.call(req)
                    
                except rospy.ServiceException, e:
                    print "Service did not process request: %s"%str(e)
                    
                else:
                    return True

    def arm(self, do_arming):
        if self.current_state.armed and do_arming:
            pass
            
        else:
            # wait for service
            rospy.wait_for_service("mavros/cmd/arming")    
            
            # service client
            set_arm = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)         
            
            # set request object
            req = CommandBoolRequest()
            req.value = do_arming          
            
            # check response
            if do_arming:
                while not rospy.is_shutdown() and not self.current_state.armed:
                    
                    try:
                        # request 
                        set_arm.call(req)
                        
                    except rospy.ServiceException, e:
                        print "Service did not process request: %s"%str(e)               
                
            else: 
                while not rospy.is_shutdown() and self.current_state.armed:
                    
                    try:
                        # request 
                        set_arm.call(req)
                        
                    except rospy.ServiceException, e:
                        print "Service did not process request: %s"%str(e)
  
    def px4keepalive(self):

        if not rospy.is_shutdown() and self.current_state.connected:

            t0 = rospy.get_time()

            if self.current_state.mode is not "OFFBOARD":
                offboard = self.set_mode("OFFBOARD")
                t0 = rospy.get_time()

            else:
                if not self.current_state.armed and (rospy.get_time() - t0) > 5:
                    self.arm(True)
                t0 = rospy.get_time()



def main():
    mav = mavrosInterface()
    try:        
        while True:
            mav.state_rate.sleep()
            if mav.keepalive == True:
                mav.px4keepalive()
    except KeyboardInterrupt:
        rospy.signal_shutdown("shutting down")

if __name__ == "__main__":
    main()

