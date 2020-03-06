#!/usr/bin/env python

import rospy
from math import pi, radians, degrees, tan, sin, cos 
import mavros 
import mavros.utils as mavUI
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from std_msgs.msg import (String, Int8, Float64)
from tf.transformations import (euler_from_quaternion, quaternion_from_euler)
from geometry_msgs.msg import Quaternion



mavros.set_namespace('mavros')  # needed to operate!

# This is the "master topic" for enabling control
onB_StateSub = '/onboard/state'
onB_substateSub = '/onboard/substate'

# This is the "pilot topic" for desired setpoints
targetWP_GPS = '/onboard/setpoint/exampleGPS'
targetWP_Atti = '/onboard/setpoint/exampleAtti'


class examplePilot():
    def __init__(self):
        rospy.init_node('examplePilot')
        self.subState = 'GPS'
        self.GPSMode = True
        self.enable = False
        self.rate = rospy.Rate(20)

        self.exampleGPSPub      = rospy.Publisher(targetWP_GPS, mavSP.PoseStamped, queue_size=5)
        self.exampleATTIPub = rospy.Publisher(targetWP_Atti, mavros_msgs.msg.AttitudeTarget, queue_size=5)
        self.pilotSubstatePub = rospy.Publisher(onB_substateSub, String, queue_size=1)

        rospy.Subscriber(onB_StateSub, String, self.onStateChange)
        rospy.Subscriber(mavros.get_topic('local_position', 'pose'), mavSP.PoseStamped, self.onPositionChange) # uses mavros.get topic to find the name of the current XYZ position of the drone 
        

        self.currUAVPos = mavSP.PoseStamped()       # used for GPS setpoints
        self.msgSPAtti = mavSP.TwistStamped()       # used for Attitude setpoints

        rospy.loginfo('examplePilot Initialised.')

    # IMPORTANT Function: enables/ disables the pilot.
    # Also has example of substate switching. 

    def onStateChange(self, msg):
        if msg.data == 'example':
            if self.enable:
                # Sub states: in this case, switch between ATTI and GPS Mode 
                if self.GPSMode == True:
                    self.subState = 'ATTI'
                    self.GPSMode = False
                else:
                    self.subState = 'GPS'
                    self.GPSMode = True

                rospy.loginfo('examplePilot enabled, switching to %s mode' % self.subState)
                
            else:
                print('example enabled (GPS Mode)')
            
            self.enable = True
            self.pilotSubstatePub.publish(self.subState)

        else:
            if self.enable:
                print('example disabled')
            self.enable = False

    # Required Function, ensures the outgoing message header has a time of "now"
    def _pubMsg(self, msg, topic):

        if self.subState == 'GPS':
            f_ID = "base_link"
        elif self.subState == 'ATTI':
            f_ID = "att_pose"
        msg.header = mavros.setpoint.Header(
            frame_id=f_ID,
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()
        print "sent: ", msg
    def onPositionChange(self, msg):
        self.currUAVPos = msg
    

    #TODO: Add attitude setpoint functionality
    def generateATTI_Msg(self):
        outMsg = mavros_msgs.msg.AttitudeTarget()
        outMsg.body_rate.x = 20
        outMsg.body_rate.y = 0
        outMsg.body_rate.z = 0
        outMsg.thrust = 0.5
    
        return outMsg

    def run(self):  

        while not rospy.is_shutdown():
            if self.enable:
                if self.subState == 'GPS':
                    self._pubMsg(self.currUAVPos, self.exampleGPSPub)
                elif self.subState == 'ATTI':
                    msg = self.generateATTI_Msg()
                    self._pubMsg(msg, self.exampleATTIPub)

                    # self._pubMsg(self.msgSPAtti, self.exampleSetpointPub)
                self.rate.sleep()


if __name__ == "__main__":
    LP = examplePilot()
    LP.run()
