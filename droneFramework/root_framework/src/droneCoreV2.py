#!/usr/bin/env python

import rospy

import mavros as mav
import mavros.utils
import mavros.command as mavCMD
import mavros.setpoint as mavSP

import mavros_msgs.msg
import mavros_msgs.srv

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import (String, Int8, Float64, Bool)
# important properties, do not mess with them!
mavros.set_namespace()

onB_StateSub = '/onboard/state'
mh_enableSub = '/onboard/enableMH'
keySub = '/gcs/keypress'
isolatedSub = '/onboard/isolated/llp_enable'

class droneCore():
    def __init__(self):
        rospy.init_node('D4Emaster_node')
        self.rate = rospy.Rate(20)
        # important variables 
        self.MH_enabled = False
        self.sysState = 'idle' 
        self.MAVROS_State = mavros_msgs.msg.State()
        self.isAirbourne = False
        
        self.uavGPSPos = None
        self.uavLocalPos = mavSP.PoseStamped()
        self.uavHdg = None  

        #"killSwitch"
        rospy.Subscriber(isolatedSub, Bool, self.onKillSwitch)

        # Subscribers 
        rospy.Subscriber(
            mavros.get_topic('state'),
            mavros_msgs.msg.State,
            self._cb_uavState)
        
        rospy.Subscriber(
            mavros.get_topic('global_position','global'),
            NavSatFix, 
            self._cb_SatFix)

        rospy.Subscriber(
            mavros.get_topic('global_position', 'compass_hdg'),
            Float64, 
            self._cb_headingUpdate)

        rospy.Subscriber(
            keySub,
            Int8,
            self._cb_onKeypress)

        # Publishers
        self.statePub = rospy.Publisher(
            onB_StateSub, 
            String,
            queue_size=1)

        self.enableMHPub = rospy.Publisher(
            mh_enableSub,
            Bool,
            queue_size=1
        )
        
        self.llpPub = rospy.Publisher(isolatedSub, Bool, queue_size=1)
        self.spLocalPub = mavSP.get_pub_position_local(queue_size=5)

        # Services
        self.setMode = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        self.enableTakeoff = rospy.ServiceProxy('/ mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
        # Message members   


        self._mavrosHandshake()

    # Core Functions 
    def _mavrosHandshake(self):
        rospy.loginfo('DroneCore: Waiting for MAVROS Connection.')
        i=0
        time = rospy.Time.now()
        for i in range(0,3):
            print'.',
            if self.MAVROS_State.connected:
                rospy.loginfo("DroneCore: MAVROS Connected!")
                break
            rospy.sleep(1)
        if not self.MAVROS_State.connected:
            errorMsg = "MAVROS not connected!"
            rospy.logfatal(errorMsg)
            rospy.signal_shutdown(errorMsg)
    
    def _pubMsg(self, msg, topic):
        msg.header = mavSP.Header(
            frame_id="att_pose",
            stamp=rospy.Time.now())

        topic.publish(msg)
        self.rate.sleep()

    def _setState(self, state):
        self.sysState = state
        if state == 'idle':
            self.enableMHPub.publish(False)
        elif not self.MH_enabled:
            self.enableMHPub.publish(True)
            self.MH_enabled = True
        self.statePub.publish(state)
    # Subscriber callbacks

    def onKillSwitch(self,msg):
        if msg.data==True:
            self.enableMHPub.publish(False)
            self.MH_enabled = False
            self.statePub.publish('isolate')

    def _cb_onKeypress(self, msg):
        keypress = str(chr(msg.data))
        keypress.lower()
    
        if keypress == 'o':
            self.enableMHPub.publish(True) # send messages to enable offboard
            self._setState('loiter')
            rospy.loginfo_once("DroneCore: Enabling Offboard")
            for i in range(0,3):
                resp = self.setMode(0,'OFFBOARD')
                if self.MAVROS_State.mode == 'OFFBOARD':
                    break
            
            # print(resp)

        if keypress == 'i':
            rospy.loginfo_once('LidarLandingEnabled')
            self._setState('lidar_land')

        if keypress == 'l':
            if self.MAVROS_State.mode != 'OFFBOARD':
                rospy.logwarn('DroneCore: OFFBOARD not enabled')
            else:
                rospy.loginfo_once("loiterPilot Enabled")
                self._setState('loiter')
        if keypress == 'p':
            rospy.logwarn('enabling iROS pilot!')
            self.llpPub.publish(True)
        # if keypress == 'r':
        #     rospy.logwarn_once('DroneCore: terminate pressed!, disabling MessageHandler')
        #     self.setMode(0,'AUTO.LOITER')
        #     self.enableMHPub.publish(False)

        if keypress == 't':
            self.droneTakeoff()
            if self.sysState == 'takeoff':
                self._setState('loiter')
  

    def _cb_SatFix(self, msg):
        self.uavGPSPos = msg
    
    def _cb_headingUpdate(self,msg):
        self.uavHdg = msg
    
    def _cb_uavState(self, msg):
        self.MAVROS_State = msg
        # if self.sysState != 'idle' and self.MAVROS_State != 'OFFBOARD':
        #     rospy.logwarn("DroneCore: System enabled, but drone is in manual control. Disabling Message Handler")
        #     self._setState('idle')
        pass

    # Functions:
    def droneTakeoff(self, alt=1.0):
        if self.isAirbourne == False or self.sysState == 'idle':
            if not self.MAVROS_State.armed:
                mavCMD.arming(True)

            preArmMsgs = self.uavLocalPos
            preArmMsgs.pose.position.z = 1.5

            for i in range(50):
                self._pubMsg(preArmMsgs, self.spLocalPub)
            self._setState('takeoff')
            self.setMode(0, 'OFFBOARD')

            self.isAirbourne = True
            self.enableMHPub.publish(self.isAirbourne)
            #wait until takeoff has occurred
            while(self.uavLocalPos.pose.position.z <= (preArmMsgs.pose.position.z-0.25)):
                self._pubMsg(preArmMsgs, self.spLocalPub)


    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()
        pass

if __name__ == "__main__":
    dcObj = droneCore()
    dcObj.run()
