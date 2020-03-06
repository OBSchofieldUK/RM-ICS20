#!/usr/bin/env python
from math import (degrees, radians, floor, isinf)

import rospy
import rospkg

from sensor_msgs.msg import LaserScan, Range

_VU8_100_Sub = '/LiDAR/VU8_100Deg/scan_raw'
_VU8_48_Sub = '/LiDAR/VU8_48Deg/scan_raw'
_LidarLite_sub = '/mavros/distance_sensor/lidarlite_pub_raw'

VU8_100_Conv_Pub = '/LiDAR/VU8_100Deg/laserscan'
VU8_48_Conv_Pub = '/LiDAR/VU8_48Deg/laserscan'
LidarLite_Pub = '/mavros/distance_sensor/lidarlite_pub'


class gazeboLeddarSim(object):
    def __init__(self):
        rospy.init_node('simConvNode')
        self.vu8Channels = []
        self.VU8Segments = 8

        self.VU8_48_Pub = rospy.Publisher(VU8_48_Conv_Pub, LaserScan, queue_size=5)
        self.VU8_100_Pub = rospy.Publisher(VU8_100_Conv_Pub, LaserScan, queue_size=5)
        self.LidarLitePub = rospy.Publisher(LidarLite_Pub, Range, queue_size=10)
        
        rospy.Subscriber(_VU8_100_Sub, LaserScan, self.onVU8Update)
        rospy.Subscriber(_VU8_48_Sub, LaserScan, self.onVU8Update)
        rospy.Subscriber(_LidarLite_sub, LaserScan, self.onLidarLiteUpdate)

        self.rate = rospy.Rate(20)

        print("NodeReady")
        # rospy.spin()

    def onVU8Update(self, msg):
        inMsg = msg
        outMsg = inMsg

        # outMsg.header = inMsg.header
        # outMsg.scan_time = inMsg.scan_time
        # outMsg.angle_increment = inMsg.angle_increment
        # outMsg.angle_max = inMsg.angle_max
        # outMsg.angle_min = inMsg.angle_min

        LiDAR_FOV = degrees(inMsg.angle_max - inMsg.angle_min)
        LiDAR_segInc = degrees(inMsg.angle_increment)
        inputSamples = len(inMsg.ranges)
        samplePerSeg = inputSamples/self.VU8Segments

        outMsg.angle_increment = inMsg.angle_increment * samplePerSeg
        # print("Input samples:", inputSamples)
        # print("LidarRange:", int(LiDAR_FOV))
        # print("Samples per segment: %d, degrees perSample %.2f" %
        #       (samplePerSeg, LiDAR_segInc))
        vu8Ranges = []
        vu8Amplitudes = []


        for seg in range(0, self.VU8Segments):
            nearestDistRng = 99999999.0
            nearestDistAmp = 0.0
            for i in range(0, samplePerSeg):
                # print("Sample: ",i+seg*samplePerSeg)
                currSampleRng = inMsg.ranges[i+seg*samplePerSeg]
                currSampleAmp = inMsg.intensities[i+seg*samplePerSeg]

                if not isinf(currSampleRng):
                    if currSampleRng < nearestDistRng:
                        nearestDistRng = currSampleRng
                        nearestDistAmp = currSampleAmp

            if nearestDistRng > 99999998.0:
                nearestDistRng = float('Inf')
            vu8Ranges.append(nearestDistRng)
            vu8Amplitudes.append(nearestDistAmp)

        outMsg.ranges = vu8Ranges
        outMsg.intensities = vu8Amplitudes

        if int(LiDAR_FOV) == 100:
            self.VU8_100_Pub.publish(outMsg)
        elif int(LiDAR_FOV) == 48:
            self.VU8_48_Pub.publish(outMsg)
        else:
            msg = (
                "simLeddar - sensor neither 48 or 100 degrees. FOV: %.2f" % LiDAR_FOV)
            rospy.logwarn_once(msg)
        pass

    def onLidarLiteUpdate(self, msg):
        inMsg = msg
        outMsg = Range()
        outMsg.header = inMsg.header
        outMsg.min_range = inMsg.range_min
        outMsg.max_range = inMsg.range_max

        outMsg.range = msg.ranges[1]
        self.LidarLitePub.publish(outMsg)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    ls = gazeboLeddarSim()
    ls.run()
