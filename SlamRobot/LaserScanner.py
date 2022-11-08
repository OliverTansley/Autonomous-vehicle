#! /usr/bin/env python

import random

import rospy
import LineDetection.LineDetection as LD
from sensor_msgs.msg import LaserScan


def callback(msg):
    
    LD.LineDetector.make_seed_segments(msg.ranges)
    


def scanENV():
    rospy.init_node("scan_printer")
    rospy.loginfo("Press Ctrl + C to terminate")
    scanCollector = rospy.Subscriber("/scan",LaserScan,callback)
    rospy.spin()

if __name__ == "__main__":
    scanENV()