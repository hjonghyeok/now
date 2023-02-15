#!/usr/bin/env python
import rospy
import sys, select, os
import sys
import tty
import termios
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy
import time


# port = /dev/ttyUSB0

Stering = ""

degree = 0
ranges = 0

publisher = rospy.Publisher('/main', Twist,queue_size=1)
x = 0
y = 0

def scanToPoint(radian, distance):
    global x, y
    x = distance * numpy.cos(radian)
    y = distance * numpy.sin(radian)
    return x, y

def callback(data):
    global x, y, degree, ranges
    count = int(data.scan_time / data.time_increment)
    for i in range(0, count, 1):
        degree = numpy.rad2deg(data.angle_min + data.angle_increment * i)
        radian = data.angle_min + data.angle_increment * i
        scanToPoint(radian, data.ranges[i])
        ranges = data.ranges[i]

def teleop():
    global Stering, degree, ranges
    rospy.loginfo("Starting teleop node")
    # rospy.init_node('rplidarNode')
    rospy.init_node('teleop', anonymous=True)
    # rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        sub = rospy.Subscriber('/scan', LaserScan, callback)
        degree += 180
        ranges *= 100

        pubmsg = Twist()

        #time.sleep(0.1)

        publisher.publish(pubmsg)
        # print('cmd : ' + str(M1_motor) + ','+ str(M2_motor))
        #rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass
