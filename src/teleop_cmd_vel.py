#!/usr/bin/env python
import rospy
import sys, select, os
import sys
import tty
import termios
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import numpy
import time
#import sys, select, os

M1_motor = 0
M2_motor = 0
velocity = 0
steering = 0
breakcontrol = 1
gear = 0
MAX_Velocity = 35
MAX_Steering = 30
MIN_Sttering = -MAX_Steering
MAX_motor = 255

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
    global x, y, degree, ranges, M1_motor, M2_motor
    count = int(data.scan_time / data.time_increment)
    for i in range(0, count, 1):
        degree = numpy.rad2deg(data.angle_min + data.angle_increment * i)
        radian = data.angle_min + data.angle_increment * i
        scanToPoint(radian, data.ranges[i])
        ranges = data.ranges[i]
        degree += 180
        ranges *= 100
        if (degree <= 7) or (degree >= 357):
            print(degree, ranges, M1_motor, M2_motor)
            if ranges >= 20:
                M1_motor += 1
                M2_motor += 1
            else:
                M1_motor = 0
                M2_motor = 0
                
                
        # time.sleep(0.1)

def getkey():
        fd = sys.stdin.fileno()
        original_attributes = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, original_attributes)
        return ch

def teleop():
    global velocity,steering,breakcontrol,gear,M1_motor,M2_motor
    rospy.loginfo("Starting teleop node")
    # rospy.init_node('rplidarNode')
    rospy.init_node('teleop', anonymous=True)
    # rospy.Subscriber("/move_base_simple/goal", 1000, callback)
    rate = rospy.Rate(10) # 10hz
#    try:
    status = 0
    while not rospy.is_shutdown():
        # sub = rospy.Subscriber('/scan', LaserScan, callback)
        key = getkey()
        if key == 'w':
            if M1_motor > M2_motor:
                M2_motor = M1_motor
            else:
                M1_motor = M2_motor
            M1_motor += 10
            M2_motor += 10
            velocity = velocity + 2
            steering = 0 
            status = status + 1
        elif key == 's':
            M1_motor = 0
            M2_motor = 0
            velocity = 0
            #steering = 0
            status = status + 1
        elif key == 'a':
            M1_motor += 10
            M2_motor -= 10
            steering = steering + 2
            status = status + 1
        elif key == 'd':
            M1_motor -= 10
            M2_motor += 10
            steering = steering - 2
            status = status + 1
        elif key == 'x':
            if M1_motor > M2_motor:
                M2_motor = M1_motor
            else:
                M1_motor = M2_motor
            M1_motor -= 10
            M2_motor -= 10
            velocity = velocity - 2
            steering = 0
            status = status + 1
        else:
            if (key == '\x03'):
                break
        pubmsg = Twist()
        if M1_motor >= MAX_motor:
            M1_motor = MAX_motor
        if M1_motor <= -MAX_motor:
            M1_motor = -MAX_motor
        if M2_motor >= MAX_motor:
            M2_motor = MAX_motor
        if M2_motor <= -MAX_motor:
            M2_motor = -MAX_motor

  
        pubmsg.linear.x = M1_motor
        pubmsg.angular.z = M2_motor
        publisher.publish(pubmsg)
        print('cmd : ' + str(M1_motor) + ','+ str(M2_motor))
        #rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass
