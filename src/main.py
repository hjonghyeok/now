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
import cv2


# port = /dev/ttyUSB0
capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640) 
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) 
histogram = None

terminal = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 15, 0.5)


M1_motor = 0
M2_motor = 0
velocity = 0
steering = 0
breakcontrol = 1
gear = 0
MAX_Velocity = 35
MAX_Steering = 30
Stering = ""
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
    global velocity,steering,breakcontrol,gear,M1_motor,M2_motor, histogram, Stering, degree, ranges
    rospy.loginfo("Starting teleop node")
    # rospy.init_node('rplidarNode')
    rospy.init_node('teleop', anonymous=True)
    # rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)
    rate = rospy.Rate(10) # 10hz
#    try:
    status = 0
    while not rospy.is_shutdown():
        # sub = rospy.Subscriber('/scan', LaserScan, callback)
        ret, frame = capture.read()
        if not ret:
            break
        draw = frame.copy()
        # key = getkey()
        if histogram is not None:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            dst = cv2.calcBackProject([hsv], [0], histogram, [0,180], 1)
            ret, (x,y,w,h) = cv2.meanShift(dst, (x,y,w,h), terminal)
            cv2.rectangle(draw,(x,y), (x+w, y+h), (0,255,0), 2)
            XX = (x+x+w)/2
            YY = (y+y+h)/2
            cv2.putText(draw, "x = " + str(XX), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 1, cv2.LINE_AA)
            cv2.putText(draw, "y = " + str(YY), (10,60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 1, cv2.LINE_AA)
            if XX > W/2+100:
                # print("right")
                Stering = "left"
            elif XX < W/2-100:
                Stering = "right"
            else :
                Stering = "str"
            cv2.putText(draw, "target", (x,y-15), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 1, cv2.LINE_AA)
                # print("left")
            # result = np.hstack((draw, cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)))
            result = draw
        else :
            # cv2.putText(draw, "Target", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 1, cv2.LINE_AA)
            result = draw

        cv2.imshow("MeanShift Tracking", result)
        W = result.shape[1]
        H = result.shape[0]
            
        key = cv2.waitKey(10) & 0xff
        if key == 27:
            break
        elif key == ord(' '):
            M1_motor = 0 
            M2_motor = 0
            x,y,w,h = cv2.selectROI("MeanShift Tracking", frame, False)
            if w and h :
                roi = frame[y:y+h, x:x+w]
                roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                histogram = cv2.calcHist([roi], [0], None, [180], [0,180])
                cv2.normalize(histogram, histogram, 0, 255, cv2.NORM_MINMAX)
            else:
                histogram = None
        degree += 180
        ranges *= 100

        pubmsg = Twist()
        if Stering == "right":
            M1_motor -= 20
            M2_motor += 20
        elif Stering == "left":
            M1_motor += 20
            M2_motor -= 20
        elif Stering == "str":
            M1_motor = 0 
            M2_motor = 0
        

        # else:
        #      #if (degree <= 20) or ((degree >= 350) and (degree <= 360)):
        #     print(degree, ranges, M1_motor, M2_motor, type(ranges))
        #     if (ranges >= 50) and (ranges <= 10000):
            
        #         M2_motor += 10  
        #         if M1_motor >= 200:
        #             M1_motor = 200   
        #         if M2_motor >= 200:
        #             M2_motor = 200   
        #     else:
        #         M1_motor -= 10
        #         M2_motor -= 10
        #         if M1_motor <= 0:
        #             M1_motor = 0   
        #         if M2_motor <= 0:
        #             M2_motor = 0   

        # M1_motor = 0
        # M2_motor = 0   
        #time.sleep(0.1)
        pubmsg.linear.x = M1_motor
        pubmsg.angular.z = M2_motor
        publisher.publish(pubmsg)
        # print('cmd : ' + str(M1_motor) + ','+ str(M2_motor))
        #rate.sleep()
    capture.release()
    cv2.destroyAllWindows()
    rospy.spin()

if __name__ == '__main__':
    try:
        teleop()
    except rospy.ROSInterruptException: pass
