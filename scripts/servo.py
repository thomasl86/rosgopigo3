#!/usr/bin/env python

import gopigo3
import time
# ROS imports
import rospy
from std_msgs.msg import Float64
from rosgopigo3.srv import *

# Instantiate GoPiGo object
gObjGpg = gopigo3.GoPiGo3()
SERVO_1 = 1
SERVO_2 = 2
gServo = SERVO_1
# Maximum count to the left
gCountMax = 2420
# Minimum count to the right
gCountMin = 620

def setPosition(angle):
    count = int(angle*10) + 900 + gCountMin
    if count > gCountMax:
        rospy.logwarn("Desired angle %2.1f exceeds the range [-90, 90].", angle)
        count = gCountMax
    if count < gCountMin:
        rospy.logwarn("Desired angle %2.1f exceeds the range [-90, 90].", angle)
        count = gCountMin

    if gServo == SERVO_1:
        gObjGpg.set_servo(gObjGpg.SERVO_1, count)
    elif gServo == SERVO_2:
        gObjGpg.set_servo(gObjGpg.SERVO_2, count)


def srvCallbackSetPosition(req):
    setPosition(req.count)
    return ServoPosResponse(1)


def callbackCmdPos(msg):
    setPosition(msg.data)


def servo():
    rospy.init_node('servo', anonymous=True)
    rospy.Subscriber('servo/cmd_pos', Float64, callbackCmdPos)
    #pubPos = rospy.publisher('servo/pos', Int16, queue_size=1)
    srvSetPos = rospy.Service('servo/set_pos', ServoPos, srvCallbackSetPosition)

    rate_hz = rospy.get_param("servo/rate", 30)
    objRate = rospy.Rate(rate_hz)  # 10hz
    gServo = rospy.get_param("servo/servo_id", SERVO_1)
    if gServo > 2 or gServo < 1:
        rospy.logerr("Servo #"+gServo+" not available.")

    while not rospy.is_shutdown():
        rospy.spin()
        #pubPos.publish()
        #objRate.sleep()


if __name__ == '__main__':
    try:
        servo()
    except rospy.ROSInterruptException:
        gObjGpg.reset_all()
        pass
