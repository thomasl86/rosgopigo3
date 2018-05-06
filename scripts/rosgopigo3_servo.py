#!/usr/bin/env python

import gopigo3
import time
# ROS imports
import rospy
from std_msgs.msg import Int16

# Instantiate GoPiGo object
gObjGpg = gopigo3.GoPiGo3()
SERVO_1 = 1
SERVO_2 = 2
gServo = SERVO_1
# Maximum count to the left
gCountMax = 2420
# Minimum count to the right
gCountMin = 620
gCount = 2420/2


def callbackCmdPos(msg):
    gCount = msg.data
    if gCount > gCountMax:
        gCount = gCountMax
    if gCount < gCountMin:
        gCount = gCountMin

    if gServo == SERVO_1:
        gObjGpg.set_servo(gObjGpg.SERVO_1, gCount)
    elif gServo == SERVO_2:
        gObjGpg.set_servo(gObjGpg.SERVO_2, gCount)


def rosgopigo3_servo():
    rospy.init_node('rosgopigo3_servo', anonymous=True)
    rospy.Subscriber('servo/cmd_pos', Int16, callbackCmdPos)
    #pubPos = rospy.publisher('servo/pos', Int16, queue_size=1)

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
        rosgopigo3_servo()
    except rospy.ROSInterruptException:
        gObjGpg.reset_all()
        pass
