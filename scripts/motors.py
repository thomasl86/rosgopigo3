#!/usr/bin/env python

# GoPiGo3 imports
import gopigo3
# ROS imports
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32

# Instantiate GoPiGo3 object
objGpg = gopigo3.GoPiGo3()


def callbackLMotor(msg):
    dataLmotor = float(msg.data)
    objGpg.set_motor_power(objGpg.MOTOR_LEFT, dataLmotor)


def callbackRMotor(msg):
    dataRmotor = float(msg.data)
    objGpg.set_motor_power(objGpg.MOTOR_RIGHT, dataRmotor)


def motors():
    pubEncLeft = rospy.Publisher('lwheel', Int16, queue_size=1)
    pubEncRight = rospy.Publisher('rwheel', Int16, queue_size=1)
    rospy.Subscriber('lmotor_cmd', Float32, callbackLMotor)
    rospy.Subscriber('rmotor_cmd', Float32, callbackRMotor)
    rospy.init_node('motors', anonymous=True)
    rate_hz = rospy.get_param("rate", 30)
    objRate = rospy.Rate(rate_hz)  # 10hz
    # Get the encoder offset?
    # TODO Not sure if the next two lines are needed.
    objGpg.offset_motor_encoder(
        objGpg.MOTOR_LEFT,
        objGpg.get_motor_encoder(
            objGpg.MOTOR_LEFT))
    objGpg.offset_motor_encoder(
        objGpg.MOTOR_RIGHT,
        objGpg.get_motor_encoder(
            objGpg.MOTOR_RIGHT))

    while not rospy.is_shutdown():
        # Read left wheel encoder data
        dataEncLeft = objGpg.get_motor_encoder(objGpg.MOTOR_LEFT)
        # Read right wheel encoder data
        dataEncRight = objGpg.get_motor_encoder(objGpg.MOTOR_RIGHT)
        # Publish the motor encoder data
        pubEncLeft.publish(dataEncLeft)
        pubEncRight.publish(dataEncRight)
        objRate.sleep()

    objGpg.reset_all()


if __name__ == '__main__':
    try:
        motors()
    except rospy.ROSInterruptException:
        objGpg.reset_all()
        pass
