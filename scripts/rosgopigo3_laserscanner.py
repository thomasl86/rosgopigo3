#!/usr/bin/env python

# GoPiGo3 imports
from di_sensors import distance_sensor
import gopigo3
import time
# ROS imports
import rospy
# TODO Int16 import is now obsolete (I think)
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan

# Instantiate GoPiGo object
objGpg = gopigo3.GoPiGo3()
# Instantiate DistanceSensor object
objDstSns = distance_sensor.DistanceSensor()
# Variables
stepSweep = 10
sweepDir = 1


def scan():
    global stepSweep, sweepDir
    # Maximum count to the left
    countMax = 2420
    # Minimum count to the right
    countMin = 620
    # Populate the LaserScan message
    numReadings = (countMax - countMin) / abs(stepSweep)
    now = rospy.get_rostime()
    msgScan = LaserScan()
    msgScan.header.stamp = now
    msgScan.header.frame_id = 'base_laser'
    msgScan.range_min = 0.010
    msgScan.range_max = 2
    msgScan.time_increment = 0.04
    msgScan.angle_increment = (3.14 / numReadings) * sweepDir
    msgScan.angle_min = 1.57 * sweepDir * -1
    msgScan.angle_max = 1.57 * sweepDir
    msgScan.ranges = []
    msgScan.intensities = []
    msgScan.ranges = []
    # Start and end index based on direction of sweep
    if sweepDir > 0:
        rangeStart = countMin
        rangeEnd = countMax
    elif sweepDir < 0:
        rangeStart = countMax
        rangeEnd = countMin

    # Carry out the sweep
    for i in range(rangeStart, rangeEnd, stepSweep * sweepDir):
        objGpg.set_servo(objGpg.SERVO_1, i)
        msgScan.ranges.append(objDstSns.read_range_single() / 1000.0)

    # Invert sign of sweepDir to signal sweep direction change
    sweepDir = sweepDir * -1

    return msgScan


def rosgopigo3_laserscanner():
    pubScan = rospy.Publisher('laserscan', LaserScan, queue_size=1)
    rospy.init_node('rosgopigo3_laserscanner', anonymous=True)
    # FIXME Rate in [Hz] is really awkward. Change to sweep duration
    #       or just let servo always go at max speed.
    rate_hz = float(rospy.get_param('~rate', 0.1389))
    rospy.loginfo("rate_hz = %s", rate_hz)
    stepSweep = round((0.04*1800)/(1/rate_hz))

    objRate = rospy.Rate(rate_hz)

    while not rospy.is_shutdown():
        rospy.loginfo("stepSweep = %s", stepSweep)
        rospy.loginfo("rate_hz = %s", rate_hz)
        # Obtain a scan
        msgScan = scan()
        # Publish the scan data
        pubScan.publish(msgScan)
        objRate.sleep()

    objGpg.reset_all()


if __name__ == '__main__':
    try:
        rosgopigo3_laserscanner()
    except rospy.ROSInterruptException:
        objGpg.reset_all()
        pass
