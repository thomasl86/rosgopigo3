#!/usr/bin/env python

# GoPiGo3 imports
from di_sensors import distance_sensor
import gopigo3
import time
# ROS imports
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan

# Instantiate GoPiGo object
objGpg = gopigo3.GoPiGo3()
# Instantiate DistanceSensor object
objDstSns = distance_sensor.DistanceSensor()


def laserscanner():

    pubScan = rospy.Publisher('laserscan', LaserScan, queue_size=1)
    pubServoPos = rospy.Publisher('servo/cmd_pos', Float64, queue_size=1)
    rospy.init_node('laserscanner', anonymous=True)

    dtSweep = float(rospy.get_param('~dt_sweep', 7.2))
    rate_hz = float(rospy.get_param('~rate', 20))
    doSweep = rospy.get_param('~do_sweep', bool(1))
    angleSweep = rospy.get_param('~angle_sweep', 180.0)
    rospy.loginfo("rate_hz = %s", rate_hz)
    amntSteps = rate_hz*dtSweep
    dtStep = dtSweep/amntSteps
    rospy.loginfo("dtStep = %s", dtStep)
    dtSleep = rospy.Duration(dtStep)
    rateStep = rospy.Rate(1/dtStep)
    rospy.loginfo("rateStep = %f", 1/dtStep)

    stepSweep = float(angleSweep)/amntSteps

    objRate = rospy.Rate(rate_hz)

    # Variables
    sweepDir = 1
    pi = 3.14159
    toRad = pi/180.0
    toDeg = 180.0/pi

    time_pre = rospy.Time.now()

    while not rospy.is_shutdown():

        # --- Obtain a scan
        # --- Populate the LaserScan message
        numReadings = angleSweep / abs(stepSweep)
        now = rospy.get_rostime()
        msgScan = LaserScan()
        msgScan.header.stamp = now
        msgScan.header.frame_id = 'base_laser'
        msgScan.range_min = 0.010
        msgScan.range_max = 2
        msgScan.ranges = []
        msgScan.intensities = []
        if doSweep:
            msgScan.time_increment = dtStep
            msgScan.angle_increment = ((angleSweep*toRad) / numReadings) * sweepDir
            msgScan.angle_min = (angleSweep*toRad)/2 * sweepDir * -1
            msgScan.angle_max = (angleSweep*toRad)/2 * sweepDir
            # --- Start and end index based on direction of sweep
            if sweepDir > 0:
                angleStart = -float(angleSweep)/2
                ii = range(1, int(amntSteps) + 1, 1)
            elif sweepDir < 0:
                angleStart = float(angleSweep)/2
                ii = range(int(amntSteps) + 1, 1, -1)

            # Carry out the sweep
            angle = angleStart
            for i in ii:
                tic = rospy.Time.now()
                if rospy.is_shutdown():
                    break
                angle = angle + stepSweep*sweepDir
                pubServoPos.publish(angle)

                msgScan.ranges.append(objDstSns.read_range_single() / 1000.0)

                rateStep.sleep()

            # Invert sign of sweepDir to signal sweep direction change
            sweepDir = sweepDir * -1
        else:
            msgScan.time_increment = dtStep
            msgScan.angle_increment = 0.2
            msgScan.angle_min = -0.1
            msgScan.angle_max = 0.1

            dist = objDstSns.read_range_single() / 1000.0
            msgScan.ranges.append(dist)
            msgScan.ranges.append(dist)

            rateStep.sleep()

        # Publish the scan data
        pubScan.publish(msgScan)

    objGpg.reset_all()


if __name__ == '__main__':
    try:
        laserscanner()
    except rospy.ROSInterruptException:
        objGpg.reset_all()
        pass
