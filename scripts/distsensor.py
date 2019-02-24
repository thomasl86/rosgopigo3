#!/usr/bin/env python

# GoPiGo3 imports
from di_sensors import distance_sensor
import gopigo3

# ROS imports
import rospy
from std_msgs.msg import Float64

# Instantiate GoPiGo object
#objGpg = gopigo3.GoPiGo3()
# Instantiate DistanceSensor object
objDstSns = distance_sensor.DistanceSensor()


def rosgopigo3_distsensor():

    rospy.init_node('distsensor', anonymous=True)
    rate_hz = float(rospy.get_param('~rate', 20))

    pubDistance = rospy.Publisher('distance', Float64, queue_size=1)

    while not rospy.is_shutdown():
        pubDistance.publish(objDstSns.read_range_single() / 1000.0)


if __name__ == '__main__':
    try:
        rosgopigo3_distsensor()
    except rospy.ROSInterruptException:
        #objGpg.reset_all()
        pass
