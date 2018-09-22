#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import Imu

def callback(data):
    print data.angular_velocity.x
def listener():
    rospy.init_node('Garbage', anonymous=True)
    rospy.Subscriber('imu/data', Imu, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
