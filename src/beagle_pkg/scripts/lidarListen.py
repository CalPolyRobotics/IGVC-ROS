#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray 

def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    interpretRanges(data.data);
    
def listener():
    rospy.init_node('lidar_listener', anonymous=True)

    rospy.Subscriber("ranges", Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def interpretRanges(ranges):
    for ind, distance in enumerate(ranges):
        if ind > 250 and ind < 290:
            if distance < 1:
                print "Too Close"
                break

if __name__ == '__main__':
    listener()
