#!/usr/bin/env python

import rospy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

rospy.init_node('sim_clock', disable_rostime=True)
pub = rospy.Publisher('/clock', Clock, queue_size=10)
r = rospy.Rate(30)
print "Simulation Clock node initialized"

while not rospy.is_shutdown():
    t = rospy.Time.now()
    msg = Clock()
    msg.clock = t
    print t.to_sec()
    pub.publish(msg)
    r.sleep()
