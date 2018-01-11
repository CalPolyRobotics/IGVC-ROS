#!/usr/bin/env python

"""
BoardComms

Main Ros node required to allow communication between the golfcart and other rosnodes

usage: rosrun communication_ros BoardComms.py [ttyACM1]
"""

import sys
from BoardCommsSub import init_subscribers
import rospy

from CommsHandler import CommsHandler

if __name__ == '__main__':
    rospy.init_node('BoardComms', anonymous=True)

    port = '/dev/ttyACM0'
    if len(sys.argv) == 2:
        port = '/dev/' + sys.argv[1]
    elif len(sys.argv) != 1:
        print 'usage: rosrun communication_ros BoardComms.py [ttyACM1]'

    comms_handler = CommsHandler(port, 115200)

    init_subscribers(comms_handler)

    comms_handler.run()
