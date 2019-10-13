#!/usr/bin/env python

"""
BoardComms

Main Ros node required to allow communication between the golfcart and other rosnodes

usage: rosrun communication_pkg BoardComms.py [ttyACM1]
"""

import sys
from BoardCommsSub import init_subscribers
import rospy

from CommsHandler import CommsHandler

if __name__ == '__main__':
    rospy.init_node('BoardComms', anonymous=True)

    port = '/dev/igvc_comm'
    if len(sys.argv) == 2:
        port = '/dev/' + sys.argv[1]
    elif len(sys.argv) != 1:
        print 'usage: rosrun communication_pkg BoardComms.py [port_path]'

    comms_handler = CommsHandler(port=port, baud=115200)

    init_subscribers(comms_handler)

    comms_handler.run()
