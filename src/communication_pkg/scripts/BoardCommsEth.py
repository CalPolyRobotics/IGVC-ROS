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

    ip = '192.168.0.10'
    port = 9485

    if len(sys.argv) == 3:
        ip = sys.argv[1]
        port = sys.argv[2]
    elif len(sys.argv) != 1:
        print 'usage: rosrun communication_pkg BoardCommsEth.py [ip_addr port]'

    comms_handler = CommsHandler(ip=ip, port=port)

    init_subscribers(comms_handler)

    comms_handler.run()
