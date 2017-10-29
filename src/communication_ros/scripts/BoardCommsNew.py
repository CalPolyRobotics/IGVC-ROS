#!/usr/bin/env python

"""
BoardComms
"""

import sys
import rospy

#from std_msgs.msg import UInt8, UInt16, UInt16MultiArray, UInt8MultiArray

from Packet import CommsHandler, Packet

GET_MESSAGES = [
    Packet(crc=0, msg_type=0x00, seq_num=0x00, data=[]), #Status
    Packet(crc=0, msg_type=0x08, seq_num=0x00, data=[]), #FNR
    #Packet(crc=0, msg_type=0x0E, seq_num=0x00, data=[]), #Speed
    #Packet(crc=0, msg_type=0x16, seq_num=0x00, data=[]), #Battery
    Packet(crc=0, msg_type=0x18, seq_num=0x00, data=[])  #Power
]

def run_process(comms_handler):
    """
    Board Comms process that continually runs until killed to manage communication
    to and from the control board
    """

    gt_idx = 0

    rate = rospy.Rate(1000) # 1000hz

    while not rospy.is_shutdown():
        print comms_handler.send_message(GET_MESSAGES[gt_idx])
        gt_idx = (gt_idx + 1) % len(GET_MESSAGES)

        rate.sleep()

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('BoardComms', anonymous=True)

    if len(sys.argv) > 1:
        run_process(CommsHandler('/dev/' + sys.argv[1], 115200))
    else:
        run_process(CommsHandler('/dev/ttyACM0', 115200))
