#!/usr/bin/env python

"""
BoardCommsSub
"""

import sys
import rospy
from Packet import Packet
from std_msgs.msg import UInt8, UInt16

this = sys.modules[__name__]
this.comms_handler = None

def init_subscribers(handler):
    """
    Initialize all the subscribers for BoardComms
    """
    this.comms_handler = handler

    rospy.Subscriber('Set_FNR', UInt8, set_fnr_callback)
    rospy.Subscriber('Set_Throttle', UInt16, set_throttle_callback)
    rospy.Subscriber('Set_Steering', UInt16, set_steering_callback)
    rospy.Subscriber('Set_Speed', UInt16, set_speed_callback)
    rospy.Subscriber('Set_Lights', UInt16, set_lights_callback)
    rospy.Subscriber('Stop', UInt8, stop_callback) # Data is irrelevant

def set_fnr_callback(data):
    """
    Callback for set fnr message
    """
    data = bytearray([data.data])
    this.comms_handler.queue_packet(Packet(crc=0x00, msg_type=0x14, seq_num=0x00, data=data))

def set_throttle_callback(data):
    """
    Callback for the set throttle message
    """
    return data

def set_steering_callback(data):
    """
    Callback for the set steering message
    """
    return data

def set_speed_callback(data):
    """
    Callback for the set speed message
    """
    return data

def set_lights_callback(data):
    """
    Callback for the set lights message
    """
    data = bytearray([data.data & 0xFF, (data.data >> 8) & 0xFF])
    this.comms_handler.queue_packet(Packet(crc=0x00, msg_type=0x14, seq_num=0x00, data=data))

def stop_callback(data):
    """
    Callback for the stop message
    """
    return data
