#!/usr/bin/env python

"""
BoardCommsSub
"""

import sys
import rospy

from Message import Message
from std_msgs.msg import UInt8, UInt8MultiArray, UInt16, Empty
from MESSAGES import MTYPE

this = sys.modules[__name__]
this.comms_handler = None

def init_subscribers(handler):
    """
    Initialize all the subscribers for BoardComms
    """
    this.comms_handler = handler

    rospy.Subscriber('Echo', UInt8MultiArray, echo_callback)
    rospy.Subscriber('Set_FNR', UInt8, set_fnr_callback)
    rospy.Subscriber('Set_Throttle', UInt16, set_throttle_callback)
    rospy.Subscriber('Set_Steering', UInt16, set_steering_callback)
    rospy.Subscriber('Set_Speed', UInt16, set_speed_callback)
    rospy.Subscriber('Set_Lights', UInt16, set_lights_callback)
    rospy.Subscriber('Stop', Empty, stop_callback)

def echo_callback(data):
    """
    Callback for echo message
    data Uint8MultiArray - Array of data to be echoed back by the board
    """
    data = bytearray(data.data)
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['echo'], data=data))

def set_fnr_callback(data):
    """
    Callback for set fnr message
    data UInt8 - FNR state for Golf Cart
    """
    data = bytearray([data.data])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_fnr'], data=data))

def set_throttle_callback(data):
    """
    Callback for the set throttle message
    data UInt16 - Throttle Value for Golf Cart
    """
    data = bytearray([data.data & 0xFF, (data.data >> 8) & 0xFF])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_throttle'], data=data))

def set_steering_callback(data):
    """
    Callback for the set steering message
    data UInt16 - Steering Value for Golf Cart
    """
    data = bytearray([data.data & 0xFF, (data.data >> 8) & 0xFF])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_steering'], data=data))

def set_speed_callback(data):
    """
    Callback for the set speed message
    data UInt16 - Speed for Golf Cart
    """
    data = bytearray([data.data & 0xFF, (data.data >> 8) & 0xFF])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_speed'], data=data))

def set_lights_callback(data):
    """
    Callback for the set lights message
    data UInt16 - light directive
    """
    data = bytearray([(data.data >> 8) & 0xFF, data.data & 0xFF])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_lights'], data=data))

def stop_callback(data):
    """
    Callback for the stop message
    data Empty - no data
    """
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['send_stop']))
