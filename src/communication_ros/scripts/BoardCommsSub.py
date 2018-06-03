#!/usr/bin/env python

"""
BoardCommsSub
"""

import sys
import rospy

from Message import Message
from std_msgs.msg import UInt8, UInt16, Empty
from MESSAGES import MTYPE

this = sys.modules[__name__]
this.comms_handler = None

def init_subscribers(handler):
    """
    Initialize all the subscribers for BoardComms
    """
    this.comms_handler = handler

    rospy.Subscriber('Set_FNR', UInt8, set_fnr_callback)
    rospy.Subscriber('Set_Speed', UInt16, set_speed_callback)
    rospy.Subscriber('Set_Steering', UInt16, set_steering_callback)
    rospy.Subscriber('Set_Speed', UInt16, set_speed_callback)
    rospy.Subscriber('Set_Brake', UInt16, set_brake_callback)
    rospy.Subscriber('Set_Leds', UInt16, set_leds_callback)
    rospy.Subscriber('Set_Turn_Signal', UInt16, set_turn_signal_callback)
    rospy.Subscriber('Set_Headlights', UInt16, set_headlights_callback)
    rospy.Subscriber('Set_Misc_Lights', UInt16, set_misc_lights_callback)
    rospy.Subscriber('Kill', Empty, kill_callback)

def status_callback(data):
    """
    Callback for status message
    data Uint8MultiArray - Array of data containing status of master and slave devices
    """
    data = bytearray(data.data)
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['echo'], data=data))

def set_fnr_callback(data):
    """
    Callback for set fnr message
    data UInt8 - FNR state for Golf Cart """
    data = bytearray([data.data])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_fnr'], data=data))

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

def set_brake_callback(data):
    """
    Callback for the set brake message
    data UInt16 - Brake value for Golf Cart
    """
    data = bytearray([data.data & 0xFF, (data.data >> 8) & 0xFF])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_brake'], data=data))

def set_leds_callback(data):
    """
    Callback for the set leds message
    data UInt16 - light directive
    """
    data = bytearray([(data.data >> 8) & 0xFF, data.data & 0xFF])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_leds'], data=data))

def set_turn_signal_callback(data):
    """
    Callback for the set turn signal message
    data UInt8 - turn signal directive
    """
    data = bytearray([data.data])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_turn_signal'], data=data))

def set_headlights_callback(data):
    """
    Callback for the set headlights message
    data UInt16 - headlights directive
    """
    data = bytearray([(data.data >> 8) & 0xFF, data.data & 0xFF])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_headlights'], data=data))

def set_misc_lights_callback(data):
    """
    Callback for the set misc lights message
    data UInt16 - misc lights directive
    """
    data = bytearray([(data.data >> 8) & 0xFF, data.data & 0xFF])
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['set_misc_lights'], data=data))


def kill_callback(data):
    """
    Callback for the kill message
    data Empty - no data
    """
    this.comms_handler.enqueue_message(Message(msg_type=MTYPE['kill']))
