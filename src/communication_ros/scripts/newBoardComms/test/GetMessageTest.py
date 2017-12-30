#!/usr/bin/env python
"""
GetMessagesTest

Tests that the get messages output the proper data
"""

import rospy
from test.Test import Test
from time import sleep
from std_msgs.msg import UInt8, UInt8MultiArray, UInt16MultiArray, UInt16  

rospy.init_node('TestBoardComms', anonymous=True)

status_sub = rospy.Subscriber('Status', UInt8, assert_status)
FNR_sub = rospy.Subscriber('Get_FNR', UInt8, assert_FNR)
speed_sub = rospy.Subscriber('Get_Speed', UInt16MultiArray, assert_speed)
steering_sub = rospy.Subscriber('Get_Steering', UInt16, assert_steer)
battery_sub = rospy.Subscriber('Get_Battery', UInt16, assert_batt)
power_sub = rospy.Subscriber('Get_Power', UInt16MultiArray, assert_power)

def assert_status(data):
    """
    Validate status data
    """
    status_sub.unregister()
    return False

def assert_FNR(data):
    """
    Validate the data is in range[0, 2]
    """
    FNR_sub.unregister()
    if data.data < 0 or data.data > 2:
        return False

    return True

def assert_speed(data):
    """
    Validate speed is in range[0, 64]
    """
    speed_sub.unregister()
    # Validate speed range [0, 64]
    if data.data < 0 or data.data > 64:
        return False

    return True

def assert_steer(data):
    """
    Validate steer data is in range[0, 65535]
    """
    steering_sub.unregister()
    if data.data < 0 or data.data > 65535:
        return False

    return True

def assert_batt(data):
    """
    Assert that battery data is correct
    TODO
    """
    return False

def assert_power(data):
    """
    Assert that power data is correct
    TODO
    """
    return False

def run_get_message_tests():
    """
    Test that BoardCommms is properly working
    """
    t = Test()
    sleep(2)

    t.run_test("Status", assert_status)
    t.run_test("FNR", assert_FNR)
    t.run_test("Speed", assert_speed)
    t.run_test("Steering", assert_steer)
    t.run_test("Power", assert_power)
    t.run_test("Battery", assert_batt)
