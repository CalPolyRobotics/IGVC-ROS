#!/usr/bin/env python
import rospy, sys
import math, os, time #might not need these
from std_msgs.msg import UInt8, UInt8MultiArray, UInt16, UInt16MultiArray, Float32MultiArray

print 'gui'

info = {
    "FNR"    : {"name" : "FNR:", "value" : ""},
    "SPDL"   : {"name" : "Speed Left (m/s):", "value": ""},
    "SPDR"   : {"name" : "Speed Right (m/s):", "value": ""},
    "STR"    : {"name" : "Steering:", "value": ""},
    "PDL"    : {"name" : "Pedal:", "value": ""},
    "PWVB"   : {"name" : "Battery (V):", "value": ""},
    "PWAB"   : {"name" : "Battery (A):", "value": ""},
    "PWV3"   : {"name" : "3.3V Rail (V):", "value": ""},
    "PWA3"   : {"name" : "3.3V Rail (A):", "value": ""},
    "PWV5"   : {"name" : "5V Rail (V):", "value": ""},
    "PWA5"   : {"name" : "5V Rail (A):", "value": ""},
    "PWV12"  : {"name" : "12V Rail (V):", "value": ""},
    "PWA12"  : {"name" : "12V Rail (A):", "value": ""}
}


speed_bufl = []
speed_bufr = []
speed_idx = 0

steer_buf = []
steer_idx = 0

pedal_buf = []
pedal_idx = 0

AVG_VAL = 64 

def init_subscribers():
    """
    Initialize the subscribers for the gui
    """
    rospy.Subscriber('Get_FNR', UInt8, fnr_callback)
    rospy.Subscriber('Get_Speed', UInt16MultiArray, speed_callback)
    rospy.Subscriber('Get_Steering', UInt16, steer_callback)
    rospy.Subscriber('Get_Pedal', UInt16, pedal_callback)
    rospy.Subscriber('Get_Power', UInt16MultiArray, power_callback)

def fnr_callback(data):
    """
    Sets FNR value in global info file
    """
    global info
    info["FNR"]["value"] = data.data

def speed_callback(data):
    """
    Averages speed values for left and right wheels
    Stores averaged values in info
    """
    global speed_bufl, speed_bufr, speed_idx
    speed_bufl[speed_idx] = data.data[0]
    speed_bufr[speed_idx] = data.data[1]
    speed_idx = (speed_idx + 1) % AVG_VAL

    if speed_idx == AVG_VAL:
        avgl = 0
        avgr = 0
        for i in range(0, AVG_VAL):
            avgl += avgl
            avgr += avgr
        info["SPDL"]["value"] = avgl/AVG_VAL/1000.0
        info["SPDR"]["value"] = avgr/AVG_VAL/1000.0



def steer_callback(data):
    """
    Averages steer values and stores averaged values in info
    """
    global steer_buf, steer_idx
    steer_buf[steer_idx] = data.data
    steer_idx = (steer_idx + 1) % AVG_VAL

    if steer_idx == AVG_VAL:
        avg = 0
        for i in range(0, AVG_VAL):
            avg += avg
        info["STR"]["value"] = avg/AVG_VAL

def pedal_callback(data):
    """
    Averages pedal values and stores averaged values in info
    """
    global pedal_buf, pedal_idx
    pedal_buf[pedal_idx] = data.data
    pedal_idx = (pedal_idx + 1) % AVG_VAL

    if pedal_idx == AVG_VAL:
        avg = 0
        for i in range(0, AVG_VAL):
            avg += avg
        info["PDL"]["value"] = avg/AVG_VAL

def power_callback(data):
    """
    Averages pedal values and stores averaged values in info
    """
    global info
    info["PWB12"]["value"] = data.data[0]
    info["PWB12"]["value"] = data.data[1]
    info["PWV3"]["value"] = data.data[2] 
    info["PWA3"]["value"] = data.data[3]
    info["PWV5"]["value"] = data.data[4]
    info["PWA5"]["value"] = data.data[5]
    info["PWV12"]["value"] = data.data[6]
    info["PWA12"]["value"] = data.data[7]


if __name__ == '__main__':
    rospy.init_node('BoardComms', anonymous=True)

    init_subscribers()
