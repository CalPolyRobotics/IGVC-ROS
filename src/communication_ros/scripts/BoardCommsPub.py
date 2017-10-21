#!/usr/bin/env python

import rospy
import binascii

from std_msgs.msg import UInt8, UInt8MultiArray, UInt16MultiArray, UInt16  

status   =  rospy.Publisher("Status", UInt8, queue_size=1000)
sonar1   =  rospy.Publisher("Get_Sonar_1", UInt8, queue_size=1000)
sonarAll =  rospy.Publisher("Get_Sonar_All", UInt8MultiArray, queue_size=1000)
fnr      =  rospy.Publisher("Get_FNR", UInt8, queue_size=1000)
speed    =  rospy.Publisher("Get_Speed", UInt16MultiArray, queue_size=1000)
steering =  rospy.Publisher("Get_Steering", UInt16, queue_size=1000)
battery  =  rospy.Publisher("Get_Battery", UInt16, queue_size=1000)
power    =  rospy.Publisher("Get_Power", UInt16MultiArray, queue_size=1000)

msg_type = [0x00, 0x02, 0x04, 0x08, 0x0E, 0x12, 0x16, 0x18]

# Only Publishing Implemented Types
# TODO: Messages are getting mixed up when read

"----------- Callbacks ----------"

def callbackRStatus(data):
   status.publish(data[0])

def callbackRSonar1(data):
   sonar1.publish(data[0])

def callbackRSonarAll(data):
   sonarAll.publish(data)

def callbackRFNR(data):
   fnr.publish(data[0])

def callbackRSpeed(data):
   speed.publish(byteArrToUInt16Arr(data))

def callbackRSteering(data):
   steering.publish((data[0] << 8) | data[1])

def callbackRPower(data):
   power.publish(byteArrToUInt16Arr(data))

def callbackRBattery(data):
   battery.publish(data[0]) 

"--------- End Callbacks --------"


"----------- Conversion ----------"
def byteArrToUInt16Arr(data):
   conv = UInt16MultiArray()
   for i in range(0, len(data), 2):
      conv.data.append((data[i] << 8) | data[i+1])
   return conv

def byteArrToUInt8Arr(data):
   conv = UInt8MultiArray()
   for i in range(0, len(data)):
      conv.data.add(data[i])
   return conv 
"--------- End Conversion --------"

publisher_callbacks = {
  0x00 : callbackRStatus,
  0x02 : callbackRSonar1,
  0x04 : callbackRSonarAll,
  0x08 : callbackRFNR,
  0x0E : callbackRSpeed,
  0x12 : callbackRSteering,
  0x16 : callbackRBattery,
  0x18 : callbackRPower}

