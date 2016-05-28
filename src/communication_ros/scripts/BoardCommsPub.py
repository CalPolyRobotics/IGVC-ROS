#!/usr/bin/env python

import rospy

from std_msgs.msg import UInt8, UInt8MultiArray, UInt16MultiArray, UInt16  

status   = rospy.Publisher("Status", UInt8, queue_size=1000)
sonar1   =  rospy.Publisher("Get_Sonar_1", UInt8, queue_size=1000)
sonarAll =  rospy.Publisher("Get_Sonar_All", UInt8MultiArray, queue_size=1000)
fnr      = rospy.Publisher("Get_FNR", UInt8, queue_size=1000)
speed    =  rospy.Publisher("Get_Speed", UInt16, queue_size=1000)
steering =  rospy.Publisher("Get_Steering", UInt16, queue_size=1000)
battery  = rospy.Publisher("Get_Battery", UInt16, queue_size=1000)
power    =  rospy.Publisher("Get_Power", UInt16MultiArray, queue_size=1000)

# msg_type = [0x00, 0x02, 0x04, 0x08, 0x0E, 0x12, 0x16, 0x18]
msg_type = [0x08, 0x0E, 0x12, 0x18]


"----------- Callbacks ----------"

def callbackRStatus(data):
   status.publish(hexToUInt(data))

def callbackRSonar1(data):
   sonar1.publish(hexToUInt(data))

def callbackRSonarAll(data):
   sonarAll.publish(hexToUInt8Array(data))

def callbackRFNR(data):
   fnr.publish(hexToUInt(data))

def callbackRSpeed(data):
   speed.publish(hexToUInt(data))

def callbackRSteering(data):
   steering.publish(hexToUInt(data))

def callbackRPower(data):
   power.publish(hexToUInt16Array(data))

"--------- End Callbacks --------"


"----------- Conversion ----------"
def hexToUInt(data):
   num = int(data, 16)
   return num

def hexToUInt16Array(data):
   array = UInt16MultiArray()

   # Take 4 Hex and append value to array
   for i in range(0, len(data), 4):
      num_str = data[i: i+4] 
      num = int(num_str, 16)
      array.data.append(num)

   return array

def hexToUInt8Array(data):
   array = UInt8MultiArray()

   # Take 2 Hex and append value to array
   for i in range(0, len(data), 2):
      num_str = data[i: i+2] 
      num = int(num_str, 16)
      array.data.append(num)

   return array

"--------- End Conversion --------"

publisher_callbacks = {
  # 0x00 : callbackRStatus,
  # 0x02 : callbackRSonar1,
  # 0x04 : callbackRSonarAll,
   0x08 : callbackRFNR,
   0x0E : callbackRSpeed,
   0x12 :  callbackRSteering,
  # 0x16 : callbackRBattery,
   0x18 : callbackRPower}

