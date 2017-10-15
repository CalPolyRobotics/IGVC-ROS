#!/usr/bin/env python

import rospy
import time

from std_msgs.msg import UInt8, UInt8MultiArray, UInt16MultiArray, UInt16  

"""
fnr = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
thrt = rospy.Publisher('Set_Throttle', UInt16, queue_size=10)
ster = rospy.Publisher('Set_Steering', UInt16, queue_size=10)
sped = rospy.Publisher('Set_Speed', UInt16, queue_size=10)
ligt = rospy.Publisher('Set_Lights', UInt16, queue_size=10)
stop = rospy.Publisher('Stop', UInt8, queue_size=10)
"""

def initTest():
   rospy.Subscriber('Status', UInt8, assertStatus )
   rospy.Subscriber('Get_Sonar_1', UInt8, assertSon1 )
   rospy.Subscriber('Get_Sonar_All', UInt8MultiArray, assertSonA )
   rospy.Subscriber('Get_FNR', UInt8, assertFNR )
   rospy.Subscriber('Get_Speed', UInt16MultiArray, assertSpeed )
   rospy.Subscriber('Get_Steering', UInt16, assertSteer )
   rospy.Subscriber('Get_Battery', UInt16, assertBatt )
   rospy.Subscriber('Get_Power', UInt16MultiArray, assertPower )

defaultResponse = "\033[1;31mNo Response\033[0;0m"

getStatPass = defaultResponse
getSon1Pass = defaultResponse
getSonAPass = defaultResponse
getFNRPass = defaultResponse
getSpeedPass = defaultResponse
getSteerPass = defaultResponse
getPowerPass = defaultResponse
getBattPass = defaultResponse

def printSuccess(msg):
   print "\033[0;32m" + msg + "\033[0;0m"

def testBoardComms():
   initTest()
   
   printSuccess("Testing Getters...")
   time.sleep(1)

   print "Status Test: ", getStatPass
   print "Sonar 1 Test: ", getSon1Pass
   print "Sonar All Test: ", getSonAPass
   print "FNR Test: ", getFNRPass
   print "Speed Test: ", getSpeedPass
   print "Steering Test: ", getSteerPass
   print "Power Test: ", getPowerPass
   print "Battery Test: ", getBattPass

"----------- Callbacks ----------" 
def assertStatus(data):
   global getStatPass
   getStatPass = "Not Implemented"

def assertSon1(data):
   global getSon1Pass
   getSon1Pass = "Not Implemented"

def assertSonA(data):
   global getSonAPass
   getSonAPass = "Not Implemented"

def assertFNR(data):
   global getFNRPass
   # Validate the data is in range[0, 1]
   if (data.data < 0 or data.data > 2):
      getFNRPass = "FNR Data is out of range [0,2]: " + data.data
   else:
      getFNRPass = "Success"

def assertSpeed(data):
   global getSpeedPass
   # Validate speed range [0, 64]
   if (data.data < 0 or data.data > 64):
      getSpeedPass = "Speed Data is out of range [0,64]:"
   else:
      getSpeedPass = "Success"

def assertSteer(data):
   global getSteerPass
   # Validate steering range [0, 65535]
   if (data.data < 0 or data.data > 64):
      getSteerPass = "Steer Data is out of range [0,65535]:"
   else:
      getSteerPass = "Success"

def assertBatt(data):
   global getStatPass

def assertPower(data):
   global getStatPass

"--------- End Callbacks --------"

if __name__ == '__main__':
   testBoardComms()
