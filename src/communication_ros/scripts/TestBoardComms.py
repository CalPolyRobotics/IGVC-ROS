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
   rospy.Subscriber('Get_Sonar_All', Uint8MultiArray, assertSonA )
   rospy.Subscriber('Get_FNR', UInt8, assertFNR )
   rospy.Subscriber('Get_Speed', UInt16MultiArray, assertSpeed )
   rospy.Subscriber('Get_Steering', UInt16, assertSteer )
   rospy.Subscriber('Get_Battery', UInt16, asserBatt )
   rospy.Subscriber('Get_Power', UInt16MultiArray, assertPower )


getStatPass = "No Response"
getSon1Pass = "No Response"
getSonAPass = "No Response"
getFNRPass = "No Response"
getSpeedPass = "No Response"
getSteerPass = "No Response"
getPowerPass = "No Response"
getBattPass = "No Response"

def testBoardComms():
   initTests()
   
   print "Testing Getters"
   time.sleep(5000)

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
   getStatPass = "Not Implemented"

def assertSon1(data):
   getSon1Pass = "Not Implemented"

def assertSonA(data):
   getSonAPass = "Not Implemented"

def assertFNR(data):

def assertSpeed(data):

def assertSteer(data):

def assertBatt(data):

def assertPower(data):

"--------- End Callbacks --------"

if __name__ == '__main__':
   testBoardComms()
