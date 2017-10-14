#!/usr/bin/env python

import rospy
import binascii

from std_msgs.msg import UInt8, UInt8MultiArray, UInt16MultiArray, UInt16  

fnr = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
thrt = rospy.Publisher('Set_Throttle', UInt16, queue_size=10)
ster = rospy.Publisher('Set_Steering', UInt16, queue_size=10)
sped = rospy.Publisher('Set_Speed', UInt16, queue_size=10)
ligt = rospy.Publisher('Set_Lights', UInt16, queue_size=10)
stop = rospy.Publisher('Stop', UInt8, queue_size=10)

def initTest():
   rospy.Subscriber('Status', UInt8, callbackStatus)
   #rospy.Subscriber('Get_Sonar_1', UInt8, callbackSonar1)
   #rospy.Subscriber('Get_Sonar_All', Uint8MultiArray, callbackSonarAll)
   rospy.Subscriber('Get_FNR', UInt8, callbackFNR)
   rospy.Subscriber('Get_Speed', UInt16MultiArray, callbackSpeed)
   rospy.Subscriber('Get_Steering', UInt16, callbackSteering)
   rospy.Subscriber('Get_Battery', UInt16, callabackBattery)
   rospy.Subscriber('Get_Power', UInt16MultiArray, callbackPower)


"----------- Callbacks ----------"

def callbackRStatus(data):

def callbackRSonar1(data):

def callbackRSonarAll(data):

def callbackRFNR(data):

def callbackRSpeed(data):

def callbackRSteering(data):

def callbackRPower(data):

def callbackRBattery(data):

"--------- End Callbacks --------"

publisher_callbacks = {
  0x00 : callbackRStatus,
  0x02 : callbackRSonar1,
  0x04 : callbackRSonarAll,
  0x08 : callbackRFNR,
  0x0E : callbackRSpeed,
  0x12 : callbackRSteering,
  0x16 : callbackRBattery,
  0x18 : callbackRPower}

def testBoardComms():

def testSetFNR():
def testSetSpeed():
def testSetSteering():
def testSetThrottle():
def testSetSpeed():
def testSetLights():
def testSetStop():

def testGetFNR():
def testGetStatus():
def testGetSpeed():
def testGetSteering():
def testGetBattery():
def testGetPower():
