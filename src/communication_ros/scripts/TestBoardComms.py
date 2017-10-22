#!/usr/bin/env python

import rospy
import time
from TestSuite import *

from std_msgs.msg import UInt8, UInt8MultiArray, UInt16MultiArray, UInt16  

"""
fnr = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
thrt = rospy.Publisher('Set_Throttle', UInt16, queue_size=10)
ster = rospy.Publisher('Set_Steering', UInt16, queue_size=10)
sped = rospy.Publisher('Set_Speed', UInt16, queue_size=10)
ligt = rospy.Publisher('Set_Lights', UInt16, queue_size=10)
stop = rospy.Publisher('Stop', UInt8, queue_size=10)
"""
global statusSub, sonar1Sub, sonarAllSub, FNRSub, speedSub, steeringSub, batterySub, powerSub


defaultResponse = buildFailString("No Response")

getStatPass = defaultResponse
getSon1Pass = defaultResponse
getSonAPass = defaultResponse
getFNRPass = defaultResponse
getSpeedPass = defaultResponse
getSteerPass = defaultResponse
getPowerPass = defaultResponse
getBattPass = defaultResponse
testsRun = 0
testsSkipped = 0
passCount = 0

def printTest(msg, result):
   global testsRun
   testsRun+=1
   print "____________________\n(" + msg + ") Test"
   print "\t"+ result

def printSummary():
   global testsRun
   global passCount
   global testsSkipped
   failCount = testsRun - passCount
   print "____________________"
   print "**************** Test Summary ****************"
   print str(testsRun) + " tests run"
   print buildWarningString("[WARNING] - Skipped " +  str(testsSkipped) + " Tests")
   if (failCount > 0):
      printFail("[FAILED] - " + str(failCount) + " Tests")
   passRate = float(100*passCount/float(testsRun))
   if (passRate >= 100):
      print(buildSuccessString("[RATE]- 100%"))
   else:
      print(buildFailString("[RATE]- " + str(passRate) + "%"))
   #print "PassRate: " + str(float(100*passCount/float(testsRun))) + "%"
   print "**********************************************"

def testBoardComms():
   
   print "**************** Testing Getters *************"
   time.sleep(2)

   printTest("Status", getStatPass)
   #printTest("Sonar 1", getSon1Pass)
   #printTest("Sonar All", getSonAPass)
   printTest("FNR", getFNRPass)
   printTest("Speed", getSpeedPass)
   printTest("Steering", getSteerPass)
   #printTest("Power", getPowerPass)
   #printTest("Battery", getBattPass)

   printSummary()

"----------- Callbacks ----------" 
def assertStatus(data):
   global statusSub, passCount, getStatPass, testsRun, testsSkipped
   statusSub.unregister()
   getStatPass = buildWarningString("Not Implemented")
   testsRun-=1
   testsSkipped+=1
   #passCount+=1

def assertSon1(data):
   global passCount, getSon1Pass, testsRun, testsSkipped
   getSon1Pass = buildWarningString("Not Implemented")
   testsRun-=1
   testsSkipped+=1
   #passCount+=1

def assertSonA(data):
   global passCount, getSonAPass, testsRun, testsSkipped
   getStatPass = buildWarningString("Not Implemented")
   testsRun-=1
   testsSkipped+=1
   #passCount+=1

def assertFNR(data):
   global FNRSub, passCount, getFNRPass
   FNRSub.unregister()
   # Validate the data is in range[0, 1]
   if (data.data < 0 or data.data > 3):
      getFNRPass = buildFailString("FNR Data is out of range [0,3]: " + data.data)
      print "data" + str(data)
   else:
      getFNRPass = buildSuccessString("Success")
      passCount+=1

def assertSpeed(data):
   global speedSub
   global passCount
   global getSpeedPass
   speedSub.unregister()
   # Validate speed range [0, 64]
   if (data.data < 0 or data.data > 416):
      getSpeedPass = buildFailString("Speed Data is out of range [0,64]:")
      #print "data" + str(data)
   else:
      getFNRPass = buildSuccessString("Success")
      passCount+=1

def assertSteer(data):
   global passCount, getSteerPass, steeringSub
   steeringSub.unregister()
   # Validate steering range [0, 65535]
   if (data.data < 0 or data.data > 64):
      getSteerPass = buildFailString("Steer Data is out of range [0,65535]:")
   else:
      getSteerPass = buildSuccessString("Success")
      passCount+=1

def assertBatt(data):
   global getStatPass

def assertPower(data):
   global getStatPass

"--------- End Callbacks --------"
if __name__ == '__main__':
   global statusSub, sonar1Sub, sonarAllSub, FNRSub, speedSub, steeringSub, batterySub, powerSub
   rospy.init_node('TestBoardComms', anonymous=True)
   statusSub = rospy.Subscriber('Status', UInt8, assertStatus )
   sonar1Sub = rospy.Subscriber('Get_Sonar_1', UInt8, assertSon1 )
   sonarAllSub = rospy.Subscriber('Get_Sonar_All', UInt8MultiArray, assertSonA )
   FNRSub = rospy.Subscriber('Get_FNR', UInt8, assertFNR )
   speedSub = rospy.Subscriber('Get_Speed', UInt16MultiArray, assertSpeed )
   steeringSub = rospy.Subscriber('Get_Steering', UInt16, assertSteer )
   batterySub = rospy.Subscriber('Get_Battery', UInt16, assertBatt )
   powerSub = rospy.Subscriber('Get_Power', UInt16MultiArray, assertPower )
   testBoardComms()
