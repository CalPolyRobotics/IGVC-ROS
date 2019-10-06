#!/usr/bin/env python
import rospy
import math

from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16

from willitcrash import get_minmax_range

# Half the Width of the Golf Cart
HALF_WIDTH = .59

# Set_FNR Publisher
PUB_STEERING = rospy.Publisher('Set_Steering', UInt16, queue_size=10)

# Max Number for Turning
MAX_TURN = 65535

# Max Turning Angles
MAX_THETA_R = 30.5
MAX_THETA_L = 35.5

# Left States
L3 = 0 
L2 = MAX_TURN * (1/8.0) 
L1 = MAX_TURN * (2/8.0) 
L0 = MAX_TURN * (3/8.0)

# Center State
C0 = MAX_TURN * (4/8.0) 

# Right 
R0 = MAX_TURN * (5/8.0)
R1 = MAX_TURN * (6/8.0)
R2 = MAX_TURN * (7/8.0)
R3 = MAX_TURN 

# Distance Considered an "Open" Path 
IN_VIEW = 4.0 

# Min Distance for a viable range point
TOL = .003

# THe Max distance of a point
MAX_POINT = 100 

# Ranges to View for Computation
RANGE_BEG = 200 
RANGE_END = 350 

# Should receive this from a publisher
# TODO TODO TODO Should be 0
FNR_state = 1 
turn_state = C0 

VIEW = 20 

# Count for Determing How Long to Keep Turned
count = 0
MAX_COUNT = 120 

def interpretRanges(ranges):
   global FNR_state
   global turn_state
   global count

   if FNR_state == 1:
      print count
      if turn_state == L3:
         if count > MAX_COUNT:
            PUB_STEERING.publish(C0)
            turn_state = C0
         else:
            count = count + 1
      if turn_state == C0:
         for i in range(RANGE_BEG, RANGE_END):
            if rangeRectangle(i, ranges[i]):
               if ranges[i] > TOL and ranges[i] < IN_VIEW:
                  PUB_STEERING.publish(L2)
                  turn_state = L3
                  count = 0
                  break
def willItCrash(ranges):
   for i in range(RANGE_BEG, RANGE_END):
      if rangeRectangle(i, ranges[i]):
         if ranges[i] > TOL and ranges[i] < VIEW:
            print "True ", i
            return True
   print "False"
   return False

def will_it_crash_turn(ranges):
   for index, value in enumerate(ranges):
      minmax = get_minmax_range(turn_state, index)
      if value > TOL and value > minmax[0] and value < minmax[1]:
            rospy.loginfo("111111111111111111111111111111111111111111111111111111")
            return True
   rospy.loginfo("0")
   return False

def rangeRectangle(index, length):
   if index > 541/2:
      if length > (HALF_WIDTH/math.cos(math.radians(180-((index - 90)/2.0)))):
         return False
   else:
      if length > (HALF_WIDTH/math.cos(math.radians((index - 90)/2.0))):
         return False
   return True

def listener():
   rospy.init_node('lidar_logic_turn', anonymous=True)
   rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, rangesCallback)
   rospy.Subscriber("Set_FNR", UInt8, fnrCallback)

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

def fnrCallback(data):
   global FNR_state

   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   FNR_state = data.data

def rangesCallback(data):
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   #interpretRanges(data.data);
   will_it_crash_turn(data.data);

if __name__ == '__main__':
   listener()

