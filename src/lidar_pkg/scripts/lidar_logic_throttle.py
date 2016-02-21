#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16

# Set_FNR Publisher
PUB_FNR = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
PUB_THR = rospy.Publisher('Set_Throttle', UInt16, queue_size=10) 

# States
FIRST_THR = 10 
SECOND_THR = 20
THIRD_THR = 45 
FOURTH_THR = 63

# Views That Trigger Throttle Changes
FIRST_VIEW = 0.5 
SECOND_VIEW = 1.5 #1.5 
THIRD_VIEW =  3.0 #3.0 
FOURTH_VIEW = 4.0 #4.0 

# Min Distance for a viable range point
TOLERANCE = .003

# Ranges to View for Computation
RANGE_BEG = 90 
RANGE_END = 451 

HALF_WIDTH = .546

# Should receive this from a publisher
FNR_state = 0
THR_state = 0
count = 0

def interpretRanges(ranges):
   global FNR_state
   global THR_state
   
   pref_state = FOURTH_THR

   if FNR_state == 2:
      pref_state = FIRST_THR
   else:
      for i in range(RANGE_BEG, RANGE_END):
         if rangeRectangle(i, ranges[i]):
            if ranges[i] > TOLERANCE:
               if ranges[i] < FIRST_VIEW:
                  pref_state = FIRST_THR
                  break 
               if ranges[i] < SECOND_VIEW:
                  pref_state = SECOND_THR
               elif not pref_state == SECOND_THR and ranges[i] < THIRD_VIEW:
                  pref_state = THIRD_THR

   if not pref_state == THR_state:
      PUB_THR.publish(pref_state)
      THR_state = pref_state

def listener():
   rospy.init_node('lidar_logic_fnr', anonymous=True)
   rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, rangesCallback)
   rospy.Subscriber("Set_FNR", UInt8, fnrCallback)

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

def rangesCallback(data):
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   interpretRanges(data.data);

def fnrCallback(data):
   global FNR_state

   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   FNR_state = data

def rangeRectangle(index, length):
   if index > 541/2:
      if length > (HALF_WIDTH/math.cos(math.radians(180-((index - 90)/2)))):
         return False
   else:
      if length > (HALF_WIDTH/math.cos(math.radians((index - 90)/2))):
         return False
   return True

if __name__ == '__main__':
   listener()
