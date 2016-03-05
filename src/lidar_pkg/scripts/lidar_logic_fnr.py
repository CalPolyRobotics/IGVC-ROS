#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray 
from std_msgs.msg import UInt8

# States
NEUTRAL = 0
FORWARD = 1
REVERSE = 2
STATE_4 = 3

# Half the Width of the Golf Cart
HALF_WIDTH = .546

# Set_FNR Publisher
PUB_FNR = rospy.Publisher('Set_FNR', UInt8, queue_size=10)

# Distance to Trigger Reverse
FIRST_VIEW = 1.0 

# Distance to Trigger Neutral Range(FirstView to SecondView)
SECOND_VIEW = 1.5 # 1.5 

# Safe Distance to Trigger Forward Range( > SecondView)

# Min Distance for a viable range point
TOLERANCE = .003

# Ranges to View for Computation
RANGE_BEG = 130 
RANGE_END = 421

# Counter for State 4: REVERSE_TO_NEUTRAL
STATE_4_COUNT_DELAY = 30

# Should receive this from a publisher
FNR_state = 0 
count = 0

def interpretRanges(ranges):
   global FNR_state
   
   if FNR_state != STATE_4:
      pref_state = FORWARD 
      for i in range(RANGE_BEG, RANGE_END):
         if rangeRectangle(i, ranges[i]):
            if ranges[i] > TOLERANCE:
               if ranges[i] < FIRST_VIEW:
                  pref_state = REVERSE 
                  break
               elif ranges[i] < SECOND_VIEW:
                  pref_state = NEUTRAL 
           
           # Otherwise: Preferred State Remains Forward

      # Set the State to Preferred if it is not Already
      if pref_state != FNR_state:
         if pref_state == FORWARD and FNR_state == REVERSE:
            PUB_FNR.publish(NEUTRAL)
         PUB_FNR.publish(pref_state)
         FNR_state = pref_state
   else:
      global count
      if count >= STATE_4_COUNT_DELAY:
         count = 0
         PUB_FNR.publish(FORWARD)
         FNR_state = FORWARD 
      else:
         count += 1

def rangeRectangle(index, length):
   if index > 541/2:
      if length > (HALF_WIDTH/math.cos(math.radians(180-((index - 90)/2)))):
         return False
   else:
      if length > (HALF_WIDTH/math.cos(math.radians((index - 90)/2))):
         return False

   return True
def listener():
   rospy.init_node('lidar_logic_fnr', anonymous=True)
   rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, callback)

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

def callback(data):
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   interpretRanges(data.data);

if __name__ == '__main__':
   listener()
