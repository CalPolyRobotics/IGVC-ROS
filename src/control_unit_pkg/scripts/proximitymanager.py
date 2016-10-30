#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import UInt8
from std_msgs.msg import UInt16
# States
NEUTRAL = 0
FORWARD = 1
REVERSE = 2
STATE_4 = 3

# Half the Width of the Golf Cart
HALF_WIDTH = .546

# Set_FNR Publisher PUB_FNR = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
# Declare Publishers
PUB_FNR = rospy.Publisher('Set_FNR', UInt8, queue_size=10)
PUB_FNR.publish(1)
#PUB_THR = rospy.Publisher('Set_Throttle', UInt16, queue_size=10)

# Distance to Trigger Reverse
FIRST_VIEW = .05

# Distance to Trigger Neutral Range(FirstView to SecondView)
SECOND_VIEW = 1.5 # 1.5

# Safe Distance to Trigger Forward Range( > SecondView)

# Min Distance for a viable range point
TOLERANCE = .003

# Ranges to View for Computation
RANGE_BEG =130 #90   # 130
RANGE_END =421 #450  # 421

# Counter for State 4: REVERSE_TO_NEUTRAL
#STATE_4_COUNT_DELAY = 30

# Initialize values, should receive this from a publisher
FNR_state = 1
currentSpeed = 0
count = 0

'''def interpretRanges(ranges):
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
'''
def interpretRanges(ranges):
   global FNR_state
   global pref_state
   pref_state = FORWARD
   print ("Pref State: " , pref_state)
   if FNR_state != STATE_4:
      for i in range(RANGE_BEG, RANGE_END):
         if range_Rectangle(i, ranges[i]):
            print "In Range Rectangle"
            if(ranges[i] > TOLERANCE):
               print "Greater than Tolderance"
               if(in_threshold_2(ranges[i]) == True):
                  print "In Threshold 2"
                  if(in_threshold_1(ranges[i]) == True):
                     #print "In Threshold 1"
                     if(is_Cart_Moving() == True):
                        pref_state = REVERSE             # change_state(REVERSE)
                        #print "IN CART MOVING -- REVERSE"
                        break
                     else:
                        pref_state = NEUTRAL
                        "CART NOT MOVING -- NEUTRAL"
                        break
                  else:
                     pref_state = NEUTRAL               #change_state(NEUTRAL)
                     print "ONLY in threshold 2"
                     break
      if pref_state != FNR_state:
         if pref_state == FORWARD and FNR_state == REVERSE:
            print("Going into Neutral")
            PUB_FNR.publish(NEUTRAL)
         if pref_state == REVERSE and FNR_state == FORWARD:
            print("Going to Neutral 2")
            PUB_FNR.publish(NEUTRAL)
         print ("Publishing state:" + pref_state)
         PUB_FNR.publish(pref_state)
         FNR_state = pref_state

   else:
     change_state(FORWARD)

def change_state(new_state):
   PUB_FNR.publish(new_state)
   FNR_state = new_state


def in_threshold_1(value): # closer
   if value < FIRST_VIEW:
      return True
   else:
      return False

def in_threshold_2(value): # Further Out
   if value < SECOND_VIEW:
      return True
   else:
      return False

def is_Cart_Moving():
   global currentSpeed
   print ("curSpeed: " + currentSpeed)
   if currentSpeed > 10:
      return True
   else:
      return False

def range_Rectangle(index, length):
   if index > 541/2:
      if length > (HALF_WIDTH/math.cos(math.radians(180-((index - 90)/2)))):
         return False
   else:
      if length > (HALF_WIDTH/math.cos(math.radians((index - 90)/2))):
         return False

   return True

def listener():
   print("Hello")
   rospy.init_node('proximity_manager', anonymous=True)
   rospy.Subscriber("lidar_scan_ranges", Float32MultiArray, callbackLidar)
   rospy.Subscriber("Get_Speed", UInt16MultiArray, callbackSpeed)
   rospy.Subscriber("Get_FNR", UInt8, callbackFNR)
   # spin() simply keeps python from exiting until this node is stopped

   rospy.spin()

def callbackLidar(data):
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   interpretRanges(data.data);

def callbackSpeed(data):
   global currentSpeed
   currentSpeed =(data.data[0])/10

   print"SPEED is : ", currentSpeed

def callbackFNR(data):
   global FNR_state
   FNR_state = data
   print ("Current State is: %s",  data)

if __name__ == '__main__':
   listener()
