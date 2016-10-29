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
#PUB_THR = rospy.Publisher('Set_Throttle', UInt16, queue_size=10)

# Distances
x0 = .1 
x1 = 1.0 
x2 = 2.0

# Safe Distance to Trigger Forward Range( > SecondView)

# Min Distance for a viable range point
TOLERANCE = .003

# Ranges to View for Computation
RANGE_BEG =130 #90   # 130 
RANGE_END =421 #450  # 421

# Counter for State 4: REVERSE_TO_NEUTRAL
#STATE_4_COUNT_DELAY = 30

# Initialize values, should receive this from a publisher
FNR_state = 0 
currentSpeed = 0

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
               if(in_threshold_2() == True):
                  print "In Threshold 2"
                  if(in_threshold_1() == True):
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
         print"Publishing state:", pref_state
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
