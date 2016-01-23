#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Float32MultiArray 

prev_list = []
cur_list = []
object_location = []
first = True
threshold = .05

class Detection:
   def __init__(self, beg, end, distance, size):
      self.beg = beg
      self.end = end
      self.distance = distance
      self.size = size
   
   def toString(self):
       result = 'Object from: ', self.beg, ' to ',  self.end, ' at a distance of: ', self.distance , ' with size: ' , self.size
       return result


def callback(data):
   # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
   interpretRanges(data.data);
    
def listener():
   rospy.init_node('lidar_listener', anonymous=True)

   rospy.Subscriber("ranges", Float32MultiArray, callback)

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

def interpretRanges(ranges):
   global prev_list
   global cur_list
   global first
   global object_location

   dif_list = []
   new_prev = []
 
   if not first:
      # Dectect and store locations of objects
      prev_num = ranges[0]
      beg = 0
      end = 0
      distance = 0
      for i, elem in enumerate(ranges):
         if(abs(prev_num - elem) < threshold):
            distance += elem
            end = i
            prev_num = elem
         else:
            if(end-beg > 1):
               ran = end-beg
               avg_dist = distance/ran
               size = 2*avg_dist*math.tan(math.radians(ran/4))
               d = Detection(beg, end, avg_dist, size)
               object_location.append(d)
            distance = 0
            beg = i
            prev_num = elem
                         
         
         new_prev.append(elem)
         dif_list.append(elem - prev_list[i])
         #if abs(dif_list[i]) > threshold and dif_list[i] < 0:
             #print dif_list[i]*25
      for elem in object_location: # Filter out small objects REMOVE AFTER TESTING
         if elem.end-elem.beg > 5:
            print elem.toString()

   else:
      for elem in ranges:
         new_prev.append(elem)

   prev_list = new_prev

   first = False

if __name__ == '__main__':
    listener()

 
class Detection:
   def __init__(self, beg, end, distance):
      self.beg = beg
      self.end = end
      self.distance = distance
   
   def toString(self):
      result = "Object from: "
      result += beg
      result += " to "
      result += end
      result += " at a distance of: "
      result += distance
      result += " meters away"
      return result
