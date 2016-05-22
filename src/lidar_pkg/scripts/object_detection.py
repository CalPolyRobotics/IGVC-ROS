#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from lidar_pkg.msg import Object
from lidar_pkg.msg import Object_Map
# Set_FNR Publisher
PUB_OBJ = rospy.Publisher('Get_Objects', Object_Map, queue_size=10)

# Min Distance for a viable range point
MIN_RANGE = .003

# Max distance and light intensities for "objects"
INTENSITY_TOL = 20 
DISTANCE_TOL  = 0.1

def listener():
   rospy.init_node('lidar_logic_object_detection', anonymous=True)
   rospy.Subscriber("lidar_scan_laser_scan", LaserScan, laserScanCallback)

   # spin() simply keeps python from exiting until this node is stopped
   rospy.spin()

def laserScanCallback(data):
   object_map = Object_Map()
   # Worry About Infinity? MIN_RANGE
   start = 0
   i = 0
   min_range = data.ranges[i]
   while(i < len(data.ranges) - 1):
      if(abs(data.ranges[i] - data.ranges[i+1]) > DISTANCE_TOL or
            abs(data.intensities[i] - data.intensities[i+1]) > INTENSITY_TOL):

         # Add Object to Mapping
         obj = Object()
         obj.start = start
         obj.end = i
         obj.distance = min_range
         object_map.objects.append(obj)

         start = i+1
         min_range = data.ranges[i+1]
      else:
         if(data.ranges[i] < min_range):
            min_range = data.ranges[i]
      
      i = i + 1

   # Append last Object
   obj = Object()
   obj.start = start
   obj.end = i
   obj.distance = min_range
   object_map.objects.append(obj)

   # Publish Mapping
   PUB_OBJ.publish(object_map)
   #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

if __name__ == '__main__':
   listener()
