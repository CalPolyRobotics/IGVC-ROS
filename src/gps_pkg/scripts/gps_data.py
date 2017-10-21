#!/usr/bin/env python
import serial
import rospy
import re
from std_msgs.msg import Int32 
from gps_pkg.msg import GPS_Coord


def extract_lat(gga_line):
   latArr = [gga_line[2][:-5],gga_line[2][-7:-5] + gga_line[2][-4:]]
   if len(latArr[0]):
      latDec = int(latArr[1])/60.0
      lat = int(latArr[0][:-2])*10000000 + int(latDec * 1000)
      if gga_line[3] == 'S':
          lat *= -1
      return lat
   else: return 0

def extract_long(gga_line):
   longArr = [gga_line[4][:-7],gga_line[4][-7:-5] + gga_line[4][-4:]]
   if len(longArr[0]):
      longDec = int(longArr[1])/60.0
      longNum = int(longArr[0])*10000000 + int(longDec * 1000)
      if gga_line[5] == 'W':
          longNum *= -1
      return longNum  
   else: return 0

def extract_angle(rmc_line):
   #for i in range(0, len(rmc_line)):
   #    print "%d : %s" %  (i, rmc_line[i])
   #return int(float(rmc_line[8])) 
   return 0

def talker():
    pub = rospy.Publisher('gps_data_coords', GPS_Coord, queue_size=10)
    rospy.init_node('gps_data', anonymous=True)
 
    with serial.Serial('/dev/ttyUSB0', baudrate=4800) as com:
       while not rospy.is_shutdown():
          lat = 0
          lng = 0
          angle = 0
          gps_data = com.readline().split(',')
          if gps_data is not None:
             if gps_data[0] == '$GPGGA':
                data = GPS_Coord()
                data.lat = extract_lat(gps_data)
                data.lng = extract_long(gps_data)
                rospy.loginfo(GPS_Coord)
                pub.publish(data)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
