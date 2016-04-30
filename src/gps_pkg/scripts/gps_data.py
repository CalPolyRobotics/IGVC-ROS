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



#          gps_info = com.readline() 
#          gga_info = re.split(',', gps_info)

          #if gga_info != None:
          #   if gga_info[0] == '$GPGGA':

          #      #latitude
          #      latArr = [gga_info[2][:-5],gga_info[2][-7:-5] + gga_info[2][-4:]]

          #      #longitude
          #      #Assuming we always receive 4 decimal points for each latitude
          #      longArr = [gga_info[4][:-7],gga_info[4][-7:-5] + gga_info[4][-4:]]

          #          if div % i  == 0:
          #              rospy.loginfo(latSum/div)
          #              pub.publish(latSum/div)
          #              rospy.loginfo(lngSum/div)
          #              pub.publish(lngSum/div)
          #              latSum = 0
          #              lngSum = 0
          #              i = 0
          #      rate.sleep()


def talker():
    pub = rospy.Publisher('gps_data_coords', GPS_Coord, queue_size=10)
    rospy.init_node('gps_data', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    with serial.Serial('/dev/ttyUSB0', baudrate=4800) as com:
       while not rospy.is_shutdown():
          lat = 0
          long = 0
          angle = 0
          div = 3
          i = 0
          gps_data = com.readline().split(',')
          if gps_data is not None:
             i += 1
             if gps_data[0] == '$GPGGA':
                lat = extract_lat(gps_data)
                long = extract_long(gps_data)
             elif gps_data[0] == '$GPRMC':
                angle = extract_angle(gps_data)
             
#             if div % i  == 0:
#                data = GPS_Coord()
#                data.lat = lat
#                data.lng = long
#                data.angle = angle
# #               rospy.loginfo(GPS_Coord)
#                pub.publish(data)
#             rate.sleep()



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
