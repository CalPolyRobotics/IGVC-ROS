#!/usr/bin/env python
import serial
import rospy
import re
from std_msgs.msg import Int32 
from gps_pkg.msg import GPS_Coord


def talker():
    pub = rospy.Publisher('gps_data_coords', GPS_Coord, queue_size=10)
    rospy.init_node('gps_data', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    latSum = 0
    lngSum = 0
    div = 2 
    with serial.Serial('/dev/ttyUSB0', baudrate=4800) as com:
        while not rospy.is_shutdown():
            gps_info = com.readline() 
            gga_info = re.split(',', gps_info)

	    if gga_info != None:
          if gga_info[0] == '$GPGGA':

             #latitude
             latArr = [gga_info[2][:-5],gga_info[2][-7:-5] + gga_info[2][-4:]]

             #longitude
             #Assuming we always receive 4 decimal points for each latitude
             longArr = [gga_info[4][:-7],gga_info[4][-7:-5] + gga_info[4][-4:]]

             if latArr[0] != '' and longArr[0] != '':
                 i += 1
                 latDec = int(latArr[1])/60.0
                 lat = int(latArr[0][:-2])*10000000 + int(latDec * 1000)
             
                 longDec = int(longArr[1])/60.0
                 longNum = int(longArr[0])*10000000 + int(longDec * 1000)

                 if gga_info[3] == 'S':
                    lat = lat * -1 
                 if gga_info[5] == 'W':
                    longNum = longNum * -1
                 latSum += lat
                 lngSum += longNum

                 if i % div  == 0:
                    data = GPS_Coord()

                    data.lat = latSum/div
                    data.lng = lngSum/div

                    rospy.loginfo(data)
                    pub.publish(data)

                    latSum = 0
                    lngSum = 0
             rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
