#!/usr/bin/env python
import rospy
from gps_pkg.msg import GPS_Coord #TODO: Have Kyle figure out why this isn't working?!
from sensor_msgs.msg import Imu

USB = '/dev/ttyUSB0'


def gps_callback(data):
    print "------------------------------------\n"  
    print "LAT: ", data.lat
    print "LONG: ", data.lng
#    print "Angle: ", data.angle



def gps_listener():
    rospy.init_node('gps_listener', anonymous = True)
    rospy.Subscriber('/gps_data_coords', GPS_Coord, gps_callback)

    rospy.spin()



def imu_callback(data):
    print "--------------------------------------------\n", data.angular_velocity

def imu_listener(): #TODO: We don't want a listener, we want a service which does something WITH the data read in
    #i.e. a Server/Client
    rospy.init_node('imu_listener', anonymous = True)
    rospy.Subscriber("imu/data", Imu, imu_callback)
    
    rospy.spin()




if __name__ == '__main__':
    try:
       pass
    except rospy.ROSInterruptException:
       pass

