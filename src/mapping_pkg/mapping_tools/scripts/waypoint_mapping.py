#!/usr/bin/env python
import rospy
import geopy
import csv
from sensor_msgs.msg import NavSatFix

class WaypointMapper:
    '''
    To save waypoints (Latitude , Longitude) of a path to a csv file.
    '''
    def __init__(self, file_name):
        self.prev_gps_info = None
        
        self.csvfile = open(file_name, "w")
        self.writer = csv.writer(self.csvfile , delimiter = ",")
        self.writer.writerow(("Latitude" , "Longitude"))       
        
        rospy.init_node('waypoint_mapper', anonymous=True)
        rospy.Subscriber('fix', NavSatFix, self.waypoint_cb)
    
    def waypoint_cb(self , gps_info):
       '''
       Call back for gps information
       
       Args:
            gps_info: a NavSatFix message
       '''
        lat_long = (gps_info.latitude, gps_info.longitude)
        
        # records the waypoint if the distance from the previously recorded waypoint is greater than 3 meters
        if self.prev_gps_info is None or geopy.distance.distance(self.prev_gps_info , lat_long) >= 3:
            prev_gps_info = (gps_info.latitude , gps_info.longitude) # update the previous gps info to the last recorded waypoint
            rospy.loginfo(prev_gps_info)
            self.writer.writerow(prev_gps_info)

if __name__ == "__main__":
    try:
        file_name = rospy.get_param("waypoint_map_file", "Waypoint_map_1.csv")
        waypoint_mapper = WaypointMapper(file_name)
    except rospy.ROSInterruptException:
        waypoint_mapper.csvfile.close()
    rospy.spin()
