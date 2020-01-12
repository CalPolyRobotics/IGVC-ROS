#!/usr/bin/env python
import csv
import rospy
import geopy.distance
from sensor_msgs.msg import NavSatFix


class WaypointCollector:
    """
    Collect waypoint information for purposes of generating a waypoint map.
    """
    def __init__(self, map_name, on_road):
        self.map_name = map_name
        self.on_road = on_road
        self.prev_waypoint = None

        # create csv writer to write waypoint to a csv file
        self.csvfile = open(self.map_name, "w")
        self.writer = csv.writer(self.csvfile, delimiter=",")
        self.writer.writerow(("Latitude", "Longitude", "OnRoad"))

        # create subscriber for waypoint collection
        rospy.Subscriber("fix", NavSatFix, self.waypoint_cb)

    def waypoint_cb(self, gps_info):
        """
        Callback for handling new NavSatFix msgs.

        Args:
            gps_info: a NavSatFix msg
        """
        curr_waypoint = (gps_info.latitude, gps_info.longitude)

        # record new waypoint if the distance from the previously recorded waypoint is greater than 3 meters
        if self.prev_waypoint is None or geopy.distance.distance(self.prev_waypoint, curr_waypoint).m >= 3:
            rospy.loginfo(curr_waypoint)
            self.writer.writerow((curr_waypoint[0], curr_waypoint[1], self.on_road))
            self.prev_waypoint = (gps_info.latitude, gps_info.longitude)  # update the previous waypoint to the newly recorded waypoint


if __name__ == '__main__':
    rospy.init_node("waypoint_collector", anonymous=True)

    try:
        map_name = rospy.get_param("waypoint_map_name")
        on_road = rospy.get_param("waypoints_on_road")
        waypoint_collector = WaypointCollector(map_name, on_road) 
    except rospy.ROSInterruptException:
        waypoint_collector.csvfile.close()

    rospy.spin() 
