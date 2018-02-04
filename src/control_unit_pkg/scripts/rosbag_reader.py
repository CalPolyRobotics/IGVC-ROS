#!/usr/bin/env python
import rospy
import rosbag

print("He;l")

bag = rosbag.Bag("test_very_good_data.bag")

for topic, msg, t in bag.read_messages(topics=["/lidar_scan_laser_scan"]):
    print msg
    break

bag.close()
