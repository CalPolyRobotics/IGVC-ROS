Follow these steps in order to generate a map:

1) In terminal window 1:
    roslaunch control_unit_pkg play_rosbag.launch
        - Launch RQT GUI for visualizing data with use_sim_time=True

2) In terminal window 2:
    Option 1:
        rosrun gmapping slam_gmapping scan:=lidar_scan_laser_scan
            - Start SLAM gmapping with default parameters
    Option 2:
        roslaunch nav_pkg gmapping.launch
            - Start SLAM gmapping with user specified parameters

3) In terminal window 3: 
    rosbag play --clock <name_of_bag>
        - Start rosbag playback using simulated time.
        - NOTE: Attempting to use the rosbag pane in RQT GUI will not work for 
          generating a map.

4) When rosbag finishes, in terminal window 4:
    rosrun map_server map_saver -f <name_of_map>
        - Save generate map


Notes:
- The map being generated can be viewed live by adding the /map topic in RViz.
            
