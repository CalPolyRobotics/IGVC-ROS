# IGVC-ROS Software &#9925;

The autonomous golf cart is a project at the Cal Poly Robotics Club focused on producing a golf cart that can drive around campus without human intervention. This repository contains the software necessary for communication and control of the golf cart. To learn more about our project, click [here](https://www.calpolyrobotics.com/igvc/).

## Project Structure
The src directory contains multiple directories, each pretaining to a package or a collection of packages.
- **camera_pkg** - contains opencv_pkg and zed_wrapper package
-- **opencv_pkg** - openCV based camera code
-- **zed_wrapper** - ZED camera ROS package
- **communication_pkg** - communication with the Olympus board, joystick control, golf cart GUI
- **control_unit_pkg** - rosbag utilities, control code    
- **gps_pkg** - packages needed for the GPS
- **imu_pkg** - package for the Microstrain 3DM-GX2 IMU
- **lidar_pkg** - packages and scripts related to the LMS111 LiDAR
- **navigation_pkg** - navigation related tools and scripts
- **simulation_pkg** - simulation models for golf cart and sensors
