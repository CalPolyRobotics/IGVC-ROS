#######################################################
nav_pkg
#######################################################

Launch Files:
    odometry.launch - Launches Odometry Node and has an argument for switching IMU direction.
        User params:
            imu_direction: 
                +/- 1 to reverse IMU direction
                default: +1
    gmapping.launch - Launches SLAM gmapping node to generate maps
        Tweak parameters as necessary to generate better maps

Scripts:
    odometry.py - creates simple odometry based on IMU data and wheel encoder data.
        - Uses imu_direction parameter. 
            default: +1
    wheel_odometry.py - creates simple odometry only based on wheel encoder data.

Source Files:
---

