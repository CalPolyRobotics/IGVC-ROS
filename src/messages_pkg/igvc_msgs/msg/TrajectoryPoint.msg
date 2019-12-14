Header header

# a [x,y,z] position in the global coordinate frame
geometry_msgs/Point point

# a velocity [x,y,z], but we only care about x which represents longitudinal velocity
geometry_msgs/Vector3 velocity

# an orientation [roll,pitch,yaw] that the vehicle should be facing in when the point is reached
geometry_msgs/Vector3 orientation
