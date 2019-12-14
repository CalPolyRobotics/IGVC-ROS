Header header

# an id to label the point with
int32 id

# a [x,y,z] position in the global coordinate frame
geometry_msgs/Point point

# a boolean denoting whether the point is on the road or not
std_msgs/Bool onRoad

# a list of Point msg ids that the point is adjacent to in the map
int32[] edges
