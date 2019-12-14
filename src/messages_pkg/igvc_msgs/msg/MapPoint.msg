Header header

# an id to label the point with
int32 id

# a [x,y,z] position in the global coordinate frame
geometry_msgs/Point point

# a boolean denoting whether the point is on the road or not
std_msgs/Bool onRoad

# a boolean denoting whether the point has been visited in a graph search
Bool visited

# a list of Point msg ids that the point is adjacent to in the map
int32[] edges
