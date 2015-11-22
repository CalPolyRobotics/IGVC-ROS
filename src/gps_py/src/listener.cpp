#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

void callback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  double num  = scan_in->ranges[0];
  std::cout << "Element 0: " << num  << " " << std::endl;
  ROS_INFO("Received Message");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("scan", 1000, callback);
  ros::spin();
  return 0;
}
