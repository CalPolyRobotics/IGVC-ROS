#include "ros/ros.h"
#include "beagle_pkg/dem.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_client");
  if (argc != 2)
  {
    ROS_INFO("usage: demo_client num");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beagle_pkg::dem>("demo");
  beagle_pkg::dem srv;
  srv.request.command = atoll(argv[1]);
  //srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Response from Server: %ld", (long int)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
