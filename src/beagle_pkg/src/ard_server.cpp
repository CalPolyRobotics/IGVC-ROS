#include <string>
#include <cstdlib>
#include <ctime>
#include "ros/ros.h"
#include "beagle_pkg/ard.h"

/* Callback function that will process any command given
 * by the master node.
 */
bool procCommand(beagle_pkg::ard::Request &req,
              beagle_pkg::ard::Response &res) {
   
   ROS_INFO("Request: %d",  (int) req.command);
   
   switch(req.command) {
    case 1:
      res.distance = rand()%20;
      ROS_INFO("Sending: %f", res.distance);
      break;
   case 2:
      res.distance = rand()%20;
      ROS_INFO("Sending: %f", res.distance);
      break;
    default:
      res.distance = 0;
      break;
   }

   return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ard_server");
  ros::NodeHandle n;

  //We will change this later, just meant for testing
  srand(time(NULL));

  ros::ServiceServer service = n.advertiseService("ardData", procCommand);
  ROS_INFO("Ready to receive master requests.");
  ros::spin();

  return 0;
}



