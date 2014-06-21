#include <string>
#include <cstdlib>
#include "ros/ros.h"
#include "beagle_pkg/dem.h"

bool procNote(beagle_pkg::dem::Request &req,
              beagle_pkg::dem::Response &res) {
   ROS_INFO("Request: %d",  req.command);
   
   switch(rec.command) {
    case 1:
      return rand()

   }
   return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "demo_server");
  ros::NodeHandle n;

  //We will change this later, just meant for testing
  srand(time(NULL));

  ros::ServiceServer service = n.advertiseService("ardData", procNote);
  ROS_INFO("Ready to receive demo msg.");
  ros::spin();

  return 0;
}



