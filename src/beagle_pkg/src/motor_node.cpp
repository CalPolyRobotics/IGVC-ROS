#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "beagle_pkg/status.h"


void statusCallback(const beagle_pkg::status::ConstPtr& msg) {
   ROS_INFO("I heard: [%d]", (int) msg->command);
   switch(msg->command) {
      case 0:
         ROS_INFO("The golf cart will keep moving");
         break;
      default:
         ROS_INFO("The golf cart will stop");
         break;
   }
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "motor_node");
	
	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("vel", 1000, statusCallback);

   ros::spin();

   return 0;
}


