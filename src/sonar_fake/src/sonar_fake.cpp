#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

#include <sstream>
#include <string>

#define NUM_SONAR 6

int main(int argc, char** argv) {

   ros::init(argc, argv, "sonar_fake");

   ros::NodeHandle n;

   ros::Publisher sonar_pub = n.advertise<std_msgs::Int32MultiArray>("data", 100);
   ros::Rate loop_rate(10); 

   ROS_INFO("Starting Sonar Fake Node");
   
   while (ros::ok()) { 
      std_msgs::Int32MultiArray data;

      data.data.clear();

      for(int i = 0; i < NUM_SONAR; i++) {
         data.data[i] = 128;
      }
      sonar_pub.publish(data);
      ros::spinOnce();

      loop_rate.sleep();
   }

   return 0;
}
