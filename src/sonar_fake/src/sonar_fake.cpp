#include "ros/ros.h"
#include <sonar_fake/SonarArray.h>
#include <sstream>
#include <string>

#define NUM_SONAR 6

int main(int argc, char** argv) {

   ros::init(argc, argv, "sonar_fake");

   ros::NodeHandle n;

   ros::Publisher sonar_pub = n.advertise<sonar_fake::SonarArray>("sonar_data", 100);
   ros::Rate loop_rate(10); 

   ROS_INFO("Starting Sonar Fake Node");
   
   
 
   while (ros::ok()) {
      sonar_fake::SonarArray sonar_data;
      sonar_data.distance.clear();

      for(int i = 0; i < NUM_SONAR; i++) {
         //sonar_data.data.push_back(((float)rand()/RAND_MAX) * 1068);
         sonar_data.distance.push_back( 5.3);
      }

      for(int i = 0; i< NUM_SONAR; i++)
      {
	  ROS_INFO("Distances (cm): %f", sonar_data.distance[i]);	
      }

      sonar_pub.publish(sonar_data);
      ros::spinOnce();

      loop_rate.sleep();
   }

   return 0;
}
