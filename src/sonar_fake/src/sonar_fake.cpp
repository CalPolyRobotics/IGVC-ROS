#include "ros/ros.h"
#include "std_msgs/UInt32.h"

#include <sstream>
#include <string>

#define NUM_SONAR 6

using namespace std;

int main(int argc, char** argv) {

   char tempStr[20];

   ros::init(argc, argv, "sonar_fake");

   ros::NodeHandle n;

   ros::Publisher sonar_pub[NUM_SONAR];
   int i;
   for(i = 0; i < NUM_SONAR; i++) { 
      string msgPrefix = "sonar/fake/";
      sprintf(tempStr, "%d", i); 
      sonar_pub[i] = n.advertise<std_msgs::UInt32>(msgPrefix + tempStr, 10); 
   } 
   ros::Rate loop_rate(10); 

   ROS_INFO("Starting Sonar Fake Node");
   
   while (ros::ok()) { 
      std_msgs::UInt32 data;

      for(i = 0; i < NUM_SONAR; i++) {
         data.data = (unsigned int)(1000 + i);
         sonar_pub[i].publish(data);
      }

      ros::spinOnce();

      loop_rate.sleep();
   }

   return 0;
}
