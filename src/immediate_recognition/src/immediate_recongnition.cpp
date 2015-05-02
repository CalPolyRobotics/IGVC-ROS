#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/LaserScan.h"

#define NUM_SONAR 6
#define NUM_LIDAR 1

void sonarCallback(const std_msgs::Int32MultiArray msg) {
    for(int i = 0; i < sizeof(msg); i++)
    {
        ROS_INFO("hello");
        if(msg.data[i] < 100)
        {
            ROS_INFO("Value too low");
        } 
        else
        {
	    ROS_INFO("hello");
            printf("hello");
            ROS_INFO("Sonar: [%d]", msg.data[i]);
        }
    }
}

void lidarCallback(const sensor_msgs::LaserScan msg)
{
    for( unsigned int j = 0; j < sizeof(msg.ranges); j++)
    {
        ROS_INFO("hello again");
        if (msg.ranges[j] < 5.0)
        {
            ROS_INFO("about to crash");
        }
        else
        {
	    ROS_INFO("hello again");
            ROS_INFO("Lidar: [%f]", msg.ranges[j]);
        } 
    }
}

int main(int argc, char** argv) {

   ros::init(argc, argv, "immediate_response");

   ros::NodeHandle n;
   ROS_INFO("receving data");
   ros::Subscriber sub_sonar = n.subscribe("sonar_data", 1000, sonarCallback);
   ros::Subscriber sub_lidar = n.subscribe("lidar_data", 50, lidarCallback);

   ros::spin();

   return 0;
}
