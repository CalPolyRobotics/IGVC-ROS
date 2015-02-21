#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"

#include <sstream>
#define LIDAR_READS 540

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_fake");
 
  ros::NodeHandle n;
  
  ros::Publisher distances_pub = n.advertise<std_msgs::Int16MultiArray>("distances", 1000);

  ros::Rate loop_rate(25);

  ROS_INFO("Starting Lidar");//console print out

  //publish data

  while(ros::ok())
  {
    std_msgs::Int16MultiArray msg;
    msg.data.clear();
    for( int i = 0; i < LIDAR_READS; i++)
    {
	msg.data.push_back(rand() % 255);
    }
    
    distances_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
