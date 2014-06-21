#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


int main(int argc, char **argv) {
   ros::init(argc, argv, "lidar_publisher");

   ros::NodeHandle n;
   
   ros::Publisher lidar_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

   ros::Rate loop_rate(10);   

   int count = 0;
   while (ros::ok())
   {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
      sensor_msgs::LaserScan msg;

      //Just for testing, will change later
      msg.range_min = rand()%10;

      lidar_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
   }

   return 0;

}
