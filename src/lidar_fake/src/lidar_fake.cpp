#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"

#include <sstream>
#define LIDAR_READS 540

int main(int argc, char **argv)
{
  /*
   * Use ros::init() function to initialize ROS.cpp
   * pass in argc and argv so that ROS can process arguements
   * name the node "lidar_fake"
  */
  ros::init(argc, argv, "lidar_fake");
 
  /* Use NodeHandle to write to nodes
   * instantiates everything necessary 
   * for the Node 
   * main communications point for ROS network
   * initializes node and allows for communication
   * with other nodes
  */
  ros::NodeHandle n;
  
  /*
   * Create a publisher to tell ROS network to publish
   * on a given topic.
   * <std_msgs:: ... > = standard ROS messages representing
   * primitive data types and mutli-arrays
   * Int16MultiArray = array of data
   * broadcasts an array of data
   * publishes on "distances"
   * creates a 1000 (char...?) buffer
  */
  ros::Publisher distances_pub =
      n.advertise<std_msgs::Int16MultiArray>("distances", 1000);

  /*
   * ros::Rate maintains a particular 
   * rate for a loop
   * maintains a rate of 25 Hz
  */
  ros::Rate loop_rate(25);

  /*
   * prints out to console
   * ROS_INFO is similar to std_out_println 
   * or printf/cout
  */
  ROS_INFO("Starting Lidar");

  /*
   * while ros::ok is true...
   * meaning while ROS system is still running
   * create a std_msgs multi array and clear data
   * then fill std_msgs with 540 random numbers
   * between 0 and 255.
   * Publishes std_msgs multi array to all subsribers.
  */
  while(ros::ok())
  {
    std_msgs::Int16MultiArray msg;

    /*
     * msg.data is a multi array
    */
    msg.data.clear();

    /*
     * Appends a random number from 0 - 255
     * to msgs (the multi array of lidar data)
     * Does 540 times
    */
    for( int i = 0; i < LIDAR_READS; i++)
    {
	msg.data.push_back(rand() % 255);
    }
    
    /*
     * publishes lidar distance data 
     * to all subscribers
    */
    distances_pub.publish(msg);

    /* 
     * spinOnce used for callbacks
     * to make sure callbacks are called
    */
    ros::spinOnce();

    /*
     * put ROS rate object to sleep
    */
    loop_rate.sleep();
  }


  return 0;
}
