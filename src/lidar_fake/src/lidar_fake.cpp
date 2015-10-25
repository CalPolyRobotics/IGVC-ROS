#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#define LIDAR_READS 540
//once every 1/25 sec = for 1.2 deg, 500+ measurements per sec
float gaussFcn(float dist, float variance, float mean)
{
	float c = 1/(variance * sqrt(2 * 3.14));
	float d = -0.5 * ((dist - mean) / variance);
	return c * exp(pow(d,2)); 
}
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
  unsigned int num_readings = LIDAR_READS;
  double laser_frequency = LIDAR_READS;
  double ranges[num_readings];
  double intensities[num_readings];
  
    ros::Publisher distances_pub = n.advertise<sensor_msgs::LaserScan>("distances", num_readings);

  int count = 0;
  ros::Rate r(1.0);

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
    
    ros::Time scan_time = ros::Time::now();

    sensor_msgs::LaserScan scan;

    scan.header.stamp = scan_time;
    scan.header.frame_id = "laser_frame";
    scan.angle_min = -2.35619449615;
    scan.angle_max = 2.35619449615;
    scan.angle_increment = 3.14/ num_readings;
    scan.time_increment = (1/ laser_frequency)/ (num_readings);
    scan.range_min = 0.0;
    scan.range_max = 20.0;

    for(unsigned int i = 0; i < num_readings; i++)
    {
	ranges[i] = 5.3;
	//ranges[i] = (float) (rand()/((float)RAND_MAX)* scan.range_max);
	intensities[i] = 100 + count;
    }

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);

    for(unsigned int i = 0; i < num_readings; i++)
    {
	//scan.ranges[i] = gaussFcn(ranges[i],1.0,9.0);
	scan.ranges[i] = 5.3;
	scan.intensities[i] = intensities[i];
	ROS_INFO("Ranges: %f", ranges[i]);
    }

    distances_pub.publish(scan);
    count++;
    r.sleep();
  }


  return 0;
}
