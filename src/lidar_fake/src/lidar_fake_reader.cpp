#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan data;
	ROS_INFO( "hello" );
}

int main(int argc, char** argv)
{
	ROS_INFO("Starting lidar receiver");
	ros::init(argc, argv, "lidar_fake_reader");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("scan",1000, scanCallback);
	ros::spin();
	return 0;
	
}
