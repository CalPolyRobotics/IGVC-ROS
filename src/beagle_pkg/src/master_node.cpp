#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "beagle_pkg/ard.h"
#include "beagle_pkg/status.h"
//#include "beagle_pkg/ArdMsg.h"

sensor_msgs::LaserScan scan_data;
sensor_msgs::NavSatFix gps_data;
beagle_pkg::ard ard_data;
ros::Publisher vel_pub;
ros::ServiceClient ard_client;

void processData();

void laser_scanner_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {
   scan_data = *scan;
}

void gps_callback(const sensor_msgs::NavSatFix::ConstPtr& scan) {
   gps_data = *scan;
}

/*void ard_callback(const beagle_pkg::ard::ConstPtr& msg) {
   ard_data = *msg;
}*/

/**
 * This tutorial demonstrateardDatas simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

   ros::init(argc, argv, "BeagleMaster");
   ros::NodeHandle n;

   /* Make BeagleMaster a publisher of the motor controller ros msg.
   * Ros provides this to us via the Twist message, which includes
   * two vectors: one corresponding to linear velocity (linear), and 
   * one corresponding to angular velocity (angular)
   */
   
   //vel_pub = n.advertise<geometry_msgs::Twist>("vel", 1000);

   /* Will eventually change. Currently set to our premade msg file
    * to check communication with motor node.
    */
   vel_pub = n.advertise<beagle_pkg::status>("vel", 1000);


   /* Make BeagleMaster a subscriber to lidar topic. This is provided by
   * Ros's LaserScan message. the topic was created from an external library
   */
   ros::Subscriber laser_sub = n.subscribe<sensor_msgs::LaserScan>("scan", 10, laser_scanner_callback);

   /* Make BeagleMaster a subscriber to gps topic. This is provided by
   * Ros's NavSatFix message.
   */
   //ros::Subscriber gps_sub = n.subscribe<sensor_msgs::NavSatFix>("gps", 10, gps_callback);

   /* Make BeagleMaster a subscriber to arduino topic. */
   ard_client = n.serviceClient<beagle_pkg::ard>("ardData");

   ros::Rate loop_rate(10);

   /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
   int count = 0;
   while (ros::ok())
   {
      //Check for updates to msg files.
      ros::spinOnce();
    
      ard_data.request.command = 1;

      processData();


      loop_rate.sleep();
      ++count;
   }

   return 0;
}

/* processData is the main decision function that
 * will publish commands to the motor controller
 * node and possibly ask for arduino data.
 */
void processData() {

    double minDist=10;
    double angle;
    double speed=0.5;
   //geometry_msgs::Twist command;
   beagle_pkg::status status;
   if(ard_client.call(ard_data)) {


       ROS_INFO("lidar data was %f, arduino reading was %f\n", scan_data.range_min, ard_data.response.distance);
      for (unsigned int i=0;i< scan_data.ranges.size();i++){

        angle = ((double)i*scan_data.angle_increment)+scan_data.angle_min;

         if (scan_data.ranges[i]< minDist){
             minDist=scan_data.ranges[i];
             std::cout << minDist << std::endl;
         }
       }

      if(minDist  <1.0 ) {
         status.command = 1;
      }
      else
         status.command = -1;

      /* TODO: look at global data (scan_data, gps_data, ard_data)
       * and make decision of what to send to the motor node.
       */
      vel_pub.publish(status);
   }
   else {
    ROS_ERROR("Failed to call ard service");
   }
}

