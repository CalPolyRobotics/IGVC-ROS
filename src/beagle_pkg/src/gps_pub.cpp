#include <iostream>
#include <ros/ros.h>
#include <gps_common/GPSFix.h>


class GpsTest
{


public:
  // Type for GPS messages
  gps_common::GPSFix gpsMsg;


  // Constructor
  GpsTest(ros::NodeHandle nh_) : n(nh_)
  {
    // Subscribing to the topic /fix
    gps_sub = n.subscribe("/fix", 100, &GpsTest::gpsCallback, this);
  }


  // Callback Function for the GPS
  void gpsCallback(const gps_common::GPSFixConstPtr &msg)
  {
    gpsMsg = *msg;
  }


private:
  // Nodehandle
  ros::NodeHandle n;

  // Subscriber
  ros::Subscriber gps_sub;
};


int main(int argc, char** argv)
{

  // Variables to store the Latitude and Longitude from the GPS respectively
  double gpsLat = 0;
  double gpsLong = 0;

  // Initializing the node for the GPS
  ros::init(argc, argv, "gps_Subscriber");
  ros::NodeHandle nh_;


  GpsTest *p = new GpsTest(nh_);


  // Getting the data from the GPS
  gpsLong = p->gpsMsg.longitude;
  gpsLat = p->gpsMsg.latitude;

  std::cout << "Current Latitude: " << gpsLat << std::endl;
  std::cout << "Current Longitude " << gpsLong << std::endl;

  ros::spin();
  return 0;
}
