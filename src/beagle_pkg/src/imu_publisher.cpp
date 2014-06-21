#include "3dmgx2.h"
#include <iostream>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "beagle_pkg/imu_msg.h"
#include "ros/ros.h"

//using std::cerr;
//using std::cout;
//using std::endl;

int main(int argc, char **argv) {
   microstrain_3dmgx2_imu::IMU imu;
   imu.openPort("/dev/ttyUSB0");
   imu.initTime(0);
   imu.setContinuous(microstrain_3dmgx2_imu::IMU::CMD_EULER);
   ros::init(argc, argv, "imu_publisher");

   ros::NodeHandle n;
   
   ros::Publisher imu_pub = n.advertise<beagle_pkg::imu_msg>("imu_data", 1000);

   ros::Rate loop_rate(10);    

   while (ros::ok()) {
      double roll, pitch, yaw;
      uint64_t time;
		imu.receiveEuler(&time, &roll, &pitch, &yaw);
      beagle_pkg::imu_msg msg;
      msg.roll = roll;
      msg.pitch = pitch;
      msg.yaw = yaw;
      imu_pub.publish(msg);

      ros::spinOnce();

      loop_rate.sleep();
		//printf("roll: %.5lf | pitch: %.5lf\n", roll, pitch);
     // usleep(5000);
   }

   imu.closePort();
}
