#include <csignal>
#include <cstdio>
#include <LMS1xx.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"

#define DEG2RAD M_PI/180.0

void interpretRanges(std::vector<float> ranges){
  // Min Distance between object and lidar 
  const float MIN = 1.0;

  // What Angle to begin and end information processing
  // The Max End Point is 270
  // 180 Degrees is 45 to 225
  const int ANG_START = 0;
  const int ANG_END = 270; 
  
  // Translate angles to index 2 per angle
  const int START = ANG_START * 2;
  const int END = ANG_END * 2;

  // Reading with 0 degrees starting on the right when sitting behind the lidar
  bool tooClose[ranges.size()];

  for(int i = 0; i < ranges.size(); i++){
     if(ranges[i] < MIN){
        tooClose[i] = true;
     }else{
        tooClose[i] = false; 
     } 
  }
  
  for(int i = START; i <= END; i++){
     if(tooClose[i]){
        std::cout << "1"; 
     }else{
        std::cout << "0"; 
     }
  }
  std::cout << "\n";
}

int main(int argc, char **argv){
  // Laser Data 
  LMS1xx laser;
  scanCfg cfg;
  scanDataCfg dataCfg;
  scanData data;

  // Publish Data 
  sensor_msgs::LaserScan scan_msg;
  std_msgs::Float32MultiArray range_msg;

  // Parameters 
  std::string host;
  std::string frame_id;

  ros::init(argc, argv, "lms1xx");
  ros::NodeHandle nh;
  ros::NodeHandle n("~");
  ros::NodeHandle nodeHandler;
  ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1);
  
  // Publisher Setup
  ros::Publisher lidar_pub = nodeHandler.advertise<std_msgs::Float32MultiArray>("ranges", 100);

  n.param<std::string>("host", host, "192.168.1.5");
  n.param<std::string>("frame_id", frame_id, "laser");

  ROS_INFO("connecting to laser at : %s", host.c_str());

  // Initialize Hardware 
  laser.connect(host);

  if (laser.isConnected())
  {
    ROS_INFO("Connected to laser.");

    laser.login();
    cfg = laser.getScanCfg();

    scan_msg.header.frame_id = frame_id;

    scan_msg.range_min = 0.01;
    scan_msg.range_max = 20.0;

    //scanningFrequency?
    scan_msg.scan_time = 100.0/cfg.scaningFrequency;

    scan_msg.angle_increment = (double)cfg.angleResolution/10000.0 * DEG2RAD;
    scan_msg.angle_min = (double)cfg.startAngle/10000.0 * DEG2RAD - M_PI/2;
    scan_msg.angle_max = (double)cfg.stopAngle/10000.0 * DEG2RAD - M_PI/2;

    std::cout << "resolution : " << (double)cfg.angleResolution/10000.0 << " deg " << std::endl;
    std::cout << "frequency : " << (double)cfg.scaningFrequency/100.0 << " Hz " << std::endl;

    int num_values;
    if (cfg.angleResolution == 2500)
    {
      num_values = 1081;
    }
    else if (cfg.angleResolution == 5000)
    {
      num_values = 541;
    }
    else
    {
      ROS_ERROR("Unsupported resolution");
      return 0;
   }

    // Set the Size of the data array
    range_msg.data.resize(num_values);

    scan_msg.time_increment = scan_msg.scan_time/num_values;

    scan_msg.ranges.resize(num_values);
    scan_msg.intensities.resize(num_values);

    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    laser.setScanDataCfg(dataCfg);

    laser.startMeas();

    status_t stat;
    do // wait for ready status
    {
      stat = laser.queryStatus();
      ros::Duration(1.0).sleep();
    }
    while (stat != ready_for_measurement);

    laser.startDevice(); // Log out to properly re-enable system after config
    laser.scanContinous(1);
    
    while (ros::ok()){

      ros::Time start = ros::Time::now();

      scan_msg.header.stamp = start;
      ++scan_msg.header.seq;

      //bool gotpackat =false;

      /// added Reconnection to  ensure that
      // while(gotpackat == false)
      //{
        try
        {
            laser.getData(data);
            //gotpackat = true;
            
            for (int i = 0; i < data.dist_len1; i++){
                scan_msg.ranges[i] = data.dist1[i] * 0.001;
                range_msg.data[i] = scan_msg.ranges[i];
                
                // std::cout << scan_msg.ranges[i];
            }

            lidar_pub.publish(range_msg);
            //interpretRanges(scan_msg.ranges);

            for (int i = 0; i < data.rssi_len1; i++){
                scan_msg.intensities[i] = data.rssi1[i];
            }

            scan_pub.publish(scan_msg);
        }catch (int e){
            std::cout << "connection lost reconnecting\n";
            //laser.disconnect();
            //laser.connect(host);
            //laser.startDevice(); // Log out to properly re-enable system after config
            //laser.scanContinous(1);
        }
      //}

      ros::spinOnce();
    }

    laser.scanContinous(0);
    laser.stopMeas();
    laser.disconnect();
  }
  else
  {
    ROS_ERROR("Connection to device failed");
  }
  return 0;
}



