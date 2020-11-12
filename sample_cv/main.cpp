
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <memory>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>

#include "UdpSender.hpp"


using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_driver.lib")
#endif
CYdLidar laser;


int main(int argc, char *argv[]) {
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);


  std::string port;
  ydlidar::init(argc, argv);

  std::map<std::string, std::string> ports = CYdLidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1) {
    it = ports.begin();
    port = it->second;
  } else {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++) {
      ydlidar::console.show("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty()) {
      ydlidar::console.show("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    } else {
      while (ydlidar::ok()) {
        ydlidar::console.show("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size()) {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id) {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  if (!ydlidar::ok()) {
    return 0;
  }

  int baudrate = 512000;

  float frequency = 10.0;
  // std::string info = "Please Input the lidar Scan frequency: ";

  // while (ydlidar::ok()) {
  //   ydlidar::console.show("%s", info.c_str());
  //   std::string number;
  //   std::cin >> number;
  //   frequency = atof(number.c_str());

  //   if (frequency >= 3 &&  frequency <= 15) {
  //     break;
  //   }

  //   info = "The lidar frequency is wrong, please re-enter: ";
  // }

  if (!ydlidar::ok()) {
    return 0;
  }


  laser.setSerialPort(port);
  laser.setSerialBaudrate(baudrate);
  laser.setAutoReconnect(true);//hot plug
  laser.setMaxRange(64.0);
  laser.setMinRange(0.1);
  laser.setMaxAngle(-180);   //180
  laser.setMinAngle(180);  //-180
  laser.setSampleRate(20);
  laser.setScanFrequency(frequency);
  laser.setReversion(true);
  laser.setFixedResolution(false);
  bool ret = laser.initialize();

  if (ret) {
    ret = laser.turnOn();
  }


  if (argc < 2) {
    printf("Usage:  %s server_ip_address [local_id]\n", argv[0]);
    return -1;
  }
  const char* ServerAddress = argv[1];
  const uint16_t DataRxPort = 3101;
  const uint16_t ImageRxPort = 3201;
  const char* LaptopAddress = argv[2];
  UdpSender udpSender(ServerAddress, LaptopAddress, ImageRxPort);

  const uint8_t array_size = 72;

  struct data_packet {
    float deg_array[array_size];
    float range_array[array_size];
  };


///////////////////// Draw map //////////////////////
  int line_spacing = 100;
  int map_width = 800;
  int map_height = 800;

  // Circle
  float max_radius = sqrt(pow(map_width,2) + pow(map_height,2));
  int x_center = int(map_width/2);
  int y_center = int(map_height/2);

  cv::Mat blank_map(map_height, map_width, CV_8UC3, cv::Scalar(0, 0, 0));
  // Draw grid
  for(int i=line_spacing; i<=map_width; i=i+line_spacing){
    cv::line(blank_map, cv::Point(i, 0), cv::Point(i, map_height), cv::Scalar(255, 100,20), 1, 8);
  }
  for(int i=line_spacing; i<=map_height; i=i+line_spacing){
    cv::line(blank_map, cv::Point(0, i), cv::Point(map_width, i), cv::Scalar(255, 100, 20), 1, 8);
  }

  //Draw circle
  for(int r=line_spacing; r<=map_width; r=r+line_spacing){
    cv::circle(blank_map, cv::Point(x_center,y_center), int(r), cv::Scalar(255, 100, 20), 1, 8);
  }

  cv::Mat lidar_map = blank_map.clone();
  float pi = 3.14159;
  float offsetAngle = 5.2*pi/180.0;
  float deg_inc = 1.6;  //2.4

  int i = 0;

  while (ret && ydlidar::ok()) {

    // UDP data
    unsigned char metaDataBuffer[1500];  // MTU of ethernet

    struct data_packet *data_packets;
    data_packets = (struct data_packet*) &(metaDataBuffer[0]);

    bool hardError;
    LaserScan scan;

      
    if (laser.doProcessSimple(scan, hardError)) {
      //printf("scan.ranges.size() = %ld\n", scan.ranges.size());

      int inc=0;
      float minAngRange = -60.0;
      float maxAngRange = 60.0;
      float prev_deg = minAngRange - deg_inc;

      for (int i = 0; i < scan.ranges.size(); i++) {
        float angle = scan.config.min_angle + i * scan.config.ang_increment + offsetAngle ;
        float dis = scan.ranges[i]*2.0;  // multiply with 2 will get correct distance in meter
        float x = dis*sin(angle - pi);
        float y = dis*cos(angle - pi);

        float x_pixel = int(x*line_spacing + (map_width/2));
        float y_pixel = int(y*line_spacing + (map_height/2));

        if (x_pixel > 1 && (x_pixel < (map_width-2))){
          if (y_pixel > 1 && (y_pixel < (map_height-2))){
            cv::Vec3b color = lidar_map.at<cv::Vec3b>(cv::Point(x_pixel,y_pixel));
            color[0] = 0;
            color[1] = 0;
            color[2] = 255;
            lidar_map.at<cv::Vec3b>(cv::Point(x_pixel,y_pixel)) = color;
          }
        }

        float deg = angle*180.0/pi;
       

        // we focus on front side (front half cirle) of the lidar
        if ((deg >= (minAngRange - deg_inc)) && (deg <= (maxAngRange + deg_inc)) && (inc < array_size)) {
           //printf("deg: %f\n", deg);
          if (abs(deg - prev_deg) >= deg_inc) {
            // printf("inc: %d  deg: %f  dist: %f\n", inc, deg, dis);
            data_packets->deg_array[inc] = deg;
            data_packets->range_array[inc] = dis;
            inc++;
            prev_deg = deg;
          }
        }

      
      }

      int dataSize = sizeof(struct data_packet);
      //printf("%d\n", dataSize);
      udpSender._sendto(ServerAddress, DataRxPort, metaDataBuffer, dataSize);
      

      //cv::imshow("Lidar map", lidar_map);
      udpSender.sendImage(lidar_map);


      lidar_map = blank_map.clone(); 
      if( cv::waitKey(1) == 27 ) break;
      
      // ydlidar::console.message("Scan received[%llu]: %u ranges is [%f]Hz",
      //                          scan.self_time_stamp, (unsigned int)scan.ranges.size(),
      //                          1.0 / scan.config.scan_time);
      
    } else {
      ydlidar::console.warning("Failed to get Lidar Data");
    }

  }

  laser.turnOff();
  laser.disconnecting();
  return 0;


}
