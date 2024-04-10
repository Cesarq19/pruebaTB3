/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>

#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include "compensator_component.h"

#define ROS2Verision "1.0.1"


int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s\n", ((std::string)ROS2Verision).c_str());

  CYdLidar laser;
  std::string str_optvalue;
  node->declare_parameter<std::string>("port", "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0");
  node->get_parameter_or<std::string>("port", str_optvalue, "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0");
  ///lidar port
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  ///ignore array
  ///std::string str_optvalue;
  node->declare_parameter<std::string>("ignore_array", "");
  node->get_parameter_or<std::string>("ignore_array", str_optvalue, "");
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  std::string frame_id;
  node->declare_parameter<std::string>("frame_id", "/laser_frame");
  node->get_parameter_or<std::string>("frame_id", frame_id, "/laser_frame");

  //////////////////////int property/////////////////
  /// lidar baudrate
  int optval;
  node->declare_parameter<int>("baudrate", 230400);
  node->get_parameter_or<int>("baudrate", optval, 230400);
  laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
  /// tof lidar
  ///int optval;
  node->declare_parameter<int>("lidar_type", TYPE_TRIANGLE);
  node->get_parameter_or<int>("lidar_type", optval, TYPE_TRIANGLE);
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  ///int optval;
  node->declare_parameter<int>("device_type", YDLIDAR_TYPE_SERIAL);
  node->get_parameter_or<int>("device_type", optval, YDLIDAR_TYPE_SERIAL);
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  ///int optval;
  node->declare_parameter<int>("sample_rate", 4);
  node->get_parameter_or<int>("sample_rate", optval, 4);
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  ///int optval;
  node->declare_parameter<int>("abnormal_check_count", 4);
  node->get_parameter_or<int>("abnormal_check_count", optval, 4);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
     

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue;
  node->declare_parameter<bool>("fixed_resolution", true);
  node->get_parameter_or<bool>("fixed_resolution", b_optvalue, true);
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));


  /// rotate 180
  ///bool b_optvalue;
  node->declare_parameter<bool>("reversion", false);
  node->get_parameter_or<bool>("reversion", b_optvalue, false);
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  ///bool b_optvalue;
  node->declare_parameter<bool>("inverted", true);
  node->get_parameter_or<bool>("inverted", b_optvalue, true);
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  ///bool b_optvalue;
  node->declare_parameter<bool>("auto_reconnect", true);
  node->get_parameter_or<bool>("auto_reconnect", b_optvalue, true);
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  ///bool b_optvalue;
  node->declare_parameter<bool>("isSingleChannel", false);
  node->get_parameter_or<bool>("isSingleChannel", b_optvalue, false);
  laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
  /// intensity
  ///bool b_optvalue;
  node->declare_parameter<bool>("intensity", true);
  node->get_parameter_or<bool>("intensity", b_optvalue, true);
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  ///bool b_optvalue;
  node->declare_parameter<bool>("support_motor_dtr", false);
  node->get_parameter_or<bool>("support_motor_dtr", b_optvalue, false);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue;
  node->declare_parameter<float>("angle_max", 180.0f);
  node->get_parameter_or<float>("angle_max", f_optvalue, 180.0f);
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  ///f_optvalue;
  node->declare_parameter<float>("angle_min", -180.0f);
  node->get_parameter_or<float>("angle_min", f_optvalue, -180.0f);
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  ///f_optvalue;
  node->declare_parameter<float>("range_max", 64.f);
  node->get_parameter_or<float>("range_max", f_optvalue, 64.f);
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  ///f_optvalue;
  node->declare_parameter<float>("range_min", 0.5f);
  node->get_parameter_or<float>("range_min", f_optvalue, 0.5f);
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  ///f_optvalue = 10.f;
  node->declare_parameter<float>("frequency", 6.f);
  node->get_parameter_or<float>("frequency", f_optvalue, 6.f);
  laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

  bool invalid_range_is_inf;
  node->declare_parameter<bool>("invalid_range_is_inf", false);
  node->get_parameter_or<bool>("invalid_range_is_inf", invalid_range_is_inf, false);



  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }
  
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 100);

  auto stop_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOff();
  };

  auto stop_service = node->create_service<std_srvs::srv::Empty>("stop_scan",stop_scan_service);

  auto start_scan_service =
    [&laser](const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<std_srvs::srv::Empty::Request> req,
  std::shared_ptr<std_srvs::srv::Empty::Response> response) -> bool
  {
    return laser.turnOn();
  };

  auto start_service = node->create_service<std_srvs::srv::Empty>("start_scan",start_scan_service);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {

    LaserScan scan;//

    if (laser.doProcessSimple(scan)) {

      auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
      //LaserScan scan_comp = compensator_component.LaserScanMsgCallback(scan);

      scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
      scan_msg->header.stamp.nanosec =  scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min = scan.config.min_angle;
      scan_msg->angle_max = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min = scan.config.min_range;
      scan_msg->range_max = scan.config.max_range;
      
      int size = (scan.config.max_angle - scan.config.min_angle)/ scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);
      for(size_t i=0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle)/scan.config.angle_increment);
        if(index >=0 && index < size) {
          scan_msg->ranges[index] = scan.points[i].range;
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }

      laser_pub->publish(*scan_msg);


    } else {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
    }
    if(!rclcpp::ok()) {
      break;
    }
    ///rclcpp::spin_some(node);
    loop_rate.sleep();
  }


  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
