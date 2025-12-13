/*
 * YDLIDAR SYSTEM
 * YDLIDAR ROS 2 Node
 *
 * Copyright 2015 - 2023 EAI TEAM
 * http://www.ydlidar.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "src/CYdLidar.h"
#include <vector>
#include <iostream>
#include <string>
#include <signal.h>
#include <math.h>

using namespace std;

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: 1.0.1");

  CYdLidar laser;

  // --- DECLARAÇÃO DE PARÂMETROS CORRIGIDA PARA ROS 2 HUMBLE ---
  // É obrigatório definir o tipo (<std::string>, <int>, etc) e um valor padrão.

  // Strings
  std::string str_opt;
  node->declare_parameter<std::string>("port", "/dev/ydlidar");
  node->declare_parameter<std::string>("frame_id", "laser_frame");
  node->declare_parameter<std::string>("ignore_array", "");

  // Inteiros
  int int_opt;
  node->declare_parameter<int>("baudrate", 115200);
  node->declare_parameter<int>("lidar_type", 1);
  node->declare_parameter<int>("device_type", 0);
  node->declare_parameter<int>("sample_rate", 3);
  node->declare_parameter<int>("abnormal_check_count", 4);
  node->declare_parameter<int>("intensity_bit", 0);
  
  // Booleanos
  bool bool_opt;
  node->declare_parameter<bool>("fixed_resolution", false);
  node->declare_parameter<bool>("reversion", true);
  node->declare_parameter<bool>("inverted", true);
  node->declare_parameter<bool>("auto_reconnect", true);
  node->declare_parameter<bool>("isSingleChannel", false);
  node->declare_parameter<bool>("intensity", false);
  node->declare_parameter<bool>("support_motor_dtr", true);
  node->declare_parameter<bool>("debug", false);
  node->declare_parameter<bool>("invalid_range_is_inf", false);

  // Floats
  float float_opt;
  node->declare_parameter<float>("angle_max", 180.0f);
  node->declare_parameter<float>("angle_min", -180.0f);
  node->declare_parameter<float>("range_max", 64.0f);
  node->declare_parameter<float>("range_min", 0.1f);
  node->declare_parameter<float>("frequency", 10.0f);
  
  // Parâmetros extras
  node->declare_parameter<int>("m1_mode", 0);
  node->declare_parameter<int>("m2_mode", 0);


  // --- LEITURA E CONFIGURAÇÃO DA SDK ---

  node->get_parameter("port", str_opt);
  laser.setlidaropt(LidarPropSerialPort, str_opt.c_str(), str_opt.size());

  node->get_parameter("ignore_array", str_opt);
  laser.setlidaropt(LidarPropIgnoreArray, str_opt.c_str(), str_opt.size());

  std::string frame_id;
  node->get_parameter("frame_id", frame_id);

  node->get_parameter("baudrate", int_opt);
  laser.setlidaropt(LidarPropSerialBaudrate, &int_opt, sizeof(int));

  node->get_parameter("lidar_type", int_opt);
  laser.setlidaropt(LidarPropLidarType, &int_opt, sizeof(int));

  node->get_parameter("device_type", int_opt);
  laser.setlidaropt(LidarPropDeviceType, &int_opt, sizeof(int));

  node->get_parameter("sample_rate", int_opt);
  laser.setlidaropt(LidarPropSampleRate, &int_opt, sizeof(int));

  node->get_parameter("abnormal_check_count", int_opt);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &int_opt, sizeof(int));

  node->get_parameter("intensity_bit", int_opt);
  laser.setlidaropt(LidarPropIntenstiyBit, &int_opt, sizeof(int));

  node->get_parameter("fixed_resolution", bool_opt);
  laser.setlidaropt(LidarPropFixedResolution, &bool_opt, sizeof(bool));

  node->get_parameter("reversion", bool_opt);
  laser.setlidaropt(LidarPropReversion, &bool_opt, sizeof(bool));

  node->get_parameter("inverted", bool_opt);
  laser.setlidaropt(LidarPropInverted, &bool_opt, sizeof(bool));

  node->get_parameter("auto_reconnect", bool_opt);
  laser.setlidaropt(LidarPropAutoReconnect, &bool_opt, sizeof(bool));

  node->get_parameter("isSingleChannel", bool_opt);
  laser.setlidaropt(LidarPropSingleChannel, &bool_opt, sizeof(bool));

  node->get_parameter("intensity", bool_opt);
  laser.setlidaropt(LidarPropIntenstiy, &bool_opt, sizeof(bool));

  node->get_parameter("support_motor_dtr", bool_opt);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &bool_opt, sizeof(bool));

  node->get_parameter("angle_max", float_opt);
  laser.setlidaropt(LidarPropMaxAngle, &float_opt, sizeof(float));

  node->get_parameter("angle_min", float_opt);
  laser.setlidaropt(LidarPropMinAngle, &float_opt, sizeof(float));

  node->get_parameter("range_max", float_opt);
  laser.setlidaropt(LidarPropMaxRange, &float_opt, sizeof(float));

  node->get_parameter("range_min", float_opt);
  laser.setlidaropt(LidarPropMinRange, &float_opt, sizeof(float));

  node->get_parameter("frequency", float_opt);
  laser.setlidaropt(LidarPropScanFrequency, &float_opt, sizeof(float));

  bool invalid_range_is_inf = false;
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);


  // Inicialização do LiDAR
  bool ret = laser.initialize();
  if (ret) {
    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s\n", laser.DescribeError());
  }

  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());

  rclcpp::Rate loop_rate(30);

  while (ret && rclcpp::ok()) {
    LaserScan scan;
    if (laser.doProcessSimple(scan)) {
      auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();

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

      int size = (scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment + 1;
      scan_msg->ranges.resize(size);
      scan_msg->intensities.resize(size);

      for (size_t i = 0; i < scan.points.size(); i++) {
        int index = std::ceil((scan.points[i].angle - scan.config.min_angle) / scan.config.angle_increment);
        if (index >= 0 && index < size) {
          if (scan.points[i].range == 0.0) {
            if (invalid_range_is_inf) {
              scan_msg->ranges[index] = std::numeric_limits<float>::infinity();
            } else {
              scan_msg->ranges[index] = 0.0;
            }
          } else {
            scan_msg->ranges[index] = scan.points[i].range;
          }
          scan_msg->intensities[index] = scan.points[i].intensity;
        }
      }
      laser_pub->publish(std::move(scan_msg));
    } else {
      RCLCPP_WARN(node->get_logger(), "Failed to get scan");
    }
    
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();

  return 0;
}
