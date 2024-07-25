//
// Created by oem on 24. 7. 25..
//

#ifndef VICON_EXAMPLE_INCLUDE_VICON_HPP_
#define VICON_EXAMPLE_INCLUDE_VICON_HPP_

#include <string>
#include <iostream>
#include <memory>
#include "DataStreamClient.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace ViconDataStreamSDK::CPP;

class Vicon
{
public:
  Vicon(std::string host_address, int buffer_size);

  void initVicon();
  void viconUpdate();

private:
  Client client_;
  std::string host_address_;
  int buffer_size_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr robot_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr object_pose_publisher_;

  void publishData(const std::string& name, const geometry_msgs::msg::Pose& msg);
};

class ViconPublisherNode : public rclcpp::Node {
public:
  ViconPublisherNode();

private:
  void timerCallback();
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr object_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr robot_pose_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  Vicon vicon_;
};

#endif // VICON_EXAMPLE_INCLUDE_VICON_HPP_