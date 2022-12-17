#ifndef SERIALPORT_NODE_HPP
#define SERIALPORT_NODE_HPP
#pragma once 

//ros
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// #include "./serialport_old.hpp"
#include "./serialport.hpp"

// #include "../../../global_user/include/coordsolver.hpp"
// #include "global_interface/msg/imu.hpp"
// #include "global_interface/msg/gimbal.hpp"
// #include "global_interface/msg/target.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

namespace seriaport
{
  
  class serial_driver : public rclcpp::Node
  {
  public:
    serial_driver(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~serial_driver();
  
  public:
    
  

  }

}