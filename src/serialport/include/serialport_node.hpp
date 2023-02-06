#ifndef SERIALPORT_NODE_HPP
#define SERIALPORT_NODE_HPP
#pragma once 

//ros
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <message_filters/subscriber.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

// #include "./serialport_old.hpp"
#include "./serialport.hpp"

#include "../../../global_user/include/coordsolver.hpp"
// #include "global_interface/msg/imu.hpp"
// #include "global_interface/msg/gimbal.hpp"
#include "global_interface/msg/target.hpp"

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace coordsolver;
using namespace global_user;

namespace seriaport
{
  
  class serial_node : public rclcpp::Node
  {
    typedef global_interface::msg::Target TargetMsg;
    typedef global_interface::msg::Stone StoneMsg;
  public:
    serial_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~serial_node();
  
  public:
    void receive_data();
    void send_stone_data(StoneMsg::SharedPtr msg);
    void send_station_data(TargetMsg::SharedPtr msg);

  public:
    rclcpp::Publisher<JointMsg>::SharedPtr join_data_pub;
    rclcpp::Subscription<TargetMsg>::SharedPtr station_info_sub_;
    rclcpp::Subscription<StoneMsg>::SharedPtr stone_info_sub_;
  
  private:
    std::unique_ptr<SerialPort> serial_port_;
    std::unique_ptr<serialport> init_serial_port();

  private:
    // void param_callback();
  }

}