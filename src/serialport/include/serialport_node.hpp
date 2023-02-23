// #ifndef SERIALPORT_NODE_HPP
// #define SERIALPORT_NODE_HPP

// #pragma once 

// //ros
// #include "rclcpp/rclcpp.hpp"
// #include <rclcpp/publisher.hpp>
// #include <rclcpp/subscription.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <std_msgs/msg/float64.hpp>
// #include <message_filters/subscriber.h>
// #include <geometry_msgs/msg/transform_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>

// // #include "./serialport_old.hpp"
// #include "serialport.hpp"

// #include "../../global_user/include/coordsolver.hpp"
// // #include "global_interface/msg/imu.hpp"
// // #include "global_interface/msg/gimbal.hpp"
// #include "../include/data_transform.hpp"
// #include "global_interface/msg/target.hpp"
// #include "global_interface/msg/stone.hpp"

// #include <Eigen/Core>
// #include <Eigen/Dense>

// using namespace coordsolver;
// using namespace global_user;

// namespace seriaport
// {
  
//   class serial_node : public rclcpp::Node
//   {
//     typedef global_interface::msg::Target TargetMsg;
//     typedef global_interface::msg::Stone StoneMsg;
//   public:
//     serial_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
//     ~serial_node();
  
//   public:
//     void receive_data();
//     void send_stone_data(StoneMsg::SharedPtr msg);
//     void send_station_data(TargetMsg::SharedPtr msg);

//   public:
//     rclcpp::Publisher<JointMsg>::SharedPtr join_data_pub;
//     rclcpp::Subscription<TargetMsg>::SharedPtr station_info_sub_;
//     rclcpp::Subscription<StoneMsg>::SharedPtr stone_info_sub_;
  
//   private:
//     std::unique_ptr<SerialPort> serial_port_;
//     std::unique_ptr<serialport> init_serial_port();

//   private:
//     // void param_callback();
//   };

// }

#ifndef SERIALPORT_NODE_HPP_
#define SERIALPORT_NODE_HPP_

#pragma once 

//ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <message_filters/subscriber.h>

#include "../include/serialport.hpp"
#include "../include/data_transform.hpp"

// #include "global_interface/msg/serial.hpp"
#include "global_interface/msg/target.hpp"
#include "global_interface/msg/stone.hpp"
#include "../../global_user/include/coordsolver.hpp"

using namespace global_user;
using namespace coordsolver;
namespace serialport
{
    class SerialPortNode : public rclcpp::Node
    {
        typedef global_interface::msg::Target TargetMsg;
        typedef global_interface::msg::Stone StoneMsg;

    public:
        SerialPortNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
        ~SerialPortNode();

    public:
        void receiveData();
        void sendData();
        void TargetMsgSub(TargetMsg::SharedPtr msg);
        void StoneMsgSub(StoneMsg::SharedPtr msg);
    
    private:
        int baud_;
        std::string id_;
        std::string device_name_;
        std::thread receive_thread_;
        CoordSolver coordsolver_;
        
        mutex mutex_;
        bool debug_without_port_;
        atomic<int> mode_;
        atomic<bool> flag_;
        VisionData vision_data_;
        rclcpp::TimerBase::SharedPtr timer_;
        
    public:
        /**
         * @brief 哨兵和其他车辆的msg不同，此处订阅者和发布者视兵种而定
         * 
         */
        // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

        // 其他兵种
        rclcpp::Subscription<TargetMsg>::SharedPtr target_info_sub_;
        rclcpp::Subscription<StoneMsg>::SharedPtr stone_info_sub_;
        // rclcpp::Publisher<SerialMsg>::SharedPtr serial_msg_pub_;

    private:
        std::unique_ptr<SerialPort> serial_port_;
        std::unique_ptr<SerialPort> initSerialPort();

        std::unique_ptr<DataTransform> data_transform_;
        std::unique_ptr<DataTransform> initDataTransform();
    
    private:
        std::map<std::string, int> params_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter>& params);
    }; 
} //namespace serialport

#endif
