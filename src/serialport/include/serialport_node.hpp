#ifndef SERIALPORT_NODE_HPP_
#define SERIALPORT_NODE_HPP_

#pragma once

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <message_filters/subscriber.h>

#include "./serialport.hpp"
#include "./data_transform.hpp"

#include "global_interface/msg/serial.hpp"
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
        typedef global_interface::msg::Serial SerialMsg;

    public:
        SerialPortNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~SerialPortNode();

    public:
        void receiveData();
        void sendData();
        void TargetMsgSub(TargetMsg::SharedPtr msg);
        void StoneMsgSub(StoneMsg::SharedPtr msg);
        void serialWatcher();

    private:
        int baud_;
        std::string id_;
        std::string device_name_;
        std::thread receive_thread_;
        CoordSolver coordsolver_;
        bool print_serial_info_;

        mutex mutex_;
        bool using_port_;
        bool tracking_target_;
        atomic<int> mode_;
        atomic<bool> flag_;
        // VisionData vision_data_;
        rclcpp::TimerBase::SharedPtr timer_;

    public:
        /**
         * @brief 工程msg
         *
         */
        rclcpp::Subscription<TargetMsg>::SharedPtr target_info_sub_;
        rclcpp::Subscription<StoneMsg>::SharedPtr stone_info_sub_;

        rclcpp::Publisher<SerialMsg>::SharedPtr serial_msg_pub_;

    private:
        std::unique_ptr<SerialPort> serial_port_;
        std::unique_ptr<SerialPort> initSerialPort();

        std::unique_ptr<DataTransform> data_transform_;
        std::unique_ptr<DataTransform> initDataTransform();

    private:
        std::map<std::string, int> params_map_;
        OnSetParametersCallbackHandle::SharedPtr callback_handle_;
        bool setParam(rclcpp::Parameter param);
        rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter> &params);
    };
} // namespace serialport

#endif
