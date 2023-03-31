#ifndef DETECTOR_NODE_HPP_
#define DETECTOR_NODE_HPP_

#include "./detector.hpp"
#include "../../global_user/include/global_user.hpp"

#include <memory>
// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <image_transport/image_transport.hpp>
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <cv_bridge/cv_bridge.h>
// TF2
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "global_interface/msg/target.hpp"
#include "global_interface/msg/serial.hpp"

using namespace global_user;
using namespace coordsolver;

namespace stone_station_detector
{
  class detector_node : public rclcpp::Node
  {
    typedef global_interface::msg::Target TargetMsg;
    typedef global_interface::msg::Serial SerialMsg;

  public:
    detector_node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~detector_node();

    std::shared_ptr<image_transport::Subscriber> img_sub;

    std::string transport_;

    // 发布矿站信息
    rclcpp::Publisher<TargetMsg>::SharedPtr station_pub;
    // 订阅串口信息
    rclcpp::Subscription<SerialMsg>::SharedPtr serial_msg_sub_;

  private:
    rclcpp::TimerBase::SharedPtr param_timer_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter> &params);
    rclcpp::Time time_start_;

  public:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
    void sensorMsgCallback(const SerialMsg &serial_msg);

  private:
    // Params callback.
    bool updateParam();

  public:
    // Mutex param_mutex_;
    SerialMsg serial_msg_;
    DetectorParam detector_params_;
    PathParam path_params_;
    DebugParam debug_;
    TargetMsg target_info;

    std::unique_ptr<detector> detector_;
    std::unique_ptr<detector> init_detector();

    // TF2
  private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_;
  rclcpp::TimerBase::SharedPtr timer_;
  void stone_station_to_cam();

  };
}

#endif