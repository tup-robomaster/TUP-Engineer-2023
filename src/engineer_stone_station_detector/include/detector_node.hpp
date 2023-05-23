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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "global_interface/msg/target.hpp"
#include "global_interface/msg/serial.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
// Marker
#include <visualization_msgs/msg/marker.hpp>

using namespace global_user;
using namespace coordsolver;

namespace stone_station_detector
{
  class DetectorNode : public rclcpp::Node
  {
    typedef global_interface::msg::Target TargetMsg;
    typedef global_interface::msg::Serial SerialMsg;

  public:
    DetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~DetectorNode();

    std::shared_ptr<image_transport::Subscriber> img_sub;
    std_msgs::msg::Header img_header_;

    std::string transport_;

    // 订阅串口信息
    rclcpp::Subscription<SerialMsg>::SharedPtr serial_msg_sub_;
    rclcpp::Publisher<TargetMsg>::SharedPtr target_pub_;

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
    Mutex msg_mutex_;
    Mutex param_mutex_;
    DetectorParam detector_params_;
    PathParam path_params_;
    DebugParam debug_;
    SerialMsg serial_msg_;
    TargetMsg target_info;
    bool is_target;
    bool is_send = false;
    int mode = 0; 
    int mode_ = 0;
    // TransformMsg tf_data;

    std::unique_ptr<Detector> detector_;
    std::unique_ptr<Detector> init_detector();

    coordsolver::CoordSolver data_process_;
    // TF2
  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    geometry_msgs::msg::TransformStamped transform_;
    rclcpp::TimerBase::SharedPtr timer_;
    void stone_station_to_cam();
  
  private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    geometry_msgs::msg::TransformStamped transform;
    rclcpp::TimerBase::SharedPtr _timer_;
    void visualization_point();

    std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
    std::shared_ptr<tf2_ros::TransformListener> tfListener_;
    rclcpp::TimerBase::SharedPtr timer;
    void tf_callback();

    std::shared_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
    rclcpp::TimerBase::SharedPtr timers;
    void tf_watch_visualization_to_base();

    // pose publish
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;
    geometry_msgs::msg::PoseStamped pose_msg_;

    // Marker publish
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cam_marker_pub_;
    rclcpp::TimerBase::SharedPtr cam_timers_;
    std::shared_ptr<tf2_ros::Buffer> cam_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> cam_tf_listener_;
    void cam_marker_callback();

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr arm_marker_pub_;
    rclcpp::TimerBase::SharedPtr arm_timers_;
    std::shared_ptr<tf2_ros::Buffer> arm_tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> arm_tf_listener_;
    void arm_marker_callback();



  public:

    struct TargetInfo
    {
      Eigen::Vector3d angle;
      Eigen::Vector3d distance;
    };
    struct Target_Info_
    {
      Eigen::Vector3d angle_;
      Eigen::Vector3d distance_;
    };

    std::vector<TargetInfo> history_info;
    std::vector<Target_Info_> history_info_;
  };
}

#endif