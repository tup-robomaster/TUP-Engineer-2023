#include "./detector.hpp"

#include <memory>
//ros
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

#include "global_interface/msg/target.hpp"

namespace stone_station_detector
{
  class detector_node : public rclcpp::Node
  {
    typedef std::chrono::_V2::steady_clock::time_point TimePoint;
    // TODO:
    typedef global_interface::msg::Target TargetMsg;
  
  public:
    detector_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~detector_node();
  
  private:
    //图像订阅信息
    std::shared_ptr<image_transport::Subscriber> img_sub;
    
    std::string transport_;
    TimePoint time_start;
    
    //发布矿站信息
    rclcpp::Publisher<TargetMsg>::SharedPtr station_pub;
  private:
    rclcpp::TimerBase::SharedPtr param_timer_;
    void param_callback();
  
  public:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
  
  public:
    detector_params detector_params_;
    debug_params debug_;
    void getParameters();

    std::unique_ptr<detector> detector_;
    std::unique_ptr<detector> init_detector();

  };
}