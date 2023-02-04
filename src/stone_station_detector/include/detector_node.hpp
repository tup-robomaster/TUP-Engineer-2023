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
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace stone_station_detector
{
  class detector_node : public rclcpp::Node
  {
    typedef std::chrono::_V2::steady_clock::time_point TimePoint;
    TODO:
    typedef global_interface::msg::Target_station TargetMsg;
  
  public:
    detector_node(const rclcpp::NodeOption& options = rclcpp::NodeOptions());
    ~detector_node();
  
  private:
    //图像订阅信息
    std::shared_ptr<image_transport::Subscriber> img_sub;
    
    std::string transport_;
    TimePoint time_start;
    
    //发布矿站信息
    StationMsg station_info;
    rclcpp::Publisher<TargetMsg>::SharedPtr station_pub;
  private:
    rclcpp::TimerBase::SharedPtr param_timer_;
    void param_callback();
  
  public:
    void image_callback(const sensor_mags::msg::Image::ConstSharedPtr &img_info);
  
  public:
    detector_params detector_params_;
    debug_params debug_;
    void getParameters();

    std::unique_ptr<detector> detecor_;
    std::unique_ptr<detector> init_detector();

  } 
}