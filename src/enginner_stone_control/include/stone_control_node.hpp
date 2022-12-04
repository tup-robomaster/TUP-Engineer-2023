#include "stone_control.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

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

namespace stone_control
{
  class stone_control_node : public rclcpp::Node
  {
    typedef std::chrono::_V2::steady_clock::time_point TimePoint;

  public:
    stone_control_node(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~stone_control_node();
  
  private:
    //subscribe image
    std::shared_ptr<image_transport::Subscriber> image_sub;

    std::string transport_;
    TimePoint time_start;
  private:
    //params callback
    rclcpp::TimerBase::SharedPtr param_timer_;
    void param_callback();
  
  public:
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info);
    FindGlod_ findgold;
  
  public:
    // std::unique_ptr<FindGlod_> FindGlod;
    // std::unique_ptr<stone_control> init_stone_control_;
  };
  

}