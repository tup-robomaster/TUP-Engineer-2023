#include "usb_cam.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

namespace camera_driver
{
  class UsbCamNode : public rclcpp::Node
  {

  public:
    UsbCamNode(const rclcpp::NodeOptions &option = rclcpp::NodeOptions());
    ~UsbCamNode(){};

  public:
    bool is_filpped;
    cv::VideoCapture cap;
    cv::Mat frame;
    bool using_video_;
    std::string video_path_;
    int cam_id_;
    bool save_video_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_frame;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;

  public:
    void image_callback();

  public:
    std::unique_ptr<UsbCam> usb_cam_;
    std::unique_ptr<UsbCam> init_usb_cam();
    usb_cam_params usb_cam_params_;
  }; // usb_cam_node

}
