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
  struct usb_cam_params
  {
    std::string frame_id = "usb_cam";
    int camera_id = 0;
    int image_width = 480;
    int image_height = 480;
    int fps = 30;
  };

  class usb_cam
  {
  public:
    usb_cam_params usb_cam_params_;

  public:
    cv::VideoCapture cap;
    bool is_open;
    cv::Mat src;

  public:
    usb_cam(usb_cam_params usb_params);
    ~usb_cam();

    void init();
  };

}
