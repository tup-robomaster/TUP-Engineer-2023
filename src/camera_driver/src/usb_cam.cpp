#include "../include/usb_cam.hpp"

namespace camera_driver
{
  usb_cam::usb_cam(usb_cam_params usb_params)
  {
    this->usb_cam_params_ = usb_params;
    // RCLCPP_INFO(this->get_logger(), "node success!");
  }
  usb_cam::~usb_cam()
  {
  }
  void usb_cam::init()
  {
    // cap.open(0);
    // if(cap.isOpened())
    // {
    //   this->is_open = true;
    // }
  }
}