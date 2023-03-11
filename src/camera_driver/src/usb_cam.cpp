#include "../include/usb_cam.hpp"

namespace camera_driver
{
  usb_cam::usb_cam(usb_cam_params usb_params)
  {
    this->usb_cam_params_ = usb_params;
  }
  usb_cam::~usb_cam()
  {
  }
  void usb_cam::init()
  {
  }
}