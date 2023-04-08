#include "../include/usb_cam.hpp"

namespace camera_driver
{
  UsbCam::UsbCam(usb_cam_params usb_params)
  {
    this->usb_cam_params_ = usb_params;
  }
  UsbCam::~UsbCam()
  {
  }
  void UsbCam::init()
  {
  }
}