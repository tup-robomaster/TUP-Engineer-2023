#include "serialport.hpp"

namespace serialport
{
  serial_driver::serial_driver(const rclcpp::NodeOptions& options)
  : Node("serial_driver", options), device_name_("ttyACM0"), baud_(115200)
  {
    
    RCLCPP_WARN(this->get_logger(), "Serialport node... ");
    
  }
}