#include "serialport.hpp"
#include "../include/serialport_node.hpp"

using std::placeholders::_1;

namespace serialport
{
  serial_node::serial_node(const rclcpp::NodeOptions& options)
  : Node("serial_port", options), device_name_("ttyACM0"), baud_(115200)
  {
    
    RCLCPP_WARN(this->get_logger(), "Serialport node... ");
    try
    {
      serial_port_ = init_serial_port();
    }
    catch(const std::exception& e)
    {
      RCLCPP_ERROR(this->get_logger(), "Error while initializing serial port: %s", e.what());
    }

    station_info_sub_ = this->create_subscription<TargetMsg>("/station_processor/target_info", 10, std::bind(&serial_node::send_station_data, this, _1) );
    // stone_info_sub_ = this->create_subscription<StoneMsg>("/stone_processor/stone_info", 10, std::bind(&seriaport::param_callback, this, _1) );
    
  }

  serial_node::~serial_node()
  {

  }

  void serial_node::receive_data()
  {
    while(1)
    {
      if (!serial_port_->serial_data_.is_initialized)
      {
          std::cout << "Offline..." << std::endl;
          usleep(5000);
          continue;
      }
      while (!serial_port_->get_Mode(bytes_num_))
        ;
           
    }
  }

  void serial_node::send_station_data(TargetMsg::SharedPtr target_info)
  {
    if(!this->deubug_without_port)
    {
      if(serial_port_->mode == 1 || serial_port_->mode == 2)
      {
        RCLCPP_INFO(this->get_logger(), "current mode is %d", serial_port_->mode);
        VisionData transmition_data = {target_info->pitch, target_info->yaw, target_info->roll, target_info->x_dis, target_info->y_dis, target_info->z_dis ,1};
        seria_port_->transformData(transmition_data);
        serial_port_->send();
      }
      else
      {   // Debug without com.
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500, "Sub station msg...");
      }
    }
  }

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<serialport::serial_node>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(serialport::serial_node)