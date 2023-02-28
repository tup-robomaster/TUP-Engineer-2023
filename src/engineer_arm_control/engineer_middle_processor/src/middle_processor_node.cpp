#include "../include/middle_processor_node.hpp"

using std::placeholders::_1;

namespace middle_processor
{
  middle_processor_node::middle_processor_node(const rclcpp::NodeOption &options)
      : Node("middle_processor", options)
  {
    RCLCPP_WARN(this->get_logger(), "Starting middle_processor node...");

    // 订阅机械臂实际移动信息
    true_position_sub = this->subscription<ArmMsg>("true_arm_info", std::bind(&middle_processor_node::true_position_callback, this, _1));
    // 发布机械臂规划移动信息
    set_position_pub = this->creat_publisher<SetMsg>("/set_arm_info", rclcpp::SensorDataQos());
    Time_start = std::chrono::steady_clock::now();
    Timer_ = this->creat_wall_timer(1000ms, std::bind(&middle_processor_node::set_callback, this));
  }

  middle_processor_node::~middle_processor_node
  {
  }

  void middle_processor_node::true_position_callback(const global_interface::msg::set_position::ConstSharedPtr &set_arm_info)
  {
  }

  void middle_processor_node::set_callback()
  {
    RCLCPP_INFO(this->get_logger(), "arm_setting callback ...");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<middle_processor::middle_processor_node>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(middle_processor::middle_processor_node)