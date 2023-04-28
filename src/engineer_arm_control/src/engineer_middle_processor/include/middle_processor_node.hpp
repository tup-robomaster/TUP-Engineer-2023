#include "middle_processor.hpp"

// ros
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>

namespace middle_processor
{
  class middle_processor_node : public rclcpp::Node
  {
    typedef std::chrono::_V2::steady_clock::time_point Time_point;
    // 机械臂规划移动角度和距离
    typedef global_interface::msg::set_position SetMsg;
    // //下位机反馈机械臂实际移动
    typedef global_interface::msg::true_position ArmMsg;

  public:
    middle_processor_node(const rclcpp::NodeOption &options = rclcpp::NodeOptions());
    ~middle_processor_node();

  private:
    // 订阅信息
    rclcpp::Subscription<ArmMsg> true_position_sub;
    rclcpp::TimerBase::SharedPtr Timer_;
    Time_point Time_start;

    void true_position_callback(const global_interface::msg::set_position::ConstSharedPtr &set_arm_info);

  private:
    // 发布信息
    SetMsg set_arm_info;
    rclcpp::Publisher<SetMsg>::SharedPtr set_position_pub;

    void set_callback();
  }

} // namespace middle_processor
