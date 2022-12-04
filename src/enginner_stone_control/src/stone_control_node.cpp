#include "../include/stone_control_node.hpp"

using std::placeholders::_1;

namespace stone_control
{
  stone_control_node::stone_control_node(const rclcpp::NodeOptions& options)
  : Node("stone_control", options)
  {
    RCLCPP_WARN(this->get_logger(), "Strarting stone_control node ...");

    time_start = std::chrono::steady_clock::now();

    transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
    image_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "/usb_image", std::bind(&stone_control_node::image_callback, this, _1), transport_));

  }

  stone_control_node::~stone_control_node()
  {

  }

  void stone_control_node::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
  {
    // RCLCPP_INFO(this->get_logger(), "image_callback ...");
  
    if(!img_info)
    {
      return;
    }
    
    cv::Mat Src;
    GoldData goldData;
    auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
    img.copyTo(Src);
    cv::resize(Src, Src, cv::Size(640, 480));
    findgold.getGold(Src,goldData);
    cv::namedWindow("Src", cv::WINDOW_AUTOSIZE);
    cv::imshow("Src", Src);
    cv::waitKey(1);
  }
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stone_control::stone_control_node>());
  rclcpp::shutdown();  

  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(stone_control::stone_control_node)