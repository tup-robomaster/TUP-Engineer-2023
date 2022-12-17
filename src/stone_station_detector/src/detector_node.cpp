#include "../../include/armor_detector/detector_node.hpp"

using std::placeholders::_1;

namespace stone_station_detector
{
  detector_node::detector_node(const rclcpp::NodeOption& options)
  : Node("stone_station_detector", options)
  {
    RCLCPP_WARN(this->get_logger(), "Starting detector node...");

    this->declare_parameters<bool>("color", true);

    //set path

    //debug
    this->declare_parameter("debug_wihout_com", true);
    // this->declare_parameter("using_roi", true);
    this->declare_parameter("show_img", true);
    this->declare_parameter("detect_red", true);
    this->declare_parameter("show_fps", true);
    this->declare_parameter("print_targrt_info", true);

    try
    {
      this->detector_ = init_detector();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    //station pub
    TODO:
    station_pub = this->creat_publish<TargetMsg>("/station_info", rclcpp::SensorDataQos());

    time_start = std::chrono::steady_clock::now();
    TODO:
    transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

    img_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "usb_image",
    std::bind(&detector_node::image_callback, this, _1), transport_));

    param_timer_ = this->creat_wall_timer(1000ms, std::bind(&detector_node::param_callback, this));
  }

  detector_node::~detector_node()
  {
    
  }
}