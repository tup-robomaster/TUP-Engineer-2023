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
    this->declare_parameter("camera_name", "0000001");
    this->declare_parameter("camera_param_path", "///.yaml");
    this->declare_parameter("network_path", "//////.xml");
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

  void detector_node::param_callback()
  {
    getParameters();
    this->detector_->debugParams(detector_params_, debug_);
  }

  void detector_node::getParameters()
  {
    bool is_red = this->get_parameter("color").as_bool();
    if(is_red)
      detector_params_.color = RED;
    else
      detector_params_.color = BLUE;
    
    debug_.detect_red = this->get_parameter("detect_red").as_bool();
    debug_.debug_without_com = this->get_parameter("debug_without_com").as_bool();
    debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
    debug_.show_img = this->get_parameter("show_img").as_bool();
    // debug_.using_roi = this->get_parameter("using_roi").as_bool();
    debug_.show_fps = this->get_parameter("show_fps").as_bool();
    debug_.print_letency = this->get_paramert("print_letency").as_bool();
    debug_.print_target_info = this->get_paramert("print_target_info").as_bool();

  }

  std::unique_ptr<detector> detector_node::nodeinit_detector()
  {

    getParameters();

    std::string camera_name = this->get_parameter("camera_name").as_string();
    std::string camrera_param_path = this->get_parameter("camera_param_path").as_string();
    std::string network_path = this->get_parameter("network_path").as_string();

    return std::make_unique<detector>(camera_name, camera_param_path, network_path, detector_params_, debug_);
  }

  void detector_node::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
  {
    RCLCPP_INFO(this->get_logger(), "image callback ...");
    global_user::TaskData src;
    std::vector<stone_station> station;

    if(!img_info)
    {
      return;
    }

    auto img  = cv_brige::toCvShare(img_info, "bgr8")->image;
    img.copyTo(src.img);

    TimePoint time_img_sub = std::chrono::steady_clock::now();
    src.timestamp = (int)(std::chrono::duration<double, std::milli>(time_img_sub - tim_start).count());

    if(detector_->stone_station_detect(src))
    {
      RCLCPP_INFO(this->get_logger(), "stone_station detector ..."); 
      // TargetMsg target_info;
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stone_station_detector::detector_node>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stone_station_detector::detector_node)