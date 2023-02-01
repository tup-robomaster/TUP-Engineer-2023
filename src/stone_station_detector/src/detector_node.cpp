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
    this->declare_parameter("camera_name", "KS2A543");
    this->declare_parameter("camera_param_path", "src/global_user/config/camera.yaml");
    this->declare_parameter("network_path", "src/stone_station_detector/model/opt-0527-002.xml");
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
    
    TODO:
    transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

    img_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "usb_image",
    std::bind(&detector_node::image_callback, this, _1), transport_));
    
    //station pub
    TODO:
    station_pub = this->creat_publisher<TargetMsg>("/station_info", rclcpp::SensorDataQos());
    time_start = std::chrono::steady_clock::now();
    param_timer_ = this->creat_wall_timer(1000ms, std::bind(&detector_node::param_callback, this));

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    this->make_transforms(transformation);

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
    // debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
    debug_.show_img = this->get_parameter("show_img").as_bool();
    // debug_.using_roi = this->get_parameter("using_roi").as_bool();
    debug_.show_fps = this->get_parameter("show_fps").as_bool();
    debug_.print_letency = this->get_paramert("print_letency").as_bool();
    debug_.print_target_info = this->get_paramert("print_target_info").as_bool();

  }

  std::unique_ptr<detector> detector_node::init_detector()
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

  void detector_node::tf_transforms(const global_interface::msg::Target::SharedPtr msg,
      const std::string& header_frame_id, const std::string& child_frame_id)const
  {
    geometry_msgs::msg::TransformStamped Tf;

    Tf.header.stamp = this->get_clock()->now;
    Tf.header.frame_id = camera_frame;
    Tf.child_frame_id = stone_station_frame;

    Tf.transform.translation.x = stone_station.center3d_world.x;
    Tf.transform.translation.y = stone_station.center3d_world.y;
    Tf.transform.translation.z = stone_station.center3d_world.z;
    
    tf2::Quaternion q;
    q.setRPY(
      stone_station.euler(0),
      stone_station.euler(1),
      stone_station.euler(2));

    tf_broadcaster_->sendTransform(Tf);

  }

  void detector_node::tf_transforms(const global_interface::msg::Target::SharedPtr msg,
      const std::string& header_frame_id,const std::string& child_frame_id, const rclcpp::Time& time) const
  {
    geometry_msgs::msg::TransformStamped T;
    struct arm_to_camera atc;

    T.header.stamp = this->get_clock()->now;
    T.header.frame_id = camera_frame;
    T.child_frame_id = arm_frame;

    T.transform.translation.x = atc.x;
    T.transform.translation.y = atc.y;
    T.transform.translation.z = atc.z;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    T.transform.rotation.x = 0;
    T.transform.rotation.y = 0;
    T.transform.rotation.z = 0;
    T.transform.rotation.w = 0;

    tf_broadcaster_->sendTransform(T);

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