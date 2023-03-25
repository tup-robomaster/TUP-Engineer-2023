#include "../include/detector_node.hpp"

using namespace std::placeholders;

namespace stone_station_detector
{
  detector_node::detector_node(const rclcpp::NodeOptions &options)
      : Node("stone_station_detector", options)
  {
    RCLCPP_WARN(this->get_logger(), "Starting station_detector node...");

    try
    { // detector类初始化
      this->detector_ = init_detector();
    }
    catch (const std::exception &e)
    {
      RCLCPP_FATAL(this->get_logger(), "Fatal while initializing detector class: %s", e.what());
    }

    if (!detector_->is_init_)
    {
      RCLCPP_INFO(this->get_logger(), "Initializing network model...");
      detector_->station_detector_.initModel(path_params_.network_path);
      detector_->coordsolver_.loadParam(path_params_.camera_param_path, path_params_.camera_name);
      if (detector_->is_save_data)
      {
        detector_->data_save.open(path_params_.save_path, ios::out | ios::trunc);
        detector_->data_save << fixed;
      }
      detector_->is_init_ = true;
    }

    transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";

    img_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "usb_image",
                                                                                                 std::bind(&detector_node::image_callback, this, _1), transport_));
    // // CameraType camera_type;
    this->declare_parameter<int>("camera_type", usb);
    int camera_type = this->get_parameter("camera_type").as_int();

    time_start_ = detector_->steady_clock_.now();

    // Qos
    rclcpp::QoS qos(0);
    qos.keep_last(5);
    qos.best_effort();
    qos.reliable();
    qos.durability();
    // qos.transient_local();
    qos.durability_volatile();
    // station pub
    station_pub = this->create_publisher<TargetMsg>("/station_info", qos);
    // time_start = std::chrono::steady_clock::now();
    // param_timer_ = this->create_wall_timer(100ms, std::bind(&detector_node::param_callback, this));

    if (debug_.using_imu)
    {
      RCLCPP_INFO(this->get_logger(), "Using imu...");
      serial_msg_.imu.header.frame_id = "imu_link";
      serial_msg_.mode = this->declare_parameter<int>("vision_mode", 1);
      // imu msg sub.
      serial_msg_sub_ = this->create_subscription<SerialMsg>("/serial_msg", qos,
                                                             std::bind(&detector_node::sensorMsgCallback, this, _1));
    }

    bool debug = false;
    this->declare_parameter<bool>("debug", true);
    this->get_parameter("debug", debug);
    if (debug)
    {
      RCLCPP_INFO(this->get_logger(), "debug...");
      // 动态调参回调
      callback_handle_ = this->add_on_set_parameters_callback(std::bind(&detector_node::paramsCallback, this, _1));
    }
  }

  detector_node::~detector_node()
  {
  }

  void detector_node::sensorMsgCallback(const SerialMsg &serial_msg)
  {
    // msg_mutex_.lock();
    serial_msg_.imu.header.stamp = this->get_clock()->now();
    if (serial_msg.mode == 1 || serial_msg.mode == 2)
      serial_msg_.mode = serial_msg.mode;
    serial_msg_.imu = serial_msg.imu;
    // msg_mutex_.unlock();
    return;
  }

  void detector_node::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
  {
    // RCLCPP_INFO(this->get_logger(), "image callback ...");
    global_user::TaskData src;
    std::vector<Stone_Station> station;
    TargetMsg target_info;

    auto img_sub_time = detector_->steady_clock_.now();
    src.timestamp = (img_sub_time - time_start_).nanoseconds();

    if (!img_info)
    {
      return;
    }

    auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
    img.copyTo(src.img);

    if (detector_->stone_station_detect(src, target_info))
    {
      RCLCPP_INFO(this->get_logger(), "stone_station detector ...");

      // target_info.timestamp = src.timestamp;

      station_pub->publish(target_info);
    }

    if (debug_.show_aim_cross)
    {
      line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0, 255, 0}, 1);
      line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0, 255, 0}, 1);
    }

    if (debug_.show_img)
    {
      if(src.img.empty())
      {
        std::cout<< "[CAMERA] Get empty image"<<std::endl;
      }
      // std::cout<<src.img.size()<<std::endl;

      cv::namedWindow("dst", cv::WINDOW_AUTOSIZE);
      cv::imshow("dst", src.img);
      cv::waitKey(1);
    }

  }

  /**
   * @brief 参数回调函数
   *
   * @param params 参数服务器参数（发生改变的参数）
   * @return rcl_interfaces::msg::SetParametersResult
   */
  rcl_interfaces::msg::SetParametersResult detector_node::paramsCallback(const std::vector<rclcpp::Parameter> &params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = "debug";
    result.successful = updateParam();

    // param_mutex_.lock();
    detector_->detector_params_ = this->detector_params_;
    detector_->debug_params_ = this->debug_;
    // param_mutex_.unlock();
    return result;
  }

  std::unique_ptr<detector> detector_node::init_detector()
  {

    this->declare_parameter<bool>("color", true);

    // TODO:Set by your own path.
    this->declare_parameter("camera_name", "KS2A543"); // 相机型号
    this->declare_parameter("camera_param_path", "src/global_user/config/camera.yaml");
    this->declare_parameter("network_path", "src/engineer_exchange_stone/model/yolox_1.onnx");
    this->declare_parameter("save_path", "src/data/old_infer1_2.txt");

    // Debug.
    this->declare_parameter("debug_without_com", true);
    this->declare_parameter("using_imu", false);
    // this->declare_parameter("using_roi", true);
    this->declare_parameter("show_aim_cross", true);
    this->declare_parameter("show_img", true);
    this->declare_parameter("detect_red", true);
    this->declare_parameter("show_fps", true);
    this->declare_parameter("print_letency", false);
    this->declare_parameter("print_target_info", true);
    this->declare_parameter("show_target", true);
    this->declare_parameter("save_data", false);
    this->declare_parameter("save_dataset", false);

    // Update param from param server.
    updateParam();

    return std::make_unique<detector>(path_params_, detector_params_, debug_);
  }

  bool detector_node::updateParam()
  {
    bool det_red = this->get_parameter("color").as_bool();
    if (det_red)
      detector_params_.color = RED;
    else
      detector_params_.color = BLUE;

    debug_.detect_red = this->get_parameter("detect_red").as_bool();
    debug_.debug_without_com = this->get_parameter("debug_without_com").as_bool();
    debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
    debug_.show_img = this->get_parameter("show_img").as_bool();
    debug_.using_imu = this->get_parameter("using_imu").as_bool();
    // debug_.using_roi = this->get_parameter("using_roi").as_bool();
    debug_.show_fps = this->get_parameter("show_fps").as_bool();
    debug_.print_letency = this->get_parameter("print_letency").as_bool();
    debug_.print_target_info = this->get_parameter("print_target_info").as_bool();
    debug_.show_target = this->get_parameter("show_target").as_bool();
    debug_.save_data = this->get_parameter("save_data").as_bool();
    debug_.save_dataset = this->get_parameter("save_dataset").as_bool();

    path_params_.camera_name = this->get_parameter("camera_name").as_string();
    path_params_.camera_param_path = this->get_parameter("camera_param_path").as_string();
    path_params_.network_path = this->get_parameter("network_path").as_string();
    path_params_.save_path = this->get_parameter("save_path").as_string();

    return true;
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stone_station_detector::detector_node>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stone_station_detector::detector_node)