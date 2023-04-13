#include "../include/detector_node.hpp"

using namespace std::placeholders;

namespace stone_station_detector
{
  DetectorNode::DetectorNode(const rclcpp::NodeOptions &options)
      : Node("stone_station_detector", options)
  {
    RCLCPP_WARN(this->get_logger(), "Starting station_detector node...");

    try
    {
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
                                                                                                 std::bind(&DetectorNode::image_callback, this, _1), transport_));

    time_start_ = detector_->steady_clock_.now();

    // Qos
    rclcpp::QoS qos(0);
    qos.keep_last(1);
    qos.reliable();
    qos.transient_local();
    qos.durability_volatile();

    if (debug_.using_imu)
    {
      RCLCPP_INFO(this->get_logger(), "Using imu...");
      serial_msg_.imu.header.frame_id = "imu_link";
      serial_msg_.mode = this->declare_parameter<int>("vision_mode", 1);
      // imu msg sub.
      serial_msg_sub_ = this->create_subscription<SerialMsg>("/serial_msg", qos,
                                                             std::bind(&DetectorNode::sensorMsgCallback, this, _1));
    }

    bool debug = true;
    this->declare_parameter<bool>("debug", true);
    this->get_parameter("debug", debug);
    if (debug)
    {
      RCLCPP_INFO(this->get_logger(), "debug...");
      // 动态调参回调
      callback_handle_ = this->add_on_set_parameters_callback(std::bind(&DetectorNode::paramsCallback, this, _1));
    }
    // TF2-stone-station-to-cam-transform
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&DetectorNode::stone_station_to_cam, this));

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    timer = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&DetectorNode::tf_callback, this));

    // Pose publish
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", qos);
    target_pub_ = this->create_publisher<TargetMsg>("target_pub", qos);

    // Marker publish
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", qos);
    timers_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&DetectorNode::marker_callback, this));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  DetectorNode::~DetectorNode()
  {
  }

  void DetectorNode::sensorMsgCallback(const SerialMsg &serial_msg)
  {
    // msg_mutex_.lock();
    serial_msg_.imu.header.stamp = this->get_clock()->now();
    if (serial_msg.mode == 1 || serial_msg.mode == 2)
      serial_msg_.mode = serial_msg.mode;
    serial_msg_.imu = serial_msg.imu;
    // msg_mutex_.unlock();
    return;
  }

  void DetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_info)
  {
    global_user::TaskData src;
    std::vector<Stone_Station> station;

    auto img_sub_time = detector_->steady_clock_.now();
    src.timestamp = (img_sub_time - time_start_).nanoseconds();

    if (!img_info)
    {
      return;
    }

    auto img = cv_bridge::toCvShare(img_info, "bgr8")->image;
    img.copyTo(src.img);

    if (detector_->stone_station_detect(src, pose_msg_))
    {
      RCLCPP_INFO(this->get_logger(), "stone_station detector ...");
      publisher_pose_->publish(pose_msg_);
    }

    if (debug_.show_aim_cross)
    {
      line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0, 255, 0}, 1);
      line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0, 255, 0}, 1);
    }

    if (debug_.show_img)
    {
      if (src.img.empty())
      {
        std::cout << "[CAMERA] Get empty image" << std::endl;
      }

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
  rcl_interfaces::msg::SetParametersResult DetectorNode::paramsCallback(const std::vector<rclcpp::Parameter> &params)
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

  void DetectorNode::stone_station_to_cam()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "cam_link";
    t.child_frame_id = "stone_station_frame";

    t.transform.translation.x = pose_msg_.pose.position.x;
    t.transform.translation.y = pose_msg_.pose.position.y;
    t.transform.translation.z = pose_msg_.pose.position.z;

    t.transform.rotation.x = pose_msg_.pose.orientation.x;
    t.transform.rotation.y = pose_msg_.pose.orientation.y;
    t.transform.rotation.z = pose_msg_.pose.orientation.z;
    t.transform.rotation.w = pose_msg_.pose.orientation.w;

    tf_broadcaster_->sendTransform(t);
  }

  void DetectorNode::tf_callback()
  {
    std::string frame_a = "arm_link";
    std::string frame_b = "stone_station_frame";

    // 获取两个坐标系之间的变换关系
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      transformStamped = tfBuffer_->lookupTransform(frame_a, frame_b, tf2::TimePoint());
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_ERROR(get_logger(), ex.what());
      return;
    }

    tf2::Quaternion q(transformStamped.transform.rotation.x,
                      transformStamped.transform.rotation.y,
                      transformStamped.transform.rotation.z,
                      transformStamped.transform.rotation.w);
    tf2Scalar roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    Eigen::Vector3d angle_last_(0, 0, 0);
    angle_last_[0] = roll;
    angle_last_[1] = pitch;
    angle_last_[2] = yaw;

    Eigen::Vector3d location_last_(0, 0, 0);
    location_last_[0] = transformStamped.transform.translation.x;
    location_last_[1] = transformStamped.transform.translation.y;
    location_last_[2] = transformStamped.transform.translation.z;

    // location_last_ = data_process_.location_last_process(location_last_);
    // angle_last_ = data_process_.angle_last_process(angle_last_);

    target_info.roll = angle_last_[0];
    target_info.pitch = angle_last_[1];
    target_info.yaw = angle_last_[2];

    target_info.x_dis = location_last_[0];
    target_info.y_dis = location_last_[1];
    target_info.z_dis = location_last_[2];

    // std::cout << "location_last_ = " << location_last_[2] << std::endl;

    target_info.is_target = true;

    target_pub_->publish(target_info);
  }

  void DetectorNode::marker_callback()
  {
    if (!tf_buffer_->canTransform("cam_link", "stone_station_frame", tf2::TimePointZero))
    {
      RCLCPP_WARN(this->get_logger(), "Cannot get transform from cam_link to stone_station_frame");
      return;
    }

    auto transform = tf_buffer_->lookupTransform("cam_link", "stone_station_frame", tf2::TimePointZero);
    geometry_msgs::msg::Quaternion q;
    tf2::convert(transform.transform.rotation, q);

    auto cube_maker = std::make_unique<visualization_msgs::msg::Marker>();
    cube_maker->header.stamp = this->now();
    cube_maker->ns = "basic_shapes";
    cube_maker->id = 0;
    cube_maker->type = visualization_msgs::msg::Marker::CUBE;
    cube_maker->action = visualization_msgs::msg::Marker::ADD;
    cube_maker->header.frame_id = "cam_link";
    cube_maker->pose.position.x = transform.transform.translation.x;
    cube_maker->pose.position.y = transform.transform.translation.y;
    cube_maker->pose.position.z = transform.transform.translation.z;
    cube_maker->pose.orientation = q;
    cube_maker->scale.x = 0.288;
    cube_maker->scale.y = 0.288;
    cube_maker->scale.z = 0.288;
    cube_maker->color.r = 0.0f;
    cube_maker->color.g = 1.0f;
    cube_maker->color.b = 0.0f;
    cube_maker->color.a = 0.80;

    marker_pub_->publish(std::move(cube_maker));
  }

  std::unique_ptr<Detector> DetectorNode::init_detector()
  {

    this->declare_parameter<bool>("color", true);

    this->declare_parameter<double>("stone_station_conf_high_thres", 0.82);

    // TODO:Set by your own path.
    this->declare_parameter("camera_name", "KS2A543"); // 相机型号
    this->declare_parameter("camera_param_path", "src/global_user/config/camera.yaml");
    this->declare_parameter("network_path", "src/engineer_exchange_stone/model/yolox_1.onnx");
    // this->declare_parameter("save_path", "src/data/old_infer1_2.txt");

    // Debug.
    this->declare_parameter("debug_without_com", true);
    // this->declare_parameter("using_imu", false);
    // this->declare_parameter("using_roi", true);
    this->declare_parameter("show_aim_cross", true);
    this->declare_parameter("show_img", true);
    this->declare_parameter("detect_red", true);
    this->declare_parameter("show_fps", true);
    this->declare_parameter("print_letency", false);
    this->declare_parameter("print_target_info", true);
    this->declare_parameter("show_target", true);
    this->declare_parameter("save_data", false);

    // Update param from param server.
    updateParam();

    return std::make_unique<Detector>(path_params_, detector_params_, debug_);
  }

  bool DetectorNode::updateParam()
  {
    bool det_red = this->get_parameter("color").as_bool();
    if (det_red)
      detector_params_.color = RED;
    else
      detector_params_.color = BLUE;

    detector_params_.stone_station_conf_high_thres = this->get_parameter("stone_station_conf_high_thres").as_double();

    debug_.detect_red = this->get_parameter("detect_red").as_bool();
    debug_.debug_without_com = this->get_parameter("debug_without_com").as_bool();
    debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
    debug_.show_img = this->get_parameter("show_img").as_bool();
    // debug_.using_imu = this->get_parameter("using_imu").as_bool();
    // debug_.using_roi = this->get_parameter("using_roi").as_bool();
    debug_.show_fps = this->get_parameter("show_fps").as_bool();
    debug_.print_letency = this->get_parameter("print_letency").as_bool();
    debug_.print_target_info = this->get_parameter("print_target_info").as_bool();
    debug_.show_target = this->get_parameter("show_target").as_bool();
    debug_.save_data = this->get_parameter("save_data").as_bool();

    path_params_.camera_name = this->get_parameter("camera_name").as_string();
    path_params_.camera_param_path = this->get_parameter("camera_param_path").as_string();
    path_params_.network_path = this->get_parameter("network_path").as_string();

    return true;
  }

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stone_station_detector::DetectorNode>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(stone_station_detector::DetectorNode)