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
      detector_->is_init_ = true;
    }

    // Qos
    rclcpp::QoS qos(0);
    qos.keep_last(3);
    qos.reliable();
    qos.transient_local();
    qos.durability_volatile();

    rmw_qos_profile_t my_rmw(rmw_qos_profile_sensor_data);
    my_rmw.depth = 1;

    transport_ = this->declare_parameter("subscribe_compressed", false) ? "compressed" : "raw";
    img_sub = std::make_shared<image_transport::Subscriber>(image_transport::create_subscription(this, "usb_image", std::bind(&DetectorNode::image_callback, this, _1), transport_, my_rmw));

    time_start_ = detector_->steady_clock_.now();

    if (debug_.using_imu)
    {
      RCLCPP_INFO(this->get_logger(), "Using imu...");
      // serial_msg.header.frame_id = "serial";
      serial_msg_sub_ = this->create_subscription<SerialMsg>("/serial_msg", qos, std::bind(&DetectorNode::sensorMsgCallback, this, _1));
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

    // Pose(stone-station-to-arm) publish
    publisher_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", qos);
    target_pub_ = this->create_publisher<TargetMsg>("target_pub", qos);

    // Marker(stone station visualization detector) publish
    cam_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("cam_visualization_marker", qos);
    cam_timers_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&DetectorNode::cam_marker_callback, this));
    cam_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    cam_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*cam_tf_buffer_);
    // Marker(stone station visualization acquisition) publish
    arm_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("arm_visualization_marker", qos);
    arm_timers_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&DetectorNode::arm_marker_callback, this));
    arm_tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    arm_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*arm_tf_buffer_);
  }

  DetectorNode::~DetectorNode()
  {
  }

  void DetectorNode::sensorMsgCallback(const SerialMsg &serial_msg)
  {
    msg_mutex_.lock();
    mode = serial_msg.mode;
    msg_mutex_.unlock();
    return;
  }

  /**
   * 相机图像数据的回调函数，用于接收相机发布的图像，进行识别处理，并发布相机与矿站之间的位置关系
   */

  void DetectorNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &img_ptr)
  {
    if (!img_ptr)
    {
      return;
    }

    global_user::TaskData src;
    img_header_ = img_ptr->header;
    rclcpp::Time time = img_ptr->header.stamp;
    rclcpp::Time now = this->get_clock()->now();
    double dura = (now.nanoseconds() - time.nanoseconds()) / 1e6;

    if ((dura) > 20.0)
      return;

    rclcpp::Time stamp = img_ptr->header.stamp;
    src.timestamp = stamp.nanoseconds();
    src.img = cv_bridge::toCvShare(img_ptr, "bgr8")->image;

    std::vector<Stone_Station> station;
    if (detector_->stone_station_detect(src, pose_msg_, is_target))
    {
      RCLCPP_INFO(this->get_logger(), "stone_station detector ...");
      publisher_pose_->publish(pose_msg_);
    }
    if (is_target == false)
    {
      target_info.is_target = is_target;

      target_info.pitch = 0;
      target_info.roll = 0;
      target_info.yaw = 0;
      target_info.x_dis = 0;
      target_info.y_dis = 0;
      target_info.z_dis = 0;

      target_pub_->publish(target_info);
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

    param_mutex_.lock();
    detector_->detector_params_ = this->detector_params_;
    detector_->debug_params_ = this->debug_;
    param_mutex_.unlock();
    return result;
  }

  /**
   * 将stone_station_frame到cam_link姿态信息转换，通过tf_broadcaster_发布转换关系。
   */

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

  /**
   * 从TF树上查询获取arm_link、stone_station_frame两个坐标系之间的变换关系，并将其转换为欧拉角和坐标系的位置信息，
   * 最终将这些信息通过 ROS 发布出去。
   */

  void DetectorNode::tf_callback()
  {
    std::string frame_a = "arm_link";
    std::string frame_b = "stone_station_frame";
    Eigen::Vector3d sum_location;
    Eigen::Vector3d sum_angle;
    Eigen::Vector3d total_sum_distance;
    Eigen::Vector3d total_sum_angle;
    Eigen::Vector3d val_distance;

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

    TargetInfo target = {angle_last_, location_last_};
    Target_Info_ target_;
    history_info.push_back(target);

    // 进行模式判断，如果模式为0则进行常规发布动态识别数据，如果模式为1则进行多帧处理数据之后取定值发布
    if (mode == 0)
    {
      target_info.is_target = is_target;
      target_info.roll = angle_last_[0];
      target_info.pitch = angle_last_[1] - CV_PI / 2;
      target_info.yaw = angle_last_[2] + CV_PI;
      target_info.x_dis = location_last_[0];
      target_info.y_dis = location_last_[1];
      target_info.z_dis = location_last_[2];
      if (mode_  == 1)
      {
        mode_ = 0;
        history_info_.clear();
      }
      if (is_target == true)
      {
        target_pub_->publish(target_info);
      }
    }

    if (mode == 1)
    {
      mode_  = mode;
      if (!history_info.empty())
      {
        if (history_info.size() == 20)
        {
          // history_info
          for (int i = 0; i < history_info.size(); i++)
          {
            sum_location[0] += history_info.at(i).distance[0];
            sum_location[1] += history_info.at(i).distance[1];
            sum_location[2] += history_info.at(i).distance[2];

            sum_angle[0] += history_info.at(i).angle[0];
            sum_angle[1] += history_info.at(i).angle[1];
            sum_angle[2] += history_info.at(i).angle[2];
          }

          total_sum_angle[0] = 0;
          total_sum_angle[1] = 0;
          total_sum_angle[2] = 0;

          total_sum_distance[0] = 0;
          total_sum_distance[1] = 0;
          total_sum_distance[2] = 0;

          total_sum_distance[0] = sum_location[0] / 20;
          total_sum_distance[1] = sum_location[1] / 20;
          total_sum_distance[2] = sum_location[2] / 20;

          total_sum_angle[0] = sum_angle[0] / 20;
          total_sum_angle[1] = sum_angle[1] / 20;
          total_sum_angle[2] = sum_angle[2] / 20;

          float angel_last_roll_1 = history_info.at(18).angle[0];
          float angel_last_roll_2 = history_info.at(19).angle[0];
          float angel_last_pitch_1 = history_info.at(18).angle[1];
          float angel_last_pitch_2 = history_info.at(19).angle[1];
          float angel_last_yaw_1 = history_info.at(18).angle[2];
          float angel_last_yaw_2 = history_info.at(19).angle[2];
          float distance_last_x_1 = history_info.at(18).distance[0];
          float distance_last_x_2 = history_info.at(19).distance[0];
          float distance_last_y_1 = history_info.at(18).distance[1];
          float distance_last_y_2 = history_info.at(19).distance[1];
          float distance_last_z_1 = history_info.at(18).distance[2];
          float distance_last_z_2 = history_info.at(19).distance[2];

          if (abs(val_distance[2] - total_sum_distance[2]) < 0.01 && abs(distance_last_z_2 - total_sum_distance[2]) < 0.005)
          {
            target_ = {total_sum_angle, total_sum_distance};
            if (history_info_.size() <= 99)
            {
              history_info_.push_back(target_);
            }
          }
          else
          {
            val_distance[2] = total_sum_distance[2];

            total_sum_angle[0] = (angel_last_roll_1 + angel_last_roll_2) / 2;
            total_sum_angle[1] = (angel_last_pitch_1 + angel_last_pitch_2) / 2;
            total_sum_angle[2] = (angel_last_yaw_1 + angel_last_yaw_2) / 2;
            total_sum_distance[0] = (distance_last_x_1 + distance_last_x_2) / 2;
            total_sum_distance[1] = (distance_last_y_1 + distance_last_y_2) / 2;
            total_sum_distance[2] = (distance_last_z_1 + distance_last_z_2) / 2;

            target_ = {total_sum_angle, total_sum_distance};
            if (history_info_.size() <= 49)
            {
              history_info_.push_back(target_);
            }
          }

          history_info.clear();
        }
      }

      if (!history_info_.empty() && history_info_.size() == 50)
      {
        target_info.roll = history_info_.back().angle_[0];
        target_info.pitch = history_info_.back().angle_[1] - CV_PI / 2;
        target_info.yaw = history_info_.back().angle_[2] + CV_PI;

        target_info.x_dis = history_info_.back().distance_[0];
        target_info.y_dis = history_info_.back().distance_[1];
        target_info.z_dis = history_info_.back().distance_[2];
        // cout << "history_info_.back().distance_[2] =  " << -history_info_.back().distance_[2] << endl;
        target_info.is_target = is_target;
        is_send = true;
      }
      //进行发布判断是否有目标以及是否满足发送条件
      if (is_target == true && is_send == true)
      {
        target_pub_->publish(target_info);
      }
    }
    // double roll_ = double((angle_last_[0] * 180) / CV_PI);
    // double yaw_ = double((angle_last_[1] * 180) / CV_PI);
    // double pitch_ = double((angle_last_[2] * 180) / CV_PI);

    if (debug_.show_transform)
    {
      RCLCPP_WARN(get_logger(), "-----------ARM_TO_STATION_INFO------------");
      // RCLCPP_INFO(get_logger(), "Roll: %lf 度", roll_);
      // RCLCPP_INFO(get_logger(), "Pitch: %lf 度", pitch_);
      // RCLCPP_INFO(get_logger(), "Yaw: %lf 度", yaw_);
      RCLCPP_INFO(get_logger(), "Roll: %lf", angle_last_[0]);
      RCLCPP_INFO(get_logger(), "Pitch: %lf", angle_last_[1]);
      RCLCPP_INFO(get_logger(), "Yaw: %lf", angle_last_[2]);
      RCLCPP_INFO(get_logger(), "X_dis: %lf", location_last_[0]);
      RCLCPP_INFO(get_logger(), "Y_dis: %lf", location_last_[1]);
      RCLCPP_INFO(get_logger(), "Z_dis: %lf", location_last_[2]);
    }
  }

  void DetectorNode::cam_marker_callback()
  {
    if (!cam_tf_buffer_->canTransform("cam_link", "stone_station_frame", tf2::TimePointZero))
    {
      RCLCPP_WARN(this->get_logger(), "Cannot get transform from cam_link to stone_station_frame");
      return;
    }

    auto transform = arm_tf_buffer_->lookupTransform("cam_link", "stone_station_frame", tf2::TimePoint());
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

    arm_marker_pub_->publish(std::move(cube_maker));
  }

  void DetectorNode::arm_marker_callback()
  {
    if (!arm_tf_buffer_->canTransform("arm_link", "stone_station_frame", tf2::TimePointZero))
    {
      RCLCPP_WARN(this->get_logger(), "Cannot get transform from arm_link to stone_station_frame");
      return;
    }

    auto transform_ = arm_tf_buffer_->lookupTransform("arm_link", "stone_station_frame", tf2::TimePoint());
    geometry_msgs::msg::Quaternion q;
    tf2::convert(transform_.transform.rotation, q);

    auto cube_maker_ = std::make_unique<visualization_msgs::msg::Marker>();
    cube_maker_->header.stamp = this->now();
    cube_maker_->ns = "basic_shapes";
    cube_maker_->id = 1;
    cube_maker_->type = visualization_msgs::msg::Marker::CUBE;
    cube_maker_->action = visualization_msgs::msg::Marker::ADD;
    cube_maker_->header.frame_id = "arm_link";
    cube_maker_->pose.position.x = transform_.transform.translation.x;
    cube_maker_->pose.position.y = transform_.transform.translation.y;
    cube_maker_->pose.position.z = transform_.transform.translation.z;
    cube_maker_->pose.orientation = q;
    cube_maker_->scale.x = 0.288;
    cube_maker_->scale.y = 0.288;
    cube_maker_->scale.z = 0.288;
    cube_maker_->color.r = 6.0f;
    cube_maker_->color.g = 1.0f;
    cube_maker_->color.b = 1.0f;
    cube_maker_->color.a = 0.80;

    arm_marker_pub_->publish(std::move(cube_maker_));
  }
  // 参数初始化
  std::unique_ptr<Detector> DetectorNode::init_detector()
  {

    this->declare_parameter<bool>("color", true);

    this->declare_parameter<double>("stone_station_conf_high_thres", 0.82);

    // TODO:Set by your own path.
    this->declare_parameter("camera_name", "KS2A543"); // 相机型号
    this->declare_parameter("camera_param_path", "src/global_user/config/camera.yaml");
    this->declare_parameter("network_path", "src/engineer_exchange_stone/model/yolox_123.onnx");

    // Debug.
    this->declare_parameter("debug_without_com", true);
    this->declare_parameter("show_aim_cross", true);
    this->declare_parameter("using_imu", false);
    this->declare_parameter("show_img", true);
    this->declare_parameter("show_transform", false);
    this->declare_parameter("detect_red", true);
    this->declare_parameter("show_fps", true);
    this->declare_parameter("print_letency", false);
    this->declare_parameter("print_target_info", false);
    this->declare_parameter("show_target", true);

    // Update param from param server.
    updateParam();

    return std::make_unique<Detector>(path_params_, detector_params_, debug_);
  }
  // 参数更新
  bool DetectorNode::updateParam()
  {
    bool det_red = this->get_parameter("color").as_bool();
    if (det_red)
      detector_params_.color = RED;
    else
      detector_params_.color = BLUE;

    debug_.detect_red = this->get_parameter("detect_red").as_bool();
    debug_.using_imu = this->get_parameter("using_imu").as_bool();
    debug_.debug_without_com = this->get_parameter("debug_without_com").as_bool();
    debug_.show_aim_cross = this->get_parameter("show_aim_cross").as_bool();
    debug_.show_img = this->get_parameter("show_img").as_bool();
    debug_.show_fps = this->get_parameter("show_fps").as_bool();
    debug_.print_letency = this->get_parameter("print_letency").as_bool();
    debug_.print_target_info = this->get_parameter("print_target_info").as_bool();
    debug_.show_target = this->get_parameter("show_target").as_bool();
    debug_.show_transform = this->get_parameter("show_transform").as_bool();

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