#include "../include/usb_cam_pub.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace camera_driver
{

  UsbCamNode::UsbCamNode(const rclcpp::NodeOptions &option)
      : Node("usb_driver", option), is_filpped(false)
  {
    this->declare_parameter<bool>("using_video", false);
    using_video_ = this->get_parameter("using_video").as_bool();
    this->declare_parameter<std::string>("video_path", "src/camera_driver/video/red_.mp4");
    video_path_ = this->get_parameter("video_path").as_string();
    this->declare_parameter<int>("cam_id", 2);
    cam_id_ = this->get_parameter("cam_id").as_int();
    this->declare_parameter<bool>("save_video", false);
    save_video_ = this->get_parameter("save_video").as_bool();

    if (using_video_)
    {
      cap.open(video_path_);
      if (!cap.isOpened())
      {
        RCLCPP_ERROR(this->get_logger(), "Open video failed!");
      }
    }
    else
    {
      cap.open(cam_id_);

      if (cap.isOpened())
      {
        RCLCPP_INFO(this->get_logger(), "Open camera success!");
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Open camera failed!");
      }
    }

    // cap.set(cv::CAP_PROP_BRIGHTNESS, -50); // 亮度

    // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    // cap.set(cv::CAP_PROP_EXPOSURE, -1); // 曝光
    rclcpp::QoS qos(0);
    qos.keep_last(1);
    // qos.reliable();
    qos.best_effort();
    qos.transient_local();
    qos.durability_volatile();

    rmw_qos_profile_t rmw_qos(rmw_qos_profile_default);
    rmw_qos.depth = 1;

    if (save_video_)
    { // Video save.
      RCLCPP_WARN_ONCE(this->get_logger(), "Saving video...");
      time_t tmpcal_ptr;
      tm *tmp_ptr = nullptr;
      tmpcal_ptr = time(nullptr);
      tmp_ptr = localtime(&tmpcal_ptr);
      char now[64];
      strftime(now, 64, "%Y-%m-%d_%H_%M_%S", tmp_ptr); // 以时间为名字
      std::string now_string(now);
      std::string pkg_path = ament_index_cpp::get_package_share_directory("camera_driver");
      std::string save_path = this->declare_parameter("save_path", "/recorder/video.webm");
      std::string path = pkg_path + save_path + now_string;
      writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
      rosbag2_storage::StorageOptions storage_options({path, "sqlite3"});
      rosbag2_cpp::ConverterOptions converter_options({rmw_get_serialization_format(),
                                                       rmw_get_serialization_format()});
      writer_->open(storage_options, converter_options);
      writer_->create_topic({"usb_image",
                             "sensor_msgs::msg::Image",
                             rmw_get_serialization_format(),
                             ""});
    }

    last_frame = std::chrono::steady_clock::now();
    img_callback_thread_ = std::thread(std::bind(&UsbCamNode::image_callback, this));
    this->camera_pub_node_ = image_transport::create_camera_publisher(this, "usb_image", rmw_qos);
  }

  std::unique_ptr<UsbCam> UsbCamNode::init_usb_cam()
  {
    this->declare_parameter("frame_id", "usb_image");
    this->declare_parameter("image_width", 680);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("fps", 30);

    usb_cam_params_.frame_id = this->get_parameter("frame_id").as_string();
    usb_cam_params_.image_width = this->get_parameter("image_width").as_int();
    usb_cam_params_.image_height = this->get_parameter("image_height").as_int();
    usb_cam_params_.fps = this->get_parameter("fps").as_int();

    return std::make_unique<UsbCam>(usb_cam_params_);
  }

  void UsbCamNode::image_callback()
  {
    while (true)
    {
      cap >> frame;

      // double alpha = 0.5; // 降低曝光的参数
      // double beta = 0;    // 降低曝光的参数
      // frame.convertTo(frame, -1, alpha, beta);

      if (!frame.empty())
      {
        rclcpp::Time now = this->get_clock()->now();
        image_msg_.header.stamp = now;
        image_msg_.encoding = "bgr8";
        camera_info_msg_.header = image_msg_.header;
        image_msg_.width = frame.cols;
        image_msg_.height = frame.rows;
        image_msg_.is_bigendian = false;
        image_msg_.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
        image_msg_.data.assign(frame.datastart, frame.dataend);

        camera_pub_node_.publish(image_msg_, camera_info_msg_);

        if (save_video_)
        { // Video recorder.
          ++frame_cnt_;
          if (frame_cnt_ % 25 == 0)
          {
            sensor_msgs::msg::Image image_msg = image_msg_;
            auto serializer = rclcpp::Serialization<sensor_msgs::msg::Image>();
            auto serialized_msg = rclcpp::SerializedMessage();
            serializer.serialize_message(&image_msg, &serialized_msg);
            auto bag_msg = std::make_shared<rosbag2_storage::SerializedBagMessage>();
            bag_msg->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
                new rcutils_uint8_array_t,
                [this](rcutils_uint8_array_t *msg)
                {
                  if (rcutils_uint8_array_fini(msg) != RCUTILS_RET_OK)
                  {
                    RCLCPP_ERROR(this->get_logger(), "RCUTILS_RET_INVALID_ARGUMENT OR RCUTILS_RET_ERROR");
                  }
                  delete msg;
                });
            *bag_msg->serialized_data = serialized_msg.release_rcl_serialized_message();
            bag_msg->topic_name = "usb_image";
            bag_msg->time_stamp = now.nanoseconds();
            writer_->write(bag_msg);
          }
        }

        // RCLCPP_INFO(this->get_logger(), "msg_ptr ...");
        // cv::namedWindow("raw_image", cv::WINDOW_AUTOSIZE);
        // cv::imshow("raw_image", frame);
        // cv::waitKey(1);
        rclcpp::Time end = this->get_clock()->now();
      }
    }
  }
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<camera_driver::UsbCamNode>());
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::UsbCamNode)
