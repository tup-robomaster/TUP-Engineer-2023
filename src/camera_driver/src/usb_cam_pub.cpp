#include "../include/usb_cam_pub.hpp"

using namespace std::chrono_literals;

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

    last_frame = std::chrono::steady_clock::now();
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("usb_image", qos);
    timer_ = this->create_wall_timer(1ms, std::bind(&UsbCamNode::image_callback, this));

    if (save_video_)
    {
      // int frame_cnt = 0;
      auto src = frame;
      const std::string &storage_location = "src/camera_driver/data/";
      char now[64];
      std::time_t tt;
      struct tm *ttime;
      int width = 640;
      int height = 480;
      tt = time(nullptr);
      ttime = localtime(&tt);
      strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime); // 以时间为名字
      std::string now_string(now);
      std::string path(std::string(storage_location + now_string).append(".avi"));
      auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(width, height)); // Avi format
      std::future<void> write_video;
      // write_video = std::async(std::launch::async, [&](){writer.write(frame);});
      // bool is_first_loop = true;
      // int frame_cnt = 0;
      // frame_cnt++;
      // if (frame_cnt % 3 == 0)
      // {
      //   frame_cnt = 0;
      //   write_video.wait();
      write_video = std::async(std::launch::async, [&]()
                               { writer.write(src); });
      // }

      RCLCPP_INFO(this->get_logger(), "Saving video...");
    }
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
    cap >> frame;

    // double alpha = 0.5; // 降低曝光的参数
    // double beta = 0;    // 降低曝光的参数
    // frame.convertTo(frame, -1, alpha, beta);

    if (!frame.empty())
    {
      sensor_msgs::msg::Image::UniquePtr image_msg = std::make_unique<sensor_msgs::msg::Image>();
      rclcpp::Time timestamp = this->get_clock()->now();

      image_msg->header.frame_id = "usb_image";
      image_msg->header.stamp = timestamp;
      image_msg->encoding = "bgr8";
      image_msg->height = frame.rows;
      image_msg->width = frame.cols;
      image_msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
      image_msg->is_bigendian = false;
      image_msg->data.assign(frame.datastart, frame.dataend);

      image_publisher_->publish(std::move(image_msg));
      // RCLCPP_INFO(this->get_logger(), "msg_ptr ...");
      // cv::namedWindow("raw_image", cv::WINDOW_AUTOSIZE);
      // cv::imshow("raw_image", frame);
      // cv::waitKey(1);
      rclcpp::Time end = this->get_clock()->now();
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
