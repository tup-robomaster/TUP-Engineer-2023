#include "../include/usb_cam_pub.hpp"

using namespace std::chrono_literals;

namespace camera_driver
{

  usb_cam_node::usb_cam_node(const rclcpp::NodeOptions &option)
      : Node("usb_driver", option), is_filpped(false)
  {

    this->declare_parameter<bool>("using_video", false);
    using_video_ = this->get_parameter("using_video").as_bool();
    this->declare_parameter<std::string>("video_path", "/home/liyuhang/Desktop/TUP-Engineer-2023/src/camera_driver/video/test.mp4");
    video_path_ = this->get_parameter("video_path").as_string();

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
      cap.open(0);
      if (cap.isOpened())
      {
        RCLCPP_INFO(this->get_logger(), "Open camera success!");
      }
    }

    // cap.set(cv::CAP_PROP_BRIGHTNESS, -50); // 亮度

    // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.25);
    // cap.set(cv::CAP_PROP_EXPOSURE, -50); // 曝光
    // std::cout<<cap.get(cv::CAP_PROP_EXPOSURE)<<std::endl;

    last_frame = std::chrono::steady_clock::now();
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("usb_image", 1);
    timer_ = this->create_wall_timer(1ms, std::bind(&usb_cam_node::image_callback, this));
  }

  std::unique_ptr<usb_cam> usb_cam_node::init_usb_cam()
  {
    usb_cam_params usb_cam_params_;

    this->declare_parameter("camera_id", 0);
    this->declare_parameter("frame_id", "usb_image");
    this->declare_parameter("image_width", 680);
    this->declare_parameter("image_height", 480);
    this->declare_parameter("fps", 30);

    usb_cam_params_.camera_id = this->get_parameter("camera_id").as_int();
    usb_cam_params_.frame_id = this->get_parameter("frame_id").as_string();
    usb_cam_params_.image_width = this->get_parameter("image_width").as_int();
    usb_cam_params_.image_height = this->get_parameter("image_height").as_int();
    usb_cam_params_.fps = this->get_parameter("fps").as_int();

    printf("camera_id: %d\n", usb_cam_params_.camera_id);
    return std::make_unique<usb_cam>(usb_cam_params_);
  }

  std::unique_ptr<sensor_msgs::msg::Image> usb_cam_node::convert_frame_to_message(cv::Mat &frame)
  {
    std_msgs::msg::Header header;
    sensor_msgs::msg::Image ros_image;

    ros_image.header = header;
    ros_image.height = frame.rows;
    ros_image.width = frame.cols;
    ros_image.encoding = "bgr8"; // 图像的编码格式

    ros_image.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    ros_image.is_bigendian = false; // 图像数据的大小端存储模式
    ros_image.data.assign(frame.datastart, frame.dataend);

    auto msg_ptr = std::make_unique<sensor_msgs::msg::Image>(ros_image);

    return msg_ptr;
  }

  void usb_cam_node::image_callback()
  {
    cap >> frame;
    auto now_frame = std::chrono::steady_clock::now();
    if (cap.isOpened())
    {
      this->last_frame = now_frame;
      rclcpp::Time timestamp = this->get_clock()->now();
      sensor_msgs::msg::Image::UniquePtr msg = convert_frame_to_message(frame);

      image_publisher_->publish(std::move(msg));
      // RCLCPP_INFO(this->get_logger(), "msg_ptr ...");
      // cv::namedWindow("raw_image", cv::WINDOW_AUTOSIZE);
      // cv::imshow("raw_image", frame);
      // cv::waitKey(1);
    }
  }

}

int main(int argc, char *argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  const rclcpp::NodeOptions options;
  auto usb_cam_node = std::make_shared<camera_driver::usb_cam_node>(options);
  exec.add_node(usb_cam_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(camera_driver::usb_cam_node)
