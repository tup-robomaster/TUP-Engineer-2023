#include "./inference.hpp"

//C++
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <memory>

//ros
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "../../global_user/include/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/target.hpp"

namespace stone_station_detector
{
  enum Color 
  {
    BLUE,
    RED
  };

  struct Stone_Station
  {
    int color;
    int area;
    double conf;
    string key;
    Point2f apex2d[4];
    Rect rect;
    RotatedRect rrect;
    Rect roi;
    Point2f center2d;
    Eigen::Vector3d center3d_cam;
    Eigen::Vector3d center3d_world;
    Eigen::Vector3d euler;
    // Eigen::Vector3d predict
    // global_user::TargetType type;
  };
  struct debug_params
  {
    /* data */
    bool debug_without_com;
    // bool using_roi;
    bool show_img;
    bool detect_red;
    bool show_fps;
    bool print_target_info;
    bool show_aim_cross;

    debug_params()
    {
      debug_without_com = true;
      // using_roi = false;
      show_img = true;
      detect_red = true;
      show_fps = true;
      print_target_info = true;
      show_aim_cross = false;
    }
  };
  
  struct detector_params
  {
    int dw, dh;           //letterbox对原图像resize的padding区域的宽度和高度
    float rescale_ratio;  //缩放比例

    Color color;
    detector_params()
    {
      color = RED;
    }
  };
  //暂时位置（待改）
  struct arm_to_camera
  {
    float x_offset = 0;
    float y_offset = 0;
    float z_offset = 0;
  };
  class detector
  {
  public:
    detector(const std::string& camera_name, const std::string& camera_param_path, const std::string& network_path,
      const detector_params& detector_params_, const debug_params& debug_params_);
    ~detector();
  public:
    std::string camera_name;
    std::string camera_param_path;
    std::string network_path;
    detector_params detector_params_;
  
  public:
    void run();
    bool stone_station_detect(global_user::TaskData &src, global_interface::msg::Target& target_info);
    void debugParams(const detector_params& detector_params, const debug_params& debug_params);
  public:
    std::vector<StationObject> objects;
    Eigen::Vector3d last_target;
    ofstream data_save;
    bool is_save_data;
  
  public:
    bool is_init;
    coordsolver::CoordSolver coordsolver_;
    Station_Detector detector_;
    arm_to_camera atc_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
  
  private:
    int count;
    rclcpp::Time time_start;
    rclcpp::Time time_infer;
    rclcpp::Time time_crop;

    int timestamp;
    Size2i input_size;

    debug_params debug_params_;
  };
}