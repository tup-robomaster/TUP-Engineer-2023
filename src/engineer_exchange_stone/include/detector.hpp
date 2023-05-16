#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

#include "./inference.hpp"

// C++
#include <iostream>
#include "rclcpp/clock.hpp"
#include <algorithm>
#include <vector>
#include <string>
#include <memory>

// ros
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "../../global_user/include/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"
#include "global_interface/msg/target.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace stone_station_detector
{

  // struct TargetInfo
  // {
  //   Eigen::Vector3d location_last_(0, 0, 0);
  //   Eigen::Vector3d angle_last_(0, 0, 0);
  // };
  enum Color
  {
    BLUE,
    RED
  };

  struct DetectorParam
  {
    double stone_station_conf_high_thres; // 矿站置信度

    Color color;
    DetectorParam()
    {
      color = RED;
    }
  };

  struct PathParam
  {
    std::string camera_name;
    std::string camera_param_path;
    std::string network_path;
    std::string save_path;
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
    int id;
    // Rect roi;
    Point2f center2d;
    Eigen::Vector3d station3d_cam;
    Eigen::Vector3d station3d_world;
    Eigen::Vector3d euler;
    Eigen::Matrix3d rvec_eigen_;
  };

  struct DebugParam
  {
    bool debug_without_com;
    bool using_imu;
    bool using_roi;
    bool show_aim_cross;
    bool show_img;
    bool detect_red;
    bool show_target;
    bool show_fps;
    bool print_letency;
    bool print_target_info;
    bool show_transform;

    DebugParam()
    {
      debug_without_com = true;
      using_imu = false;
      using_roi = false;
      show_aim_cross = true;
      show_img = true;
      detect_red = true;
      show_target = true;
      show_fps = true;
      print_letency = false;
      print_target_info = true;
      show_transform = false;
    }
  };

  class Detector
  {
  public:
    Detector(const PathParam &path_param, const DetectorParam &detector_params, const DebugParam &debug_params);
    ~Detector();

  public:
    bool stone_station_detect(global_user::TaskData &src, geometry_msgs::msg::PoseStamped &pose_msg_, bool &is_target);
    void showTarget(TaskData &src);

  public:
    std::vector<StationObject> objects;
    std::vector<Stone_Station> stone_stations;
    Eigen::Vector3d last_target;
    ofstream data_save;
    bool is_init_;

  public:
    coordsolver::CoordSolver coordsolver_;
    StationDetector station_detector_;
    rclcpp::Logger logger_;
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  private:
    int count;
    rclcpp::Time time_start;
    rclcpp::Time time_infer;
    rclcpp::Time time_crop;

    int timestamp;
    Size2i input_size;

  public:
    DebugParam debug_params_;
    DetectorParam detector_params_;
    PathParam path_params_;

  public:
    // struct TargetInfo
    // {
    //   Eigen::Vector3d location_last_(0, 0, 0);
    //   Eigen::Vector3d angle_last_(0, 0, 0);
    // };
    // const int history_deque_len = 12;                                     //队列长度
    // const int min_fitting_len = 10;                                         //最短队列长度
    // Eigen::Vector3d sum_location = {0, 0, 0};
    // Eigen::Vector3d sum_angle = {0, 0, 0};
    // Eigen::Vector3d total_sum_distance = {0, 0, 0};
    // Eigen::Vector3d total_sum_angle = {0, 0, 0};
  };
}

#endif