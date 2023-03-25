#ifndef DETECTOR_HPP_
#define DETECTOR_HPP_

#include "./inference.hpp"

// C++
#include <iostream>
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

namespace stone_station_detector
{
  enum Color
  {
    BLUE,
    RED
  };

  struct DetectorParam
  {
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
    bool save_data;
    bool save_dataset;

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
      save_data = false;
    }
  };

  // 暂时位置（待改）
  struct arm_to_camera
  {
    float x_offset = 0;
    float y_offset = 0;
    float z_offset = 0;
  };
  class detector
  {
  public:
    detector(const PathParam& path_param, const DetectorParam& detector_params, const DebugParam& debug_params);
    ~detector();


  public:
    bool stone_station_detect(global_user::TaskData &src, global_interface::msg::Target &target_info);
    void showTarget(TaskData& src);
  public:
    std::vector<StationObject> objects;
    std::vector<Stone_Station> stone_stations;
    Eigen::Vector3d last_target;
    ofstream data_save;
    bool is_save_data;
    bool is_init_;

  public:
    coordsolver::CoordSolver coordsolver_;
    Station_Detector station_detector_;
    arm_to_camera atc_;
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

  };
}

#endif