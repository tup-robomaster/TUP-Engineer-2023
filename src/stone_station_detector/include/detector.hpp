// #include "./armor_tracker.h"
// #include "./inference.h"
#include "./inference.hpp"

//C++
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>

//ros
#include <rclcpp/rclcpp.hpp>

// #include "../../global_user/include/global_user/global_user.hpp"
// #include "../../global_user/include/coordsolver.hpp"
// #include "global_interface/msg/target.hpp"

typedef std::chrono::_V2::steady_clock::time_point TimePoint;

namespace stone_station_detector
{
  enum Color 
  {
    BLUE,
    RED
  };
  struct debug_params
  {
    /* data */
    bool debug_without_com;
    bool using_roi;
    bool show_img;
    bool detect_red;
    bool show_fps;
    bool print_target_info;

    debug_params()
    {
      debug_without_com = true;
      using_roi = false;
      show_img = true;
      detect_red = true;
      show_fps = true;
      print_target_info = true;
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

  class detector
  {
  public:
    detector(const std::string& camera_name, const std::string& camera_param_path, const std::string& network_path,
      const detector_params& detector_params_, const debug_params& debug_params_);
    ~detector();
  private:
    std::string camera_name;
    std::string camera_param_path;
    std::string network_path;
    detector_params detector_params_;
  
  public:
    void run();
  
  public:
    std::vector<StationObject> objects;

    ofstream data_save;
    bool is_save_data;
  
  private:
    bool is_init;

    StationDetector detector_;
  
  private:
    int count:
    TimePoint time_start;
    TimePoint time_infer;
    TimePoint time_crop;

    int timestamp;

    debug_params debug_params_;
  };
}