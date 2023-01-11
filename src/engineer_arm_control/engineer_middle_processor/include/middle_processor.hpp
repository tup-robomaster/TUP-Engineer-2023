#include "./inference.hpp"

//C++
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>

//ros
#include <rclcpp/rclcpp.hpp>

#include "../../global_user/include/global_user.hpp"
#include "../../global_user/include/coordsolver.hpp"

typedef std::chrono::_V2::steady_clock::time_point Timepoint;

namespace middle_processor
{

  class processor
  {
    public:
      struct set_arm_position
      {
        int err_picth;
        int err_yaw;
        int err_roll;

        int err_x_dis;
        int err_y_dis;
        int err_z_dis;
      };

      struct current_arm_position
      {
        int current_picth;
        int current_yaw;
        int current_roll;

        int current_x_dis;
        int current_y_dis;
        int current_z_dis;
      };
      
    public:
      processor();
      ~processor();
      void run();
      void get_current_arm_position();
  };
}
