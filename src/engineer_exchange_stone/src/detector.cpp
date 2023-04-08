#include "../include/detector.hpp"

using namespace std;

namespace stone_station_detector
{
  Detector::Detector(const PathParam &path_param, const DetectorParam &detector_params, const DebugParam &debug_params)
      : detector_params_(detector_params),
        path_params_(path_param), debug_params_(debug_params), logger_(rclcpp::get_logger("stone_station_detector"))
  {
    is_init_ = false;
    is_save_data = false;
  }

  Detector::~Detector()
  {
    if (is_save_data)
    {
      data_save.close();
    }
  }

  bool Detector::stone_station_detect(global_user::TaskData &src, geometry_msgs::msg::PoseStamped &pose_msg_)
  {
    if (!is_init_)
    {
      if (is_save_data)
      {
        data_save.open("src/data/dis_info_1.txt", ios::out | ios::trunc);
        data_save << fixed;
      }
      is_init_ = true;
    }

    time_start = steady_clock_.now();
    auto input = src.img;
    timestamp = src.timestamp;

    time_crop = steady_clock_.now();
    objects.clear();

    if (!station_detector_.detect(input, objects))
    {
      return false;
    }

    time_infer = steady_clock_.now();

    for (auto object : objects)
    {
      if (detector_params_.color == RED)
      {
        if (object.color != 1)
          continue;
      }
      else
      {
        if (object.color != 0)
          continue;
      }
      Stone_Station stone_station;
      stone_station.color = object.color;
      stone_station.conf = object.prob; // 置信度

      // 置信度大于一定值放行
      // if (stone_station.conf >= this->detector_params_.stone_station_conf_high_thres)
      // {
      //   continue;
      // }

      if (object.color == 0)
        stone_station.key = "B";
      if (object.color == 1)
        stone_station.key = "R";

      memcpy(stone_station.apex2d, object.apex, 4 * sizeof(cv::Point2f));

      Point2f apex_sum;
      for (auto apex : stone_station.apex2d)
        apex_sum += apex;
      stone_station.center2d = apex_sum / 4.f;

      circle(src.img, stone_station.center2d, 5, {200, 100, 25}, -1);

      // 生成矿站旋转矩形
      std::vector<Point2f> points_pic(stone_station.apex2d, stone_station.apex2d + 4);
      RotatedRect points_pic_rrect = minAreaRect(points_pic);
      stone_station.rrect = points_pic_rrect;

      // 进行pnp解算,采取迭代法
      // int pnp_method = SOLVEPNP_IPPE;
      int pnp_method = SOLVEPNP_ITERATIVE;

      auto pnp_result = coordsolver_.pnp(points_pic, pnp_method);

      stone_station.station3d_world = pnp_result.station_world;
      stone_station.station3d_cam = pnp_result.station_cam;
      stone_station.euler = pnp_result.euler;
      stone_station.area = object.area;
      auto stone_stations = stone_station;

      // 坐标系转换获得最终yaw，pitch，roll，x，y，z
      last_target[0] = stone_stations.station3d_cam[0];
      last_target[1] = stone_stations.station3d_cam[1];
      last_target[2] = stone_stations.station3d_cam[2];

      auto angle = stone_stations.euler;

      coordsolver_.angle_process(angle);
      coordsolver_.dis_process(last_target);

      // angle[2] = angle[2]-CV_PI;

      tf2::Quaternion qu;
      qu.setRPY(angle[0], angle[1], angle[2]);
      // pose_msg_.header.stamp = this->get_clock()->now();
      pose_msg_.header.frame_id = "arm_to_cam";
      pose_msg_.pose.position.x = last_target[0];
      pose_msg_.pose.position.y = last_target[1];
      pose_msg_.pose.position.z = last_target[2];
      pose_msg_.pose.orientation.x = qu.x();
      pose_msg_.pose.orientation.y = qu.y();
      pose_msg_.pose.orientation.z = qu.z();
      pose_msg_.pose.orientation.w = qu.w();

      if (debug_params_.show_target)
      {
        RCLCPP_DEBUG_ONCE(logger_, "Show target...");

        std::string id_str = to_string(stone_stations.id);

        if (stone_stations.color == 0)
          putText(src.img, "B" + id_str, stone_stations.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {255, 100, 0}, 2);
        if (stone_stations.color == 1)
          putText(src.img, "R" + id_str, stone_stations.apex2d[0], FONT_HERSHEY_SIMPLEX, 1, {0, 0, 255}, 2);

        for (int i = 0; i < 4; i++)
          line(src.img, stone_stations.apex2d[i % 4], stone_stations.apex2d[(i + 1) % 4], {0, 255, 0}, 1);

        auto target_center = coordsolver_.reproject(stone_stations.station3d_cam);
        circle(src.img, target_center, 4, {0, 0, 255}, 2);
      }

      auto time_predict = steady_clock_.now();
      double dr_crop_ns = (time_crop - time_start).nanoseconds();
      double dr_infer_ns = (time_infer - time_crop).nanoseconds();
      double dr_full_ns = (time_predict - time_start).nanoseconds();

      if (debug_params_.print_letency)
      {
        // 降低输出频率，避免影响帧率
        if (count % 5 == 0)
        {
          RCLCPP_INFO(logger_, "-----------TIME------------");
          RCLCPP_INFO(logger_, "Crop:  %lfms", (dr_crop_ns / 1e6));
          RCLCPP_INFO(logger_, "Infer: %lfms", (dr_infer_ns / 1e6));
          RCLCPP_INFO(logger_, "Total: %lfms", (dr_full_ns / 1e6));
        }

        // if (is_save_data_)
        // {
        //   data_save_ << setprecision(3) << (float)(dr_infer_ns / 1e6) << endl;
        // }
      }

      double roll = double((angle[0] * 180) / CV_PI);
      double yaw = double((angle[1] * 180) / CV_PI);
      double pitch = double(((angle[2]) * 180) / CV_PI);

      if (debug_params_.print_target_info)
      {
        if (count % 5 == 0)
        {
          RCLCPP_INFO(logger_, "-----------INFO------------");
          RCLCPP_INFO(logger_, "roll: %lf 度", roll);
          RCLCPP_INFO(logger_, "Yaw: %lf 度", yaw);
          RCLCPP_INFO(logger_, "Pitch: %lf 度", pitch);
          // RCLCPP_INFO(logger_, "roll: %lf", angle[0]);
          // RCLCPP_INFO(logger_, "Yaw: %lf", angle[1]);
          // RCLCPP_INFO(logger_, "Pitch: %lf", angle[2]);
          RCLCPP_INFO(logger_, "X_dis: %lf", last_target[0]);
          RCLCPP_INFO(logger_, "Y_dis: %lf", last_target[1]);
          RCLCPP_INFO(logger_, "Z_dis: %lf", last_target[2]);
          RCLCPP_INFO(logger_, "Dist: %fm", (float)stone_station.station3d_cam.norm());
          count = 0;
        }
      }
      if (debug_params_.show_fps)
      {
        char ch[10];
        sprintf(ch, "%.2f", (1e9 / dr_full_ns));
        std::string fps_str = ch;
        putText(src.img, fps_str, {10, 25}, FONT_HERSHEY_SIMPLEX, 1, {0, 255, 0});
      }

      // // 若预测出错取消本次数据发送
      // if (isnan(angle[0]) || isnan(angle[1]) || isnan(angle[3]) || isnan(last_target[0]) || isnan(last_target[1]) || isnan(last_target[2]))
      // {
      //   LOG(ERROR)<<"NAN Detected! Data Transmit Aborted!";
      //   return false;
      // }
    }

    return true;
  }

}