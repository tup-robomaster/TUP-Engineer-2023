#include "../include/detector.hpp"

using namespace std;

namespace stone_station_detector
{
  detector::detector(const PathParam &path_param, const DetectorParam &detector_params, const DebugParam &debug_params)
      : detector_params_(detector_params),
        path_params_(path_param), debug_params_(debug_params), logger_(rclcpp::get_logger("stone_station_detector"))
  {
    is_init_ = false;
    // input_size = {1920, 1080};

    is_save_data = false;
  }

  detector::~detector()
  {
    if (is_save_data)
    {
      data_save.close();
    }
  }

  bool detector::stone_station_detect(global_user::TaskData &src, global_interface::msg::Target &target_info)
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
      stone_station.conf = object.prob;

      if (object.color == 0)
        stone_station.key = "B";
      if (object.color == 1)
        stone_station.key = "R";

      memcpy(stone_station.apex2d, object.apex, 4 * sizeof(cv::Point2f));

      //   for(int i = 0; i < 4; i++)
      //   {
      //     stone_station.apex2d[i] += Point2f((float)roi_offset.x, (float)roi_offset.y);
      //   }

      Point2f apex_sum;
      for (auto apex : stone_station.apex2d)
        apex_sum += apex;
      stone_station.center2d = apex_sum / 4.f;

      for (int k = 0; k < 4; k++)
        line(src.img, stone_station.apex2d[k % 4], stone_station.apex2d[(k + 1) % 4], {200, 100, 25}, 1);

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
      last_target[0] = stone_stations.station3d_cam[0] + atc_.x_offset; // 横移距离
      last_target[1] = stone_stations.station3d_cam[1] + atc_.y_offset; // 抬升距离
      last_target[2] = stone_stations.station3d_cam[2] + atc_.z_offset; // 前伸距离

      // auto angle = coordsolver_.getAngle(stone_station.euler);
      auto angle = stone_stations.euler;

      target_info.x_dis = last_target[0];
      target_info.y_dis = last_target[1];
      target_info.z_dis = last_target[2];

      target_info.roll = angle[0];
      target_info.yaw = angle[1];
      target_info.pitch = angle[2];

      target_info.is_target = true;

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

      if (debug_params_.print_target_info)
      {
        if (count % 5 == 0)
        {
          RCLCPP_INFO(logger_, "-----------INFO------------");
          RCLCPP_INFO(logger_, "roll: %lf", angle[0]);
          RCLCPP_INFO(logger_, "Yaw: %lf", angle[1]);
          RCLCPP_INFO(logger_, "Pitch: %lf", angle[2]);
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