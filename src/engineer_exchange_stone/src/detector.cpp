#include "../include/detector.hpp"

using namespace std;

namespace stone_station_detector
{
  detector::detector(const std::string& camera_name_, const std::string& camera_param_path_, const std::string& network_path_,
  const detector_params& _detector_params_, const debug_params& _debug_param_)
  : detector_params_(_detector_params_), debug_params_(_debug_param_)
  {
    //参数设置
    this->camera_name = camera_name_;
    this->camera_param_path = camera_param_path_;
    this->network_path = network_path_;

    is_init = false;
    input_size = {720,720};

    is_save_data = false;
  }

  detector::~detector()
  {
    if(is_save_data)
    {
      data_save.close();
    }
  }
  void detector::debugParams(const detector_params& detector_params, const debug_params& debug_params)
  {
    //detector params
    this->detector_params_.dw = detector_params.dw;
    this->detector_params_.dh = detector_params.dh;
    this->detector_params_.rescale_ratio = detector_params.rescale_ratio;

    //debug
    this->debug_params_.debug_without_com = debug_params.debug_without_com;
    // this->debug_params_.using_roi = debug_params.using_roi;
    // this->debug_params_.show_aim_cross = debug_params.show_aim_cross;
    this->debug_params_.show_img = debug_params.show_img;
    this->debug_params_.detect_red = debug_params.detect_red;
    // this->debug_params_.print_letency = debug_params.print_letency;
    this->debug_params_.print_target_info = debug_params.print_target_info;
  }

  bool detector::stone_station_detect(global_user::TaskData &src, global_interface::msg::Target& target_info)
  {
    if(!is_init)
    {
      detector_.initModel(network_path);
      coordsolver_.loadParam(camera_param_path, camera_name);

      if(is_save_data)
      {
        data_save.open("src/data/dis_info_1.txt", ios::out | ios::trunc);
        data_save << fixed;
      }
      is_init = true;
    }

    time_start = steady_clock_.now();
    auto input = src.img;
    timestamp = src.timestamp;

    time_crop = steady_clock_.now();
    
    objects.clear();
    
    if(!detector_.detect(input, objects))
    {
      if(debug_params_.show_aim_cross)
      {
        line(src.img, Point2f(src.img.size().width / 2, 0), Point2f(src.img.size().width / 2, src.img.size().height), {0, 255, 0}, 1);
        line(src.img, Point2f(0, src.img.size().height / 2), Point2f(src.img.size().width, src.img.size().height / 2), {0,255, 0}, 1);
      }
      if(debug_params_.show_img)
      {
        namedWindow("dst", 0);
        imshow("dst", src.img);
        waitKey(1);
      }
      return false;
    }

    time_infer = steady_clock_.now();

    for(auto object : objects)
    {
      if(detector_params_.color == RED)
      {
        if(object.color != 1)
          continue;
      }
      else
      {
        if(object.color != 0)
          continue;      
      }

      Stone_Station stone_station; 
      stone_station.color = object.color;
      stone_station.conf = object.prob;

      if(object.color == 0)
        stone_station.key = "B";
      if(object.color ==1)
        stone_station.key = "R";
      
      memcpy(stone_station.apex2d, object.apex, 4 * sizeof(cv::Point2f));

    //   for(int i = 0; i < 4; i++)
    //   {
    //     stone_station.apex2d[i] += Point2f((float)roi_offset.x, (float)roi_offset.y);
    //   }

      Point2f apex_sum;
      for(auto apex : stone_station.apex2d)
        apex_sum +=apex;
      stone_station.center2d = apex_sum / 4.f;

      //生成矿站旋转矩形
      std::vector<Point2f> points_pic(stone_station.apex2d, stone_station.apex2d + 4);
      RotatedRect points_pic_rrect = minAreaRect(points_pic);
      stone_station.rrect = points_pic_rrect;
      auto bbox = points_pic_rrect.boundingRect();

      //进行pnp解算,采取迭代法
      int pnp_method = SOLVEPNP_ITERATIVE;

      auto pnp_result = coordsolver_.pnp(points_pic, pnp_method);

      stone_station.center3d_world = pnp_result.station_world;
      stone_station.center3d_cam = pnp_result.station_cam;
      stone_station.euler = pnp_result.euler;
      stone_station.area = object.area;
      // station_.push_back(stone_station);

      //坐标系转换获得最终yaw，pitch，roll，x，y，z
      last_target[0] = stone_station.center3d_cam[0] + atc_.x_offset;
      last_target[1] = stone_station.center3d_cam[1] + atc_.y_offset;
      last_target[2] = stone_station.center3d_cam[2] + atc_.z_offset;
      
      auto angle = stone_station.euler;

      target_info.x_dis = last_target[0];
      target_info.y_dis = last_target[1];
      target_info.z_dis = last_target[2];

      target_info.roll = angle[0];
      target_info.yaw = angle[1];
      target_info.pitch = angle[2];
      
      // if(debug_params_.print_letency)
      // {
      //   //降低输出频率，避免影响帧率
      //   if (count % 5 == 0)
      //   {
      //     fmt::print(fmt::fg(fmt::color::gray), "-----------TIME------------\n");
      //     fmt::print(fmt::fg(fmt::color::blue_violet), "Crop: {} ms\n"   ,dr_crop_ms);
      //     fmt::print(fmt::fg(fmt::color::golden_rod), "Infer: {} ms\n",dr_infer_ms);
      //     // fmt::print(fmt::fg(fmt::color::green_yellow), "Predict: {} ms\n",dr_predict_ms);
      //     fmt::print(fmt::fg(fmt::color::orange_red), "Total: {} ms\n",dr_full_ms);
      //   }
      // }
      if(debug_params_.print_target_info)
      {
        // if (count % 5 == 0)
        // {
          fmt::print(fmt::fg(fmt::color::gray), "-----------INFO------------\n");
          fmt::print(fmt::fg(fmt::color::green_yellow), "roll: {} \n",angle[0]);
          fmt::print(fmt::fg(fmt::color::blue_violet), "Yaw: {} \n",angle[1]);
          fmt::print(fmt::fg(fmt::color::golden_rod), "Pitch: {} \n",angle[2]);
          fmt::print(fmt::fg(fmt::color::golden_rod), "x_dis: {} \n",last_target[0]);
          fmt::print(fmt::fg(fmt::color::golden_rod), "y_dis: {} \n",last_target[1]);
          fmt::print(fmt::fg(fmt::color::golden_rod), "z_dis: {} \n",last_target[2]);

          // if(is_save_data)
          // {
          //     data_save << setprecision(3) << (float)target.center3d_cam.norm() << endl;
          
          // count = 0;
          // }
          // else
          // {
          //   count++;
          // }
        }
        //若预测出错取消本次数据发送
        if (isnan(angle[0]) || isnan(angle[1]) || isnan(angle[3]) || isnan(last_target[0]) || isnan(last_target[1]) || isnan(last_target[2]))
        {
            // LOG(ERROR)<<"NAN Detected! Data Transmit Aborted!";
            return false;
        }

        if(debug_params_.show_img)
        {
            namedWindow("dst",0);
            imshow("dst",src.img);
            waitKey(1);
        }

      }

    return true;
  }
}