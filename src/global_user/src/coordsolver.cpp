#include "../include/coordsolver.hpp"

namespace coordsolver
{
  CoordSolver::CoordSolver()
      : logger_(rclcpp::get_logger("coordsolver"))
  {
  }

  CoordSolver::~CoordSolver()
  {
  }

  bool CoordSolver::loadParam(std::string coord_path, std::string param_name)
  {
    YAML::Node config = YAML::LoadFile(coord_path);

    Eigen::MatrixXd mat_intrinsic(3, 3);
    Eigen::MatrixXd mat_coeff(1, 5);

    // 初始化内参矩阵
    auto read_vector = config[param_name]["Intrinsic"].as<vector<float>>();
    initMatrix(mat_intrinsic, read_vector);
    eigen2cv(mat_intrinsic, intrinsic); // 将Eigen::Tensor(张量)类型转换为cv::Mat类型

    // 初始化畸变矩阵
    read_vector = config[param_name]["Coeff"].as<vector<float>>();
    initMatrix(mat_coeff, read_vector);
    eigen2cv(mat_coeff, dis_coeff);

    return true;
  }

  PnPInfo CoordSolver::pnp(const std::vector<cv::Point2f> &points_pic, int method)
  {
    std::vector<Point3d> points_world;

    points_world =
        {
            // {-0.1375, 0.1375, 0},
            // {-0.1375, -0.1375, 0},
            // {0.1375, -0.1375, 0},
            // {0.1375, 0.1375, 0}

            {0, 0.1375, 0.1375},
            {0, 0.1375, -0.1375},
            {0, -0.1375, -0.1375},
            {0, -0.1375, 0.1375}

            // {-0.066, 0.027, 0},
            // {-0.066, -0.027, 0},
            // {0.066, -0.027, 0},
            // {0.066, 0.027, 0}
        };

    Mat rvec;
    Mat tvec;
    Mat rmat;
    Eigen::Matrix3d rvec_eigen;
    Eigen::Vector3d tvec_eigen;
    // Eigen::Vector3d R_center_world = {0, -0.7, -0.05};

    solvePnP(points_world, points_pic, intrinsic, dis_coeff, rvec, tvec, false, method);

    PnPInfo result;
    Rodrigues(rvec, rmat); // 由旋转向量得到旋转矩阵
    cv2eigen(rmat, rvec_eigen);
    cv2eigen(tvec, tvec_eigen);

    result.station_cam = tvec_eigen;
    result.rvec_eigen_ = rvec_eigen;
    result.euler = ::global_user::rotationMatrixToEulerAngles(rvec_eigen);
    return result;
  }

  /**
   * @brief 重投影
   *
   * @param xyz 目标三维坐标
   * @return cv::Point2f 图像坐标系上坐标(x,y)
   */

  cv::Point2f CoordSolver::reproject(Eigen::Vector3d &xyz)
  {
    Eigen::Matrix3d mat_intrinsic;
    cv2eigen(intrinsic, mat_intrinsic);
    //(u,v,1)^T = (1/Z) * K * (X,Y,Z)^T
    auto result = (1.f / xyz[2]) * mat_intrinsic * (xyz); // 解算前进行单位转换
    return cv::Point2f(result[0], result[1]);
  }

  /**
   * @brief 发送数据处理
   *
   * @param
   * @return 发送数据数据处理结果
   */

  Eigen::Vector3d CoordSolver::location_last_process(Eigen::Vector3d &location_last_)
  {
    int count = 0;
    Eigen::Vector3d data_location(0, 0, 0);
    loop:
    if (count <= 100)
    {
      data_location[0] += location_last_[0];
      data_location[1] += location_last_[1];
      data_location[2] += location_last_[2];

      count++;
      goto loop;
    }

    data_location[0] = data_location[0] / 100;
    data_location[1] = data_location[1] / 100;
    data_location[2] = data_location[2] / 100;
    return data_location;
  }

  Eigen::Vector3d CoordSolver::angle_last_process(Eigen::Vector3d &angle_last_)
  {
    int count = 0;
    Eigen::Vector3d data_angle(0, 0, 0);
    if (count <= 100)
    {
      data_angle[0] += angle_last_[0];
      data_angle[1] += angle_last_[1];
      data_angle[2] += angle_last_[2];

      count++;
    }
    else
    {

    }

    data_angle[0] = data_angle[0] / 100;
    data_angle[1] = data_angle[1] / 100;
    data_angle[2] = data_angle[2] / 100;

    return data_angle;
  }

} // namespace coordsolver
