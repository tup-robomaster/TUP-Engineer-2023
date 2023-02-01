#include "../include/coordsolver.hpp"

namespace coordsolver
{
  CoordSolver::CoordSolver()
  {
  }

  CoordSolver::~CoordSolver()
  {
  }

  bool CoordSolver::loadParam(string coord_path, string param_name)
  {
    YAML::Node config = YAML::LoadFile(coord_path);

    Eigen::MatrixXd mat_intrinsic(3, 3);
    Eigen::MatrixXd mat_coeff(1 ,5);

    //初始化内参矩阵
    auto read_vector = config[param_name]["Intrinsic"].as<vector<float>>();
    initMatrix(mat_intrinsic, read_vector);
    eigen2cv(mat_intrinsic, intrinsic);   //将Eigen::Tensor(张量)类型转换为cv::Mat类型

    //初始化畸变矩阵
    read_vector = config[param_name]["Coeff"].as<vector<float>>();
    initMatrix(mat_coeff, read_vector);
    eigen2cv(mat_coeff, dis_coeff);

    return true;
  }

  PnPInfo coordsolver::pnp(const std::vector<cv::Point2f> &points_pic, int method = cv::SOLVEPNP_IPPE)
  {
    std::vector<Point3d> points_world;

    points_world = 
    {
      {-137.500, 137.500, 0},
      {-137.500, -137.500, 0},
      {137.500, -137.500, 0},
      {137.500, 137.500, 0}
    };

    Mat rvec;
    Mat tvec;
    Eigen::Matrix3d rvec_eigen;
    
    solvePnP(points_world, points_pic, intrinsic, dis_coeff, rvec, tvec, false, method);

    PnPInfo result;
    // Rodrigues(rvec, rmat); //由旋转向量得到旋转矩阵
    cv2eigen(rvec, rvec_eigen);
    cv2eigen(tvec, tvec_eigen);

    result.station_cam = tvec_eigen;
    result.euler = ::global_user::rotationMatrixToEulerAngles(rvec_eigen);

    return result;
  }


} // namespace coordsolver