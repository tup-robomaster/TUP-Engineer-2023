// ros
#include <rclcpp/rclcpp.hpp>

#include <yaml-cpp/yaml.h>
#include <fmt/color.h>
#include <fmt/format.h>
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include "global_user.hpp"

using namespace global_user;
using namespace std;
using namespace cv;

namespace coordsolver
{
  struct PnPInfo
  {
    Eigen::Vector3d station_cam;
    Eigen::Vector3d station_world;
    Eigen::Vector3d R_station_world;
    Eigen::Vector3d R_station_cam;
    Eigen::Vector3d euler;
    Eigen::Vector3d rmat;
  };

  class CoordSolver
  {
  public:
    CoordSolver();
    ~CoordSolver();

    bool loadParam(std::string coord_path, std::string param_name);

    PnPInfo pnp(const std::vector<Point2f> &points_pic, int method);

    cv::Point2f reproject(Eigen::Vector3d &xyz);
    Eigen::Vector2d getAngle(Eigen::Vector3d &xyz_cam);
    Eigen::Vector3d angle_process(Eigen::Vector3d &angle);
    Eigen::Vector3d dis_process(Eigen::Vector3d &last_target);
    cv::Point2f getHeading(Eigen::Vector3d &xyz_cam);

  private:
    Mat intrinsic;
    Mat dis_coeff;
    rclcpp::Logger logger_;

    YAML::Node param_node;
  };
}