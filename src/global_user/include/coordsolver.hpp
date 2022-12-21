#include <yaml-cpp/yaml.h>
#include <fmt/color.h>
#include <fmt/format.h>
#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "global_user.hpp"

using namespace std;
using namespace cv;

namespace coordsolver
{
  struct PnPInfo
  {
    Eigen::Vector3d station_cam;
    Eigen::Vector3d station_world;
    Eigen::Vector3d euler;
    Eigen::Vector3d rmat;
  };

  class CoordSolver
  {
  public:
    CoordSolver();
    ~CoordSolver();

    bool loadParam(string coord_path, string param_name);

    PnPInfo pnp(const std::vector<Point2f> &points_pic, int method);

    cv::Point2f reproject(Eigen::Vector3d &xyz);
    cv::Point2f getHeading(Eigen::Vector3d &xyz_cam);

  private:
    Mat intrinsic;
    Mat dis_coeff;

    YAML::Node param_node;
  };
}