#include <vector>
#include <thread>
#include <memory>
#include <string>
#include <iterator>
#include <unistd.h>
#include <string>

#include <fstream>
#include <yaml-cpp/yaml.h>
#include <fmt/format.h>
#include <fmt/color.h>
#include <glog/logging.h>

#include <opencv2/opencv.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>

// #include "rclcpp/rclcpp.hpp"

namespace global_user
{
  const std::string config_file_stone_station = "src/global_user/config/stone_station.yaml";

  class global
  {
  public:
    global_user();
    ~global_user()

  private:
    std::string config_path[1];
  };

  struct TaskData
  {
    int mode; 
    cv::Mat img;
    Eigen::Quaterniond quat;
    int timestamp;
  };

  template<typename T>
  bool initMatrix(Eigen::MatrixXd &matrix,std::vector<T> &vector)
  {
    int cnt = 0;
    for(int row = 0;row < matrix.rows();row++)
    {
      for(int col = 0;col < matrix.cols();col++)
      {
        matrix(row,col) = vector[cnt];
        cnt++;
      }
    }
    return true;
  }

  float calcTriangleArea(cv::Point2f pts[3]);
  float calcTetragonArea(cv::Point2f pts[4]);
  double rangedAngleRad(double &angle);

  std::string symbolicToReal(std::string path);
  std::string relativeToFull(std::string relative, std::string src);
  std::string treeToPath(std::vector<std::string> &tree);
  std::string getParent(std::string path);
  std::vector<std::string> readLines(std::string file_path);
  std::vector<std::string> generatePathTree(std::string path);
  
  Eigen::Vector3d rotationMatrixToEulerAngles(Eigen::Matrix3d &R);
  Eigen::Vector3d calcDeltaEuler(Eigen::Vector3d euler1, Eigen::Vector3d euler2);
  Eigen::AngleAxisd eulerToAngleAxisd(Eigen::Vector3d euler);
  Eigen::Matrix3d eulerToRotationMatrix(Eigen::Vector3d &theta);
  float calcDistance(cv::Point2f& p1, cv::Point2f& p2);
}
