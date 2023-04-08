#ifndef INFERENCE_HPP_
#define INFERENCE_HPP_

// c++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

// openvino
#include <openvino/openvino.hpp>
// #include <format_reader_ptr.h>

// opencv
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

// eigen
#include <Eigen/Core>

#include "../../global_user/include/global_user.hpp"

using namespace std;
using namespace cv;
using namespace InferenceEngine;
using namespace global_user;

namespace stone_station_detector
{
  struct StationObject
  {
    cv::Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
  };

  class StationDetector
  {
  public:
    StationDetector();
    ~StationDetector();
    bool detect(cv::Mat &src, std::vector<StationObject> &objects);
    bool initModel(string path);

  private:
    int dw, dh;
    float rescale_ratio; // 缩放比例

    ov::Core core;
    std::shared_ptr<ov::Model> model; // 网络
    ov::CompiledModel compiled_model; // 可执行网络
    ov::InferRequest infer_request;   // 推理请求
    ov::Tensor input_tensor;

    string input_name;
    string output_name;

    // TODO:
    Eigen::Matrix<float, 3, 3> transfrom_matrix;
  };
}

#endif