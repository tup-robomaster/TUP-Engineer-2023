#ifndef INFERENCE_HPP_
#define INFERENCE_HPP_

//C++
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <iostream>

#include <inference_engine.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <fftw3.h>

#include "../../global_user/include/global_user.hpp"

using namespace std;
using namespace cv;
using namespace InferenceEngine;
using namespace global_user;

namespace stone_station_detector
{
  struct StationObject
  {
    Point2f apex[4];
    cv::Rect_<float> rect;
    int cls;
    int color;
    int area;
    float prob;
    std::vector<cv::Point2f> pts;
  };

  class Station_Detector
  {
  public:
    Station_Detector();
    ~Station_Detector();
    bool detect(cv::Mat &src, std::vector<StationObject>& objects);
    bool initModel(string path);

  private:
    int dw, dh;
    float rescale_ratio;//缩放比例
    Core ie;
    CNNNetwork network;           //网络
    ExecutableNetwork executable_network;   //可执行网络
    InferRequest infer_request;        //推理请求
    MemoryBlob::CPtr moutput;
    string input_name;
    string output_name;

    Eigen::Matrix<float,3,3> transfrom_matrix;
  };
}


#endif