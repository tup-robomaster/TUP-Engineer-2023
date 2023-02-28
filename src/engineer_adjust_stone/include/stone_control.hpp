#include "opencv2/opencv.hpp"
#include "iostream"

using namespace std;
using namespace cv;

namespace stone_control
{
  struct GoldData
  {
    bool IfR;
    RotatedRect box;
    vector<Point2f> points;
    int RDirection;
    Mat rvecs;
    Mat tvecs;
  };

  class FindGlod_
  {
  public:
    int threshold_value = 100;
    // Mat kernel_big = Mat::zeros(Size(21,21),CV_16FC1);
    // Mat kernel_s = Mat::zeros(Size(5,5),CV_16FC1);
  public:
    FindGlod_();
    ~FindGlod_();
    void getGold(Mat &Src, GoldData &goldData);
    void getPnP(GoldData goldData, Mat &Src);

  private:
    void preSrc(Mat Src, Mat &dst);
    bool sort_Y(Point p1, Point p2)
    {
      return p1.y < p2.y;
    };
    double pointsDistance(Point2f p1, Point2f p2)
    {
      return pow(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2), 0.5);
    };
  };

}