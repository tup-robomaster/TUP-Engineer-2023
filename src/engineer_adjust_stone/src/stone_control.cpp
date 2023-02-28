#include "../include/stone_control.hpp"
// #include "../include/Debug.hpp"

namespace stone_control
{
  FindGlod_::FindGlod_()
  {
  }
  FindGlod_::~FindGlod_()
  {
  }
  void FindGlod_::preSrc(Mat Src, Mat &dst)
  {
    Mat gray = Mat::zeros(Src.size(), CV_8UC1);
    cvtColor(Src, gray, COLOR_BGR2GRAY);
    threshold(gray, dst, threshold_value, 255, THRESH_BINARY);
    // erode(dst,dst,kernel_s);
    // erode(dst,dst,kernel_big);
    // dilate(dst,dst,kernel_big);
    // dilate(dst,dst,kernel_s);
    bitwise_not(dst, dst);
    namedWindow("pre", WINDOW_NORMAL);
    resizeWindow("pre", 640, 480);
    imshow("pre", dst);
  }

  void FindGlod_::getGold(Mat &Src, GoldData &goldData)
  {
    Mat dst = Mat::zeros(Src.size(), CV_8UC1);
    preSrc(Src, dst);
    vector<vector<Point>> contours_point;
    vector<vector<Point>> first_points;
    findContours(dst, contours_point, RETR_TREE, CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours_point.size(); i++)
    {
      float temp_size = contourArea(contours_point[i]);
      RotatedRect box = minAreaRect(contours_point[i]);
      if (temp_size > 8000 && temp_size < 30000 && box.size.width / box.size.height < 1.3 && box.size.width / box.size.height > 0.7)
      {
        first_points.push_back(contours_point[i]);
      }
    }
    vector<RotatedRect> square;
    vector<vector<Point>> second_points;
    for (size_t i = 0; i < first_points.size(); i++)
    {
      RotatedRect box = minAreaRect(first_points[i]);
      float temp_size = contourArea(first_points[i]);
      if (temp_size > 8000 && temp_size < 30000 && temp_size / box.size.area() > 0.8)
      {
        square.push_back(box);
      }
      if (temp_size > 800 && temp_size < 30000)
      {
        second_points.push_back(first_points[i]);
      }
    }
    if (square.size() == 1)
    {
      goldData.IfR = true;
    }
    else
    {
      goldData.IfR = false;
    }
    if (goldData.IfR)
    {
      float temp_area = square[0].size.area();
      vector<Point> final_box;
      //   float temp_point_distance = 0;
      for (size_t i = 0; i < second_points.size(); i++)
      {
        RotatedRect box = minAreaRect(second_points[i]);
        if (box.size.area() / temp_area > 0.4 && box.size.area() / temp_area < 1.2)
        {
          final_box.push_back(box.center);
        }
      }
      if (final_box.size() == 4)
      {
        RotatedRect final_Gold = minAreaRect(final_box);
        Point2f *gold_points = new Point2f[4];
        final_Gold.points(gold_points);
        for (size_t i = 0; i < 4; i++)
        {
          double temp_distance = 0.0;
          Point2f temp_calc_Point;
          for (size_t j = 0; j < 4; j++)
          {
            double calc_dis = pointsDistance(gold_points[i], final_box[j]);
            if (calc_dis < temp_distance || temp_distance == 0.0)
            {
              temp_distance = calc_dis;
              temp_calc_Point = final_box[j];
            }
          }
          goldData.points.push_back(temp_calc_Point);
        }

        for (size_t i = 0; i < 4; i++)
        {

          line(Src, goldData.points[i], goldData.points[(i + 1) % 4], Scalar(125, 125, 125), 10);
        }

        goldData.box = final_Gold;
        getPnP(goldData, Src);
        if (square[0].center.x - final_Gold.center.x < 0 && square[0].center.y - final_Gold.center.y < 0)
        {
          goldData.RDirection = 3;
          cout << "down" << endl;
        }
        if (square[0].center.x - final_Gold.center.x < 0 && square[0].center.y - final_Gold.center.y > 0)
        {
          goldData.RDirection = 2;
          cout << "right" << endl;
        }
        if (square[0].center.x - final_Gold.center.x > 0 && square[0].center.y - final_Gold.center.y < 0)
        {
          goldData.RDirection = 1;
          cout << "left" << endl;
        }
        if (square[0].center.x - final_Gold.center.x > 0 && square[0].center.y - final_Gold.center.y > 0)
        {
          goldData.RDirection = 0;
          cout << "up" << endl;
        }

        circle(Src, square[0].center, 10, Scalar(255, 0, 255), -1);
        circle(Src, final_Gold.center, 10, Scalar(255, 0, 0), -1);
        circle(Src, goldData.points[0], 15, Scalar(255, 200, 0), 5);
        circle(Src, goldData.points[1], 15, Scalar(255, 200, 0), 5);
        circle(Src, goldData.points[2], 15, Scalar(255, 200, 0), 5);
        circle(Src, goldData.points[3], 15, Scalar(255, 200, 0), 5);
      }
    }
    else if (square.size() > 1)
    {
      circle(Src, square[0].center, 10, Scalar(180, 0, 180), -1);
      circle(Src, square[1].center, 10, Scalar(180, 0, 180), -1);
    }
  }

  void FindGlod_::getPnP(GoldData goldData, Mat &Src)
  {
    vector<Point3f> refenerce_ObjectPoint;
    vector<Point2f> refenerce_ImagePoint;
    refenerce_ImagePoint.push_back(goldData.points[0]);
    refenerce_ImagePoint.push_back(goldData.points[3]);
    refenerce_ImagePoint.push_back(goldData.points[1]);
    refenerce_ImagePoint.push_back(goldData.points[2]);
    refenerce_ObjectPoint.push_back(Point3f(0.0, 0.0, 0.0));
    refenerce_ObjectPoint.push_back(Point3f(0.0, 1.5, 0.0));
    refenerce_ObjectPoint.push_back(Point3f(1.5, 0.0, 0.0));
    refenerce_ObjectPoint.push_back(Point3f(1.5, 1.5, 0.0));

    Mat rvecs = Mat::zeros(3, 1, CV_64FC1);
    Mat tvecs = Mat::zeros(3, 1, CV_64FC1);

    Mat camera_Matrix = (cv::Mat_<double>(3, 3) << 1200.9, 0.000000000000, 134.8634, 0.000000000000, 1196.2, 366.1528, 0.000000000000, 0.000000000000, 1.000000000000);
    Mat dis_Coeffs = (cv::Mat_<double>(1, 5) << 0.0, 0.0, 0, 0, 0);

    solvePnP(refenerce_ObjectPoint, refenerce_ImagePoint, camera_Matrix, dis_Coeffs, rvecs, tvecs, false, SOLVEPNP_P3P);
    projectPoints(refenerce_ObjectPoint, rvecs, tvecs, camera_Matrix, dis_Coeffs, refenerce_ImagePoint);

    goldData.rvecs = rvecs;
    goldData.tvecs = tvecs;

    line(Src, refenerce_ImagePoint[0], refenerce_ImagePoint[1], Scalar(0, 0, 255), 2);
    line(Src, refenerce_ImagePoint[0], refenerce_ImagePoint[2], Scalar(0, 255, 0), 2);
    line(Src, refenerce_ImagePoint[0], refenerce_ImagePoint[3], Scalar(255, 0, 0), 2);
  }
}