#ifndef DATA_TRANSFORM_HPP_
#define DATA_TRANSFORM_HPP_

#include <string.h>

//c++
#include <iostream>
#include <vector>
#include <string>

//opencv
#include <opencv2/opencv.hpp>

#include "CRC_Check.hpp"

using namespace std;
namespace serialport
{
    /**
     * @brief 模式选择（取消视觉，矿石识别，矿站识别）
     * 
     */
    enum MODE
    {
        CLOSE_VISION,
        STONE_DETACTE,
        STONE_STATION_DETACTE
    };
    
    // 字节数为4的结构体
    // typedef union
    // {
    //     float f;
    //     unsigned char c[4];
    // } float2uchar;

    // typedef union
    // {
    //     int16_t d;
    //     unsigned char c[2];
    // } int16uchar;

    //TODO:待改
    typedef struct VisionData
    {
      double timestamp;
      float pitch_angle; //偏航角
      float yaw_angle;   //俯仰角
      float roll_angle;  //翻滚角
      float x_dis;  //前伸距离
      float y_dis;  //横移距离
      float z_dis;  //抬升距离
      int isFindTarget; //当识别的图片范围内有目标
    } VisionData;
    
    class DataTransform
    {
    public:
        DataTransform();
        ~DataTransform();

        void transformData(int mode, const VisionData &data, uchar* trans_data); 
        void getQuatData(uchar* raw_data, vector<float>& quat);
        void getGyroData(uchar* raw_data, vector<float>& gyro);
        void getAccData(uchar* raw_data, vector<float>& acc);
        void getBulletSpeed(uchar* raw_data, float& bullet_speed);
        void getThetaAngle(uchar* raw_data, float& theta);

        int mode_;       
    private:
        CRC_Check crc_check_;
        float ucharRaw2Float(uchar *data); // 将4个uchar合并成一个float
        bool ucharRaw2FloatVector(uchar *data, int bytes, std::vector<float> &vec);
        uchar* float2UcharRaw(float float_data);
        bool float2UcharRawArray(float float_data[], int num, uchar* raw_data);
    };
} //namespace serialport

#endif