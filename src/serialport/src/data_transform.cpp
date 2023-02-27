#include "../include/data_transform.hpp"

namespace serialport
{
    DataTransform::DataTransform()
    : logger_(rclcpp::get_logger("serialport"))
    {
    }

    DataTransform::~DataTransform()
    {
    }

    /**
     * @brief 数据转化
     * 
     * @param mode 模式位
     * @param vision_data 上位机发送的数据
     * @param trans_data 转化后的数据
     */
    void DataTransform::transformData(int mode, const VisionData &vision_data, uchar* trans_data)
    {
        trans_data[0] = 0xA5;
        trans_data[1] = mode;
        crc_check_.Append_CRC8_Check_Sum(trans_data, 3);
        if(mode == STONE_STATION_DETECT)
        {
            float float_data[] = {vision_data.pitch_angle, vision_data.yaw_angle, vision_data.roll_angle, vision_data.x_dis, vision_data.y_dis, vision_data.z_dis};
            float2UcharRawArray(float_data, 3, &trans_data[3]);
            trans_data[20] = vision_data.isFindTarget;
            trans_data[21] = 0x00;
            crc_check_.Append_CRC16_Check_Sum(trans_data, 64);
        }
        if(mode == STONE_DETECT)
        {
            
        }
    }
    
    /**
     * @brief 获取四元数
     * 
     * @param raw_data 原数据 
     * @param quat 转化后得到的四元数
     */
    void DataTransform::getQuatData(uchar* raw_data, vector<float>& quat)
    {
        ucharRaw2FloatVector(raw_data, 16, quat);
        return;
    }

    /**
     * @brief 获取角速度
     * 
     * @param raw_data 原数据
     * @param gyro 角速度数据
     */
    void DataTransform::getGyroData(uchar* raw_data, vector<float>& gyro)
    {
        ucharRaw2FloatVector(raw_data, 12, gyro);
        return;
    }

    /**
     * @brief 获取加速度数据
     * 
     * @param raw_data 原数据
     * @param acc 加速度数据
     */
    void DataTransform::getAccData(uchar* raw_data, vector<float>& acc)
    {
        ucharRaw2FloatVector(raw_data, 12, acc);
        return;
    }

    /**
     * @brief 获取裁判系统弹速
     * 
     * @param raw_data 原数据
     * @param bullet_speed 弹速
     */
    void DataTransform::getBulletSpeed(uchar* raw_data, float& bullet_speed)
    {
        bullet_speed = ucharRaw2Float(raw_data);
        return;
    }

    void DataTransform::getThetaAngle(uchar* raw_data, float& theta)
    {
        theta = ucharRaw2Float(raw_data);
        return;
    }

    /**
     * @brief 将4个uchar转换为float
     * @param data data首地址指针
     * @return
     */
    float DataTransform::ucharRaw2Float(uchar *data)
    {
        float float_data;
        float_data = *((float*)data);
        return float_data;
    };

    /**
     * @brief float转uchar
     * 
     * @param float_data float型数据
     * @return uchar* 返回uchar指针
     */
    uchar* DataTransform::float2UcharRaw(float float_data)
    {
        uchar* raw_data = nullptr;
        raw_data = (uchar*)(&float_data); 
        return std::move(raw_data);
    }   

    /**
     * @brief uchar原始数据转换为float vector
     * @param data 首地址指针
     * @param bytes 字节数
     * @param vec float vector地址
     */
    bool DataTransform::ucharRaw2FloatVector(uchar *data, int bytes, std::vector<float> &vec)
    {
        // std::vector<uchar*> pts;
        assert(bytes % 4 == 0);
        vec.clear();
        for (int i = 0; i < bytes; i += 4)
        {
            float float_data = ucharRaw2Float(&data[i]);
            vec.push_back(float_data);
        }
        return true;
    }
    
    /**
     * @brief float转uchar数组
     * 
     * @param float_data float型数据
     * @param num float型数组长度
     * @param raw_data uchar指针
     * @return true 
     * @return false 
     */
    bool DataTransform::float2UcharRawArray(float float_data[], int num, uchar* raw_data)
    {
        // memcpy(raw_data, float_data, sizeof(float) * num);
        for(int ii = 0; ii < num; ++ii)
        {
            uchar* data = float2UcharRaw(float_data[ii]);
            for(int jj = 0; jj < 4; ++jj)
            {
                raw_data[ii * 4 + jj] = data[jj];
            }
        }
        return true;
    }
} //namespace serialport