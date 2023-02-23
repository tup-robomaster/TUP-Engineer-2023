// #ifndef SERIALPORT_H
// #define SERIALPORT_H
// /**
//  *@class  SerialPort
//  *@brief  set serialport,recieve and send
//  *@param  int fd
//  */
// #include <atomic>
// #include <sys/types.h>
// #include <unistd.h>
// #include <sys/stat.h>
// #include <dirent.h>
// #include <fcntl.h>
// #include <termios.h>
// #include <stdio.h>
// #include <stdlib.h>
// #include <errno.h>
// #include <sys/ioctl.h>
// #include <linux/netlink.h>

// #include <iostream>
// #include <vector>

// #include "../include/CRC_Check.hpp"
// // #include "../../../global_user/include/global_user/global_user.hpp"

// using namespace std;
// using namespace global_user;

// // namespace serialport
// // {
// //   #define TRUE 1
// //   #define FALSE 0
  
// //   //模式
// //   #define CmdID0 0x00; //关闭视觉
// //   #define CmdID1 0x01; //识别矿站
// //   #define CmdID3 0x03; //识别矿石
  
// //   // C_lflag
// //   #define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)
  
// //   //默认串口名
// //   const vector<string> DEFAULT_PORT = {"ttyUSB", "ttyACM"};
// //   //默认串口最大编号
// //   const int MAX_ITER = 3;

// //   //字节数为4的结构体
// //   typedef union
// //   {
// //     float f;
// //     unsigned char c[4];
// //   } float2uchar;
  
// //   typedef struct
// //   {
// //     string id;
// //     string alias;
// //     string path;
// //   } Device;
  
// //   // //字节数为2的uchar数据类型
// //   // typedef union
// //   // {
// //   //     int16_t d;
// //   //     unsigned char c[2];
// //   // } int16uchar;
  
// //   //用于保存目标相关角度和距离信息及瞄准情况
// //   typedef struct
// //   {
// //     float2uchar pitch_angle; //偏航角
// //     float2uchar yaw_angle;   //俯仰角
// //     float2uchar roll_angle;  //翻滚角
// //     float2uchar x_dis;  //前伸距离
// //     float2uchar y_dis;  //横移距离
// //     float2uchar z_dis;  //抬升距离
// //     int isFindTarget; //当识别的图片范围内有目标

// //   } VisionData;
  
// //   struct SerialData
// //   {    
// //     bool is_initialized;
// //     Device device;
// //     int fd;       // 当前串口号
// //     int last_fd;  // 上一次串口号
// //     int speed;
// //     int baud;
// //     int databits;
// //     int stopbits;
// //     int parity;
// //     unsigned char rdata[255]; // raw_data
// //   };

// //   class SerialPort
// //   {
// //   public:
       
// //       SerialData serial_data_;
      
// //       atomic_bool need_init = true;
// //       int mode;
// //       unsigned char rdata[255];                 // raw_data
// //       float pitch; //俯仰角
// //       float yaw;   //偏航角度
// //       float roll;  //翻滚角度
// //       float x_dis; //前伸距离
// //       float y_dis; //横移距离
// //       float z_dis; //抬升距离
// //       SerialPort(const string ID, const int BUAD);
// //       SerialPort(char *);
// //       bool initSerialPort();
// //       bool get_Mode();
// //       bool withoutSerialPort();
// //       Device getDeviceInfo(string path);
// //       Device setDeviceByID(std::vector<Device> devices);
// //       std::vector<Device> listPorts();
// //       void TransformData(const VisionData &data); //主要方案
// //       void send();
// //       void set_Brate();
// //       int set_Bit();
// //       void closePort();
// //       // void TransformDataFirst(int Xpos, int Ypos, int dis); //方案1
// //   private:
// //       CRC_Check crc_check_;
// //       rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
// //       bool debug_without_com_; //是否无串口调试
// //       int mode; 

// //       unsigned char Tdata[40];                  // transfrom data
  
// //       string serial_id;
  
// //       float exchange_data(unsigned char *data); //将4个uchar合并成一个float
// //       bool get_pitch(unsigned char *data);
// //       bool get_yaw(unsigned char *data);
// //       bool get_roll(unsigned char *data);
// //       bool get_xdis(unsigned char *data);
// //       bool get_ydis(unsigned char *data);
// //       bool get_zdis(unsigned char *data);
  
// //   };

// // }
// // #endif // SERIALPORT_H



// #define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL) //C_lflag
// const std::vector<std::string> DEFAULT_PORT = {"ttyUSB", "ttyACM"}; //默认串口名
// constexpr int MAX_ITER = 3; //默认串口最大编号
// #define TRUE 1
// #define FALSE 0

// // 模式
//   #define CmdID0 0x00; //关闭视觉
//   #define CmdID1 0x01; //识别矿站
//   #define CmdID3 0x03; //识别矿石

// using namespace std;
// using namespace global_user;
// namespace serialport
// {
//     //字节数为4的结构体
//     typedef union
//     {
//         float f;
//         unsigned char c[4];
//     } float2uchar;

//     typedef struct
//     {
//         string id;
//         string alias;
//         string path;
//     } Device;

//     // 字节数为2的uchar数据类型
//     // typedef union
//     // {
//     //     int16_t d;
//     //     unsigned char c[2];
//     // } int16uchar;

//     //用于保存目标相关角度和距离信息及瞄准情况
//     typedef struct
//     {
//       float2uchar pitch_angle; //偏航角
//       float2uchar yaw_angle;   //俯仰角
//       float2uchar roll_angle;  //翻滚角
//       float2uchar x_dis;  //前伸距离
//       float2uchar y_dis;  //横移距离
//       float2uchar z_dis;  //抬升距离
//       int isFindTarget; //当识别的图片范围内有目标
//     } VisionData;

//     struct SerialData
//     {
//         bool is_initialized;
//         Device device;
//         int fd;       // 当前串口号
//         int last_fd;  // 上一次串口号
//         int speed;
//         int baud;
//         int databits;
//         int stopbits;
//         int parity;
//         unsigned char rdata[255]; // raw_data
//     };

//     /**
//      *@class  SerialPort
//      *@brief  Set serialport, recieve and send.
//      *@param  int fd
//     */
//     class SerialPort
//     {
//     public:
//         SerialPort();
//         SerialPort(const string ID, const int BUAD, bool debug_without_com, bool is_sentry_mode);
//         ~SerialPort();
    
//         SerialData serial_data_;              
//         bool get_Mode(int bytes);
//         bool initSerialPort();
//         void transformData(const VisionData &data); //主要方案
//         void send();
//     private:
//         bool withoutSerialPort();
//         Device getDeviceInfo(string path);
//         Device setDeviceByID(std::vector<Device> devices);
//         std::vector<Device> listPorts();
//         void set_Brate();
//         int set_Bit();
//         void closePort();
//     private:
//         bool is_sentry_mode_;
//         CRC_Check crc_check_;
//         string serial_id_;
//         rclcpp::Time timestamp_;

//         unsigned char Tdata[30];                  // transfrom data
//         float exchange_data(unsigned char *data); // 将4个uchar合并成一个float
//         bool get_pitch(unsigned char *data);
//         bool get_yaw(unsigned char *data);
//         bool get_roll(unsigned char *data);
//         bool get_xdis(unsigned char *data);
//         bool get_ydis(unsigned char *data);
//         bool get_zdis(unsigned char *data);
    
//         rclcpp::Logger logger_;
//     public:
//         rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
//         bool debug_without_com_; //是否无串口调试
//         bool print_imu_data_; 
//         int mode;       
//         float bullet_speed_;
//         float quat[4]; //四元数
//         float acc[3];  //加速度
//         float gyro[3]; //角速度
//     };
// } // namespace serialport

// #endif // SERIALPORT_HPP_


#ifndef SERIALPORT_HPP_
#define SERIALPORT_HPP_

//linux
#include <atomic>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>

//c++
#include <iostream>
#include <vector>

//ros
#include <rclcpp/rclcpp.hpp>

#include "./CRC_Check.hpp"
#include "../../global_user/include/global_user.hpp"

#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL) //C_lflag
const std::vector<std::string> DEFAULT_PORT = {"ttyUSB", "ttyACM"}; //默认串口名
constexpr int MAX_ITER = 3; //默认串口最大编号

using namespace std;
using namespace global_user;
namespace serialport
{
    typedef struct
    {
        string id;
        string alias;
        string path;
    } Device;

    struct SerialData
    {
        bool is_initialized;
        Device device;
        int fd;       // 当前串口号
        int last_fd;  // 上一次串口号
        int speed;
        int baud;
        int databits;
        int stopbits;
        int parity;
        unsigned char rdata[64]; // raw_data
    };

    /**
     *@class  SerialPort
     *@brief  Set serialport, recieve and send data.
     *@param  int fd
    */
    class SerialPort
    {
    public:
        SerialPort();
        SerialPort(const string ID, const int BUAD, bool debug_without_com);
        ~SerialPort();

        //收发数据
        bool receiveData(int lens = 64);
        void sendData(int bytes_num = 64);

        //开闭串口
        bool openPort();
        void closePort();

        uchar Tdata[64]; 
        SerialData serial_data_;
    private:
        string serial_id_;
        CRC_Check crc_check_;
        
        bool setBit();
        void setBrate();
        std::vector<Device> listPorts();
        Device getDeviceInfo(string path);
        Device setDeviceByID(std::vector<Device> devices);

        bool withoutSerialPort();
        
    private:
        rclcpp::Logger logger_;
        rclcpp::Time timestamp_;

    public:
        rclcpp::Clock steady_clock_{RCL_STEADY_TIME};
        bool debug_without_com_; //是否无串口调试
    };
} // namespace serialport

#endif // SERIALPORT_HPP_
