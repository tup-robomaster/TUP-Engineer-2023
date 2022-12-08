#ifndef SERIALPORT_H
#define SERIALPORT_H
/**
 *@class  SerialPort
 *@brief  set serialport,recieve and send
 *@param  int fd
 */
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

#include <iostream>
#include <vector>

#include "../general/general.h"

#include "CRC_Check.h"
#include "../debug.h"

using namespace std;

namespace serialport
{
  #define TRUE 1
  #define FALSE 0
  
  //模式
  #define CmdID0 0x00; //关闭视觉
  #define CmdID1 0x01; //识别矿石
  #define CmdID3 0x03; //识别矿站
  // #define CmdID4 0x04; //大符
  
  // C_lflag
  #define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)
  
  //默认串口名
  const vector<string> DEFAULT_PORT = {"ttyUSB", "ttyACM"};
  //默认串口最大编号
  const int MAX_ITER = 3;

  //字节数为4的结构体
  typedef union
  {
    float f;
    unsigned char c[4];
  } float2uchar;
  
  typedef struct
  {
    string id;
    string alias;
    string path;
  } Device;
  
  // //字节数为2的uchar数据类型
  // typedef union
  // {
  //     int16_t d;
  //     unsigned char c[2];
  // } int16uchar;
  
  //用于保存目标相关角度和距离信息及瞄准情况
  typedef struct
  {
    float2uchar pitch_angle; //偏航角
    float2uchar yaw_angle;   //俯仰角
    float2uchar roll_angle;  //翻滚角
    float2uchar x_dis;  //前伸距离
    float2uchar y_dis;  //横移距离
    float2uchar z_dis;  //抬升距离
    int isSwitched;   //目标是否发生切换
    int isFindTarget; //当识别的图片范围内有目标
    // int isSpinning;   //目标是否处于陀螺状态
    // int ismiddle;     //设置1表示目标进入了可以开火的范围，设置0则表示目标尚未进入可开火的范围，默认置0
  } VisionData;
  
  class SerialPort
  {
  public:
      atomic_bool need_init = true;
      Device device;
      int fd;      //串口号
      int last_fd; //上一次串口号
      int speed;
      int baud;
      int mode;
      int databits, stopbits, parity;
      unsigned char rdata[255];                 // raw_data
      float quat[4]; //四元数
      float acc[3]; //加速度
      float gyro[3]; //角速度
      float bullet_speed;
      SerialPort(const string ID, const int BUAD);
      SerialPort(char *);
      bool initSerialPort();
      bool get_Mode();
      bool withoutSerialPort();
      Device getDeviceInfo(string path);
      Device setDeviceByID(std::vector<Device> devices);
      std::vector<Device> listPorts();
      void TransformData(const VisionData &data); //主要方案
      void send();
      void set_Brate();
      int set_Bit();
      void closePort();
      // void TransformDataFirst(int Xpos, int Ypos, int dis); //方案1
  private:
      unsigned char Tdata[40];                  // transfrom data
  
      string serial_id;
  
      float exchange_data(unsigned char *data); //将4个uchar合并成一个float
      bool get_pitch(unsigned char *data);
      bool get_yaw(unsigned char *data);
      bool get_roll(unsigned char *data);
      bool get_xdis(unsigned char *data);
      bool get_ydis(unsigned char *data);
      bool get_zdis(unsigned char *data);
  
  };

}
#endif // SERIALPORT_H
