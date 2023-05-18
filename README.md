# TUP_Engineer
沈阳航空航天大学TUP战队2023赛季工程视觉开发

## Version
V1.0

程序基于ros-galactic框架

## 1.Brief


功能模块流程图：

## 2.Introduction
- Packages

|    Name   |      Function  | Description |
|    ---    |        ---     |    ---         |
|global_user|     定义全局接口 |包含通用函数/结构体/枚举类型|
|global_interface|自定义消息   |全局msg定义|
|serialport|      串口驱动     |定义与下位机的通信协议|
|camera_driver|相机驱动        | 包含USB相机驱动接口|
|engineer_stone_station_detector| 矿站检测节点 | 识别出目标并发布|
|engineer_arm_control| 机械臂控制节点 | 进行机械臂的轨迹规划 |
|engineer_adjust_stone| 矿石检测节点 | 进行矿石的识别|

- Custom messages

| Message           |          Topic            |   Description |
|  ---              |          ---              |      ---      |
| Stone.msg | /stone_pub| 目标矿石消息 |
| Target.msg     | /target_pub                |  目标矿站消息|
| Serial.msg        | /serial_msg               |   串口消息|

## 3.Environment

|Library | URL |  Description |
| ---    | --- | ---          |
|OpenVINO| https://www.intel.com/content/www/us/en/developer/tools/openvino-toolkit/download.html| 目前使用的版本是OpenVINO2022.1，直接offline下载一键安装即可，需要给权限 |
|glog| https://github.com/google/glog/releases/tag/v0.5.0|
yaml-cpp| https://github.com/jbeder/yaml-cpp/releases/tag/yaml-cpp-0.5.3  | 需指定将yaml-cpp库编译成共享库
Ceres   | http://ceres-solver.org/installation.html | Click  "latest stable release"
Eigen   | https://gitlab.com/libeigen/eigen/-/releases/3.4.0       | 编译此库前首先对ceres库进行编译
OpenCV  |https://github.com/opencv/opencv/tree/4.2.0 \ https://github.com/opencv/opencv_contrib/tree/4.x | 编译时两个包放一块，注意编译时需传入指定参数 |

## 4.Debug
- Prepare
  - 
  - 参数配置文件位置：src/global_user/config/stone_station.yaml(参数配置详解见:[Stone_Station_Config](src/global_user/README.md)  )
  - 对应的launch文件位置：src/global_user/launch/engineer_exchange_stone_bringup.launch.py
  - 首先根据实际情况更改相机号（cam_id）；
  - 与下位机通信调试时将配置文件中的using_imu参数改为true，同时把launch文件中的using_imu参数置为True；
- Compile
  - 
    
        colcon build --symlink-install --packages-up-to global_user global_interface

        . install/setup.bash

        colcon build --symlink-install

        . install/setup.bash

- Command
  - 
        
        ros2 launch global_user engineer_exchange_stone_bringup.launch.py
- Startup
  - 

  - Step1:设置shell脚本权限
        
        sudo chmod 777 your_shell_script_path 
  - Step2:配置程序启动首选项

        gnome-terminal -- your_shell_script_path
- Log
  -  
  - 日志保存路径设置

        sudo gedit ~/.bashrc
        export ROS_HOME=~/ros_logs

## 5.Develop log

| Date |  Issue   |   Debug    |
| ---  |  ---     | ---   | 
|2022-11-11| 开始搭建程序框架。|
|2022-11-30| 完成调矿模块编写（并不完整待开发）。|
|2022-12-01| 开始编写矿站识别模块，以及串口通讯部分。|
|2022-12-20| 基本完成对矿站识别部分的编写，除网络部分。|
|2022-01-08| 完成机械臂轨迹规划部分框架的搭建。|
|2022-02-01| 调整方案。|
|2023-03-15| 使用ue5模型数据集2000张，重新标定相机参数，完成初版识别程序，但识别效果不加，推断为数据集不够，识别解算参数有所改善。|
|2023-03-17| 第一阶段识别程序完工，目前问题主要有，识别解算参数误差过大，解算程序有问题，增加数据集数量后，识别效果有所改善，接下来主要调试和更改解算部分。|
|2023-03-25| 由于解算出现误差，查找资料，做了多种尝试，最后定位为相机内参问题，多次重新标定，最后控制误差在1cm左右，数据集扭曲，导致整个模型崩溃，重新标定。|
|2023-04-01| 加入了TF2，包括相机系，矿站系，车体系，车臂系的转换关系，将自义定消息换成geometry_msgs中的消息，最后发布。|
|2023-04-08| 加入Marker,可以看到矿站实体化，重新整合代码，规范代码，写成组件形式启动，研究并尝试MoveIt2开发。|
|2023-04-15| 构建工程urdf模型，并尝试使用moveit_setup_assistant转成ros可规划文件，并尝试在rviz中规划。|
|2023-04-22| | 
|2023-05-06| 初步调试，结果显示出还有几个问题需要解决，第一个识别问题，需继续标注数据集进行训练，第二个角度解算，位置解算数据错误或有较大误差，这是由于电控初始角度信息，和视觉设定不一致造成的，给一定的角度位置补偿即可，写数据集转换，合成脚本。|
|2023-05-13| 调试过程中发现，须设置定值发送下位机，所以进行了识别的数据处理，经处理好的数据以定值的形式发送下位机。|
|2023-05-20| 更改程序逻辑，如果模式为0则进行常规发布动态识别数据，如果模式为1则进行多帧处理数据之后取定值发布，保证有实时的数据反馈，并更换坐标系位置为标准朝向。|

## 6.Supplement

| Issue    |  Debug  |  Add   |
| ---            |   ---   | ----    |
|Failed to load module "canberra-gtk-module"|sudo apt-get install libcanberra-gtk-module|
|Could not find a package configuration file provided by "camera_info_manager" with any of the following names|sudo apt install ros-galactic-camera-info-manager|
|Could not find a package configuration file provided by "tf2_geometry_msgs"with any of the following names|sudo apt install ros-galactic-tf2-geometry-msgs
|串口权限永久解决|1) whoami --查看用户名2) sudo usermod -aG dialout username|
