# TUP_Engineer
沈阳航空航天大学TUP战队2023赛季工程视觉开发

## Version
V1.0

程序基于ros-galactic框架

## 开发日志
Date:2022-11-11 开始搭建程序框架。

Date:2022-11-30 完成调矿模块编写。

Date:2022-12-01 开始编写矿站识别模块，以及串口通讯部分。

Date:2022-12-20 基本完成对矿站识别部分的编写，除网络部分。

Date:2022-01-08 完成机械臂轨迹规划部分框架的搭建。

Data:2022-02-01 调整方案。

Data:2023-03-15 使用ue5模型数据集2000张，重新标定相机参数，完成初版识别程序，但识别效果不加，推断为数据集不够，识别解算参数有所改善。

Data:2023-03-17 第一阶段识别程序完工，目前问题主要有，识别解算参数误差过大，解算程序有问题，增加数据集数量后，识别效果有所改善，接下来主要调试和更改解算部分。

Data:2023-03-25 由于解算出现误差，查找资料，做了多种尝试，最后定位为相机内参问题，多次重新标定，最后控制误差在1cm左右，数据集扭曲，导致整个模型崩溃，重新标定。

Data:2023-04-01 加入了TF2，包括相机系，矿站系，车体系，车臂系的转换关系，将自义定消息换成geometry_msgs中的消息，最后发布。

Data:2023-04-08 加入Marker,可以看到矿站实体化，重新整合代码，规范代码，写成组件形式启动，研究并尝试moveit开发。

## 问题
若出现： Failed to load module "canberra-gtk-module"

sudo apt-get install libcanberra-gtk-module

若出现： Could not find a package configuration file provided by
  "camera_info_manager" with any of the following names

sudo apt install ros-galactic-camera-info-manager 

若出现： Could not find a package configuration file provided by "tf2_geometry_msgs"
  with any of the following names

sudo apt install ros-galactic-tf2-geometry-msgs

## 使用说明
### 1）Env
### 2）Compile
    colcon build --symlink-install --packages-up-to global_user global_interface
    . install/setup.bash
    colcon build --symlink-install
    . install/setup.bash
#### 1.矿站识别调试
    调试说明：
    参数配置文件位置：src/global_user/config/stone_station.yaml
    对应的launch文件位置：src/global_user/launch/engineer_exchange_stone_bringup.launch.py
    1.首先根据实际情况更改相机类型（camera_type）和型号(camera_name)（包括stone_station_detector空间下对应的参数），调试视频则把camera_type赋为3；
    2.与下位机通信调试时将配置文件中的using_imu参数改为true，同时把launch文件中的using_imu参数置为True；

运行命令：

    ros2 launch global_user engineer_exchange_stone_bringup.launch.py