# 此包用于存放全局参数配置文件

## 1.config目录文件参数配置说明
- stone_station.yaml 
    -
    /stone_station_detector

| 参数名            | 释义             | 配置说明               |
| ----------------- | ---------------- | ---------------------- |
|camera_name|   相机型号    |   `跑程序前得改！！！`  |
| camera_param_path | 相机配置参数路径 |                        |
| network_path      | 网络权重路径     |                        |
|debug         |        是否开启调试模式，此模式会发布一些调试需要的功能和消息      |`调试程序前得改！！！`             |
|detect_red |   是否检测红色    |   |
| show_fps          | 是否显示帧率     | 调试可视化             |
| show_img          | 是否显示图像     | `调试程序前得改！！！` |
|show_aim_cross         |   是否画出十字线           | 调试可视化            |
|show_target       |     是否框出识别到的所有装甲板         | 调试可视化，`调试时得改！！！`            |
|show_transform|是否打印tf转换后的信息 |调试时输出 |
|print_letency      |    是否打印耗时信息（包含网络推理耗时/数据处理耗时/节点运行总延时等）          |调试时输出             |
|print_target_info       |是否打印目标信息              | 调试时输出            | \

- camera_ros.yaml 
    -  
    - /camera_driver

| 参数名            | 释义             | 配置说明               |
| ----------------- | ---------------- | ---------------------- |
|cam_id |   usb相机ID号 |   `调试程序前得改！！！`  |
|using_video    |   是否使用视频进行调试    |  |
|video_path |   视频路径    | 视频在src/camera_driver/video  |
|save_video |   是否保存视频   |   |