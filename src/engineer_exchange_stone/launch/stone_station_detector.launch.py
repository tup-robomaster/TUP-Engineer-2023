from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument #此函数功能是声明一个启动描述参数，该参数具有名称、默认值和文档
# import os 

def generate_launch_description():
        ld = LaunchDescription()
    # cam_config = os.path.abspath('config/camera.yml')

        # DeclareLaunchArgument(name = 'param_file',
        #                       default_value = cam_config
        # ),
        usb_cam_node = Node(
            package = 'camera_driver',
            executable = 'usb_cam_pub',
            name = 'usb_cam_pub',
            output = 'screen'
        )
        stone_station_detector_node = Node(
            package = 'stone_station_detector',
            executable = 'stone_station_detector_node',
            # parameters = [cam_config],
            name = 'stone_station_detector_node',
            output = 'screen',
            # namespace = 'camera_driver'
        )

        ld.add_action(usb_cam_node)
        ld.add_action(stone_station_detector_node)

        return ld