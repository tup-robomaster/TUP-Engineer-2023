from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument #此函数功能是声明一个启动描述参数，该参数具有名称、默认值和文档
import os 

def generate_launch_description():
    # cam_config = os.path.abspath('config/camera.yml')
    return  LaunchDescription([

        # DeclareLaunchArgument(name = 'param_file',
        #                       default_value = cam_config
        # ),

        Node(
            package = 'camera_driver',
            executable = 'usb_cam_pub',
            # parameters = [cam_config],
            name = 'usb_cam_pub',
            output = 'screen',
            # namespace = 'camera_driver'
        )

    ])