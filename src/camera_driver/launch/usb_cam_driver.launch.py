import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument #此函数功能是声明一个启动描述参数，该参数具有名称、默认值和文档
import os 

def generate_launch_description():

    share_path = get_package_share_directory('global_user')
    cam_config = os.path.join(share_path, 'config/camera_ros.yaml')

    return  LaunchDescription([
        DeclareLaunchArgument(name='params_file',
                              default_value=cam_config),

        Node(
            package = 'camera_driver',
            executable = 'usb_cam_pub',
            parameters =[LaunchConfiguration('params_file')],
            name = 'usb_cam_pub',
            output = 'screen',
            namespace = ''
        )

    ])