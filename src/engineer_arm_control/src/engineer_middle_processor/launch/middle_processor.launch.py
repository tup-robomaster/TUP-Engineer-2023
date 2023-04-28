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
        middle_processor_node = Node(
            package = 'middle_processor',
            executable = 'usb_cam_pub',
            name = 'usb_cam_pub',
            output = 'screen'
        )

        ld.add_action(middle_processor_node)

        return ld