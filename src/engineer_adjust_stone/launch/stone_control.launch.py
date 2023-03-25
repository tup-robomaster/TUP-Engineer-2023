from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
# from launch.actions import DeclareLaunchArgument #此函数功能是声明一个启动描述参数，该参数具有名称、默认值和文档
# import os 

def generate_launch_description():
        ld = LaunchDescription()
        
        usb_cam_node = Node(
            package = 'camera_driver',
            executable = 'usb_cam_pub',
            name = 'usb_cam_pub',
            output = 'screen'
        )
        stone_control_node = Node(
            package = 'stone_control',
            executable = 'stone_control_node',
            # parameters = [cam_config],
            name = 'stone_control_node',
            output = 'screen',
            # namespace = 'camera_driver'
        )

        ld.add_action(usb_cam_node)
        ld.add_action(stone_control_node)

        return ld
