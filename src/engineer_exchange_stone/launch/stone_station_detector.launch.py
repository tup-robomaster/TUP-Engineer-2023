from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

import os 

def generate_launch_description():
        ld = LaunchDescription()
        share_path = get_package_share_directory('global_user')

        cam_config = os.path.join(share_path, 'config', 'camera_ros.yaml')
        stone_station_config = os.path.join(share_path, 'config', 'stone_station.yaml')

        usb_cam_node = Node(
            package = 'camera_driver',
            executable = 'usb_cam_pub',
            parameters = [cam_config],
            name = 'usb_cam_pub',
            output = 'screen'
        )
        stone_station_detector_node = Node(
            package = 'stone_station_detector',
            executable = 'stone_station_detector_node',
            parameters = [stone_station_config],
            name = 'stone_station_detector_node',
            output = 'screen'
        )

        ld.add_action(usb_cam_node)
        ld.add_action(stone_station_detector_node)

        return ld