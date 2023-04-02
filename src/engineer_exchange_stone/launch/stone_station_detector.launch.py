from argparse import Namespace
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
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

        tf_cam_to_base_node_ = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0.217", "0.423", "0.170", "-1.570796325", "0", "-1.570796325", "base_link", "cam_link"]
        )

        tf_arm_to_base_node_ = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0.423", "0", "0.382", "0", "0", "0", "base_link", "arm_link"]
        )

        ld.add_action(usb_cam_node)
        ld.add_action(stone_station_detector_node)
        ld.add_action(tf_cam_to_base_node_)
        ld.add_action(tf_arm_to_base_node_)

        return ld