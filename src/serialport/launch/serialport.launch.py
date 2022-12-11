import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription():
    ld.add_action(
        None(
            name = "serialport",
            package = "serialport",
            executable = "serial_driver",
            namespace = "",
            output = "screen"
        )
    )