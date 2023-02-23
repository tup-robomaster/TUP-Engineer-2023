import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription()
    ld.add_action(
        Node(
            name = "serialport",
            package = "serialport",
            executable = "serialport_node",
            namespace = "",
            output = "screen"
            # parameters=[param_file,
            #     {
            #       "debug_without_com": 'false',
            #       "baud": 115200 
            #     }
            # ]
        )
    )