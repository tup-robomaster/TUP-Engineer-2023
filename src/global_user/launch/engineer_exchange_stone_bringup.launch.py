import os
import yaml
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression
from launch_ros.descriptions import ComposableNode
from launch.substitutions import ThisLaunchFileDir
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    detector_node_launch_file = os.path.join(get_package_share_directory('stone_station_detector'), 'launch/stone_station_detector.launch.py')
    
    # camera_param_file = os.path.join(get_package_share_directory('global_user'), 'config/camera_ros.yaml')
    stone_station_param_file = os.path.join(get_package_share_directory('global_user'), 'config/stone_station.yaml')
    
    # camera_type = LaunchConfiguration('camera_type')
    # use_serial = LaunchConfiguration('using_imu')

    # declare_camera_type = DeclareLaunchArgument(
    #     name='camera_type',
    #     default_value='usb',
    #     description='usb usb_01 usb_02'
    # )

    # declare_use_serial = DeclareLaunchArgument(
    #     name='using_imu',
    #     default_value='False',
    #     description='debug without serial port.'
    # )

    # with open(camera_param_file, 'r') as f:
    #     usb_cam_params = yaml.safe_load(f)['/usb_cam_driver']['ros__parameters']

    with open(stone_station_param_file, 'r') as f:
        stone_station_detector_params = yaml.safe_load(f)['/stone_station_detector']['ros__parameters']
    
    return LaunchDescription([
        # declare_camera_type,
        # declare_use_serial,

        # Node(
        #     package='serialport',
        #     executable='serialport_node',
        #     name='serialport',
        #     output='screen',
        #     # emulate_tty=True,
        #     # parameters=[{
        #     #     'using_imu': False,
        #     #     'debug_without_com': True
        #     # }],
        #     # condition=IfCondition(PythonExpression(["'", use_serial, "' == 'True'"]))
        # ),
        

        ComposableNodeContainer(
            name='stone_station_detector_container',
            namespace='',
            output='screen',
            package='rclcpp_components',
            executable='component_container',
            # condition=IfCondition(PythonExpression(["'", camera_type, "' == 'usb'"])),
            composable_node_descriptions=[

                ComposableNode(
                    package='camera_driver',
                    plugin='camera_driver::UsbCamNode',
                    name='camera_driver',
                    # parameters=[usb_cam_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
                ComposableNode(
                    package='stone_station_detector',
                    plugin='stone_station_detector::DetectorNode',
                    name='stone_station_detector',
                    parameters=[stone_station_detector_params],
                    extra_arguments=[{
                        'use_intra_process_comms':True
                    }]
                ),
            ],
        ),

        Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0.217", "0.423", "0.423", "-1.570796325", "0", "-1.570796325", "base_link", "cam_link"]
        ),

        Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=["0.423", "0", "0.421", "0", "0", "0", "base_link", "arm_link"]
        )

    ])