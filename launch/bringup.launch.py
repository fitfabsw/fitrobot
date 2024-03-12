import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("rosbridge_server"),
                    "launch",
                    "rosbridge_websocket_launch.xml",
                )
            )
        ),
        # Node(
        #     package='fitrobot',
        #     executable='master_service',
        # ),
        Node(
            package='fitrobot',
            executable='master_async_service',
        ),
        Node(
            package='fitrobot',
            executable='save_map_service',
        ),
        Node(
            package='fitrobot',
            executable='list_map_service',
        ),
        # Node( 
        #     package='fitrobot',
        #     executable='check_robot_status',
        # ),
        # Node( TODO: Currently this will produce the 'Invalid frame ID "odom" passed to canTransform argument target_frameqq' error message and make this bringup.launch.py to fail
        #     package='fitrobotcpp',
        #     executable='check_robot_status',
        # ),
        Node(
            package='fitrobot',
            executable='waypoint_follower',
        ),
        Node(
            package='fitrobot',
            executable='list_station_service',
        ),
    ])
