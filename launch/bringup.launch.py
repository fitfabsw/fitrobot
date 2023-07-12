import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory("articubot_one"),
        #             "launch",
        #             "launch_robot.launch.py",
        #         )
        #     )
        # ),
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("rosbridge_server"),
                    "launch",
                    "rosbridge_websocket_launch.xml",
                )
            )
        ),
        Node(
            package='fitrobot',
            executable='server_node',
        ),
        Node(
            package='fitrobot',
            executable='bridge_node',
        ),
        # Node(
        #     package='fitrobot',
        #     executable='tf_converter_node',
        # ),
    ])