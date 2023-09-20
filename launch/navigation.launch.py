import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    map_arg = DeclareLaunchArgument("map", default_value="")
    map_lc = LaunchConfiguration("map")

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                ),
            ),
            launch_arguments = {"map": map_lc, "params_file": "/home/pi/zbotartic_ws/src/articubot_one/config/nav2_params.yaml"}.items()
        ),
    ])