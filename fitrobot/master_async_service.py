import os
import asyncio
from typing import List
from typing import Text
from typing import Tuple
from collections import OrderedDict
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from ros2launch.api import get_share_file_path_from_package
from fitrobot_interfaces.srv import Master, TerminateProcess


class MasterAsyncService(Node):
    def __init__(self):
        super().__init__("master_async_service")
        self.srv = self.create_service(Master, "run_slam_or_navigation", self.master_callback)
        self.terminate_service = self.create_service(
            TerminateProcess, "terminate_slam_or_navigation", self.terminate_slam_or_navigation_callback
        )
        robot_type = os.getenv("ROBOT_TYPE", "lino")
        if robot_type == "lino":
            self.declare_parameter("active_nav_map", "office_res002_0914.yaml")
        elif robot_type == "artic":
            self.declare_parameter("active_nav_map", "office_res002_0523.yaml")
        self.launch_service = None
        self.event_loop = asyncio.get_event_loop()

    async def master_callback(self, request, response):
        self.get_logger().info("master_callback start")
        request_action = request.request_action
        request_param = request.request_param
        # Launch the slam/navigation in an asynchronous manner
        if request_action == "slam":
            asyncio.ensure_future(self.run_slam())
        elif request_action == "navigation":
            asyncio.ensure_future(self.run_navigation(request_param))
        else:
            pass
        self.get_logger().info("master_callback end")
        return response

    def run(self):
        asyncio.ensure_future(self.ros_spin())
        self.event_loop.run_forever()

    async def ros_spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
            await asyncio.sleep(0.01)

    def terminate_slam_or_navigation_callback(self, request, response):
        self.get_logger().info("Terminating navigation...")
        asyncio.ensure_future(self.clean_up())
        response.success = True
        self.get_logger().info("end")
        return response

    async def clean_up(self):
        if self.launch_service:
            await self.launch_service.shutdown()
            self.launch_service = None

    @staticmethod
    def parse_launch_arguments(launch_arguments: List[Text]) -> List[Tuple[Text, Text]]:
        parsed_launch_arguments = OrderedDict()  # type: ignore
        for argument in launch_arguments:
            count = argument.count(":=")
            if (
                count == 0
                or argument.startswith(":=")
                or (count == 1 and argument.endswith(":="))
            ):
                raise RuntimeError(
                    "malformed launch argument '{}', expected format '<name>:=<value>'"
                    .format(argument)
                )
            name, value = argument.split(":=", maxsplit=1)
            parsed_launch_arguments[name] = value  # last one wins is intentional
        return parsed_launch_arguments.items()

    async def run_navigation(self, map_name):
        self.get_logger().info("Starting navigation service...")
        await self.clean_up()  # Clean up any existing launched tasks
        log_file_path = "/tmp/master_service_navigation_log.txt"

        def to_file(event):
            with open(log_file_path, "a") as log_file:
                log_file.write(event.text.decode())

        self.get_logger().info("...1")

        # for simualtions
        # maploc = os.path.join(
        #     get_package_share_directory("linorobot2_navigation"), "maps"
        # )
        # map_path = f"map:={maploc}/{map_name}"

        # for real robots
        maploc = os.path.join(get_package_share_directory("fitrobot"), "maps")
        map_path = f"map:={maploc}/{map_name}"

        robot_type = os.getenv("ROBOT_TYPE", "lino")

        launch_file_name = "navigation.launch.py"
        if robot_type == "artic":
            launch_file_name = "navigation_keepout.launch.py"
            path = get_share_file_path_from_package(
                package_name="articubot_one", file_name=launch_file_name
            )
        elif robot_type == "lino":
            launch_file_name = "navigation_keepout.launch.py"
            # launch_file_name = "navigation.launch.py"
            path = get_share_file_path_from_package(
                package_name="linorobot2_navigation", file_name=launch_file_name
            )
        launch_file_arguments = [map_path, "__log_level:=error"]
        self.get_logger().info("...2")
        self.launch_service = launch.LaunchService(debug=False)
        launch_description = launch.LaunchDescription(
            [
                RegisterEventHandler(
                    OnProcessIO(
                        on_stdout=to_file,
                        on_stderr=to_file,
                    )
                ),
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.AnyLaunchDescriptionSource(path),
                    launch_arguments=dict(
                        self.parse_launch_arguments(launch_file_arguments)
                    ).items(),
                ),
            ]
        )
        self.get_logger().info("...3")
        self.launch_service.include_launch_description(launch_description)
        self.get_logger().info("...4")
        await self.launch_service.run_async()

    async def run_slam(self):
        self.get_logger().debug("\n啟動建圖服務")
        robot_type = os.getenv("ROBOT_TYPE", "lino")
        await self.clean_up()  # Clean up any existing launched tasks

        log_file_path = "/tmp/master_service_slam_log.txt"

        def to_file(event):
            with open(log_file_path, "a") as log_file:
                log_file.write(event.text.decode())

        launch_file_name = "slam.launch.py"
        if robot_type == "artic":
            path = get_share_file_path_from_package(
                package_name="articubot_one", file_name=launch_file_name
            )
        elif robot_type == "lino":
            path = get_share_file_path_from_package(
                package_name="linorobot2_navigation", file_name=launch_file_name
            )
        launch_file_arguments = ["__log_level:=error"]
        self.get_logger().info("...2")
        self.launch_service = launch.LaunchService(debug=False)
        launch_description = launch.LaunchDescription(
            [
                RegisterEventHandler(
                    OnProcessIO(
                        on_stdout=to_file,
                        on_stderr=to_file,
                    )
                ),
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.AnyLaunchDescriptionSource(path),
                    launch_arguments=dict(
                        self.parse_launch_arguments(launch_file_arguments)
                    ).items(),
                ),
            ]
        )
        self.get_logger().info("...3")
        self.launch_service.include_launch_description(launch_description)
        self.get_logger().info("...4")
        await self.launch_service.run_async()


def main(args=None):
    rclpy.init(args=args)
    master_service = MasterAsyncService()
    master_service.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
