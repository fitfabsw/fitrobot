import os
import asyncio
from typing import List
from typing import Text
from typing import Tuple
from collections import OrderedDict
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessIO
from ros2launch.api import get_share_file_path_from_package
from fitrobot_interfaces.srv import Master, TerminateProcess
from fitrobot_interfaces.msg import RobotStatus


class MasterAsyncService(Node):
    def __init__(self):
        super().__init__("master_service")
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.status_pub = self.create_publisher(
            RobotStatus,
            "robot_status",
            qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.srv = self.create_service(Master, "master", self.master_callback)
        self.terminate_service = self.create_service(
            TerminateProcess,
            "terminate_slam_or_navigation",
            self.terminate_slam_or_navigation_callback,
        )
        self.robot_type = os.getenv("ROBOT_TYPE", "lino")
        self.use_sim = bool(os.getenv("USE_SIM", 0))

        if self.use_sim:  # for simualtions
            self.declare_parameter("active_nav_map", "turtlebot3_world.yaml")
        else:  # for real robots
            if self.robot_type == "lino":
                self.declare_parameter("active_nav_map", "lino2_office_20240129.yaml")
            elif self.robot_type == "artic":
                self.declare_parameter("active_nav_map", "office_res002_0523.yaml")

        self.launch_service = None
        self.event_loop = asyncio.get_event_loop()

        self.get_cli = self.wait_for_service(
            "/check_robot_status_node/get_parameters", GetParameters
        )
        self.cli = self.wait_for_service(
            "/check_robot_status_node/set_parameters", SetParameters
        )

    def send_set_parameters_request(self, param_value):
        param_name = "fitrobot_status"
        val = ParameterValue(
            integer_value=param_value, type=ParameterType.PARAMETER_INTEGER
        )
        req = SetParameters.Request(parameters=[Parameter(name=param_name, value=val)])
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_future_done)

    def on_future_done(self, future):
        try:
            response = future.result()
            if response.results[0].successful:
                self.get_logger().info("Service call successful")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    async def send_get_parameters_request(self):
        req = GetParameters.Request()
        req.names = ["fitrobot_status"]
        future = self.get_cli.call_async(req)
        response = await future
        robot_status = response.values[0].integer_value
        return robot_status

    def wait_for_service(self, srv_name, srv_type):
        cli = self.create_client(srv_type, srv_name)
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"服務{srv_name}尚未啟動，等待中...")
        return cli

    async def master_callback(self, request, response):
        self.get_logger().info("master_callback start")
        request_action = request.request_action
        request_param = request.request_param
        # Launch the slam/navigation in an asynchronous manner
        if request_action == "slam":
            asyncio.ensure_future(self.run_slam())
            asyncio.ensure_future(self.ensure_robotstatus_slam())
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
            # await asyncio.sleep(0.01)  # no need for high frequency. also help reduce CPU usage
            await asyncio.sleep(1)

    def terminate_slam_or_navigation_callback(self, request, response):
        self.get_logger().info("Terminating navigation...")
        asyncio.ensure_future(self.clean_up())
        response.success = True
        self.get_logger().info("end")
        return response

    async def ensure_robotstatus_bringup(self):
        while True:
            robot_status = await self.send_get_parameters_request()
            if robot_status == 1:
                break
            await asyncio.sleep(1)

    async def ensure_robotstatus_slam(self):
        while "slam_toolbox" not in self.get_node_names():
            await asyncio.sleep(
                0.5
            )  # sleep for a short duration to prevent a busy loop
        self.status_pub.publish(RobotStatus(status=RobotStatus.SLAM))
        self.send_set_parameters_request(RobotStatus.SLAM)

    async def clean_up(self):
        if self.launch_service:
            self.get_logger().info("clean_up: ready to shutdown.")
            await self.launch_service.shutdown()
            await self.ensure_robotstatus_bringup()
            self.get_logger().info("clean_up: done. robot_status back to BRINGUP")
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

        if self.use_sim:  # for simualtions
            maploc = os.path.join(
                get_package_share_directory("linorobot2_navigation"), "maps"
            )
            maskloc = os.path.join(
                get_package_share_directory("linorobot2_navigation"), "masks"
            )
        else:  # for real robots
            maploc = os.path.join(get_package_share_directory("fitrobot"), "maps")
            maskloc = os.path.join(get_package_share_directory("fitrobot"), "masks")

        map_path = f"map:={maploc}/{map_name}"
        mask_path = f"mask:={maskloc}/keepout_mask_{map_name}"
        self.get_logger().info(f"map_path: {map_path}")
        self.get_logger().info(f"mask_path: {mask_path}")

        launch_file_name = "navigation.launch.py"
        if self.robot_type == "artic":
            launch_file_name = "navigation_keepout.launch.py"
            path = get_share_file_path_from_package(
                package_name="articubot_one", file_name=launch_file_name
            )
        elif self.robot_type == "lino":
            # launch_file_name = "navigation_keepout.launch.py"
            launch_file_name = "navigation.launch.py"
            path = get_share_file_path_from_package(
                package_name="linorobot2_navigation", file_name=launch_file_name
            )
        launch_file_arguments = [map_path, mask_path, "__log_level:=error"]
        self.launch_service = launch.LaunchService(debug=False)
        launch_description = launch.LaunchDescription([
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
        ])
        self.get_logger().info("...3")
        self.launch_service.include_launch_description(launch_description)
        self.get_logger().info("...4")
        await self.launch_service.run_async()

    async def run_slam(self):
        self.get_logger().debug("\n啟動建圖服務")
        await self.clean_up()  # Clean up any existing launched tasks

        log_file_path = "/tmp/master_service_slam_log.txt"

        def to_file(event):
            with open(log_file_path, "a") as log_file:
                log_file.write(event.text.decode())

        launch_file_name = "slam.launch.py"
        if self.robot_type == "artic":
            path = get_share_file_path_from_package(
                package_name="articubot_one", file_name=launch_file_name
            )
        elif self.robot_type == "lino":
            path = get_share_file_path_from_package(
                package_name="linorobot2_navigation", file_name=launch_file_name
            )
        launch_file_arguments = ["__log_level:=error"]
        self.get_logger().info("...2")
        self.launch_service = launch.LaunchService(debug=False)
        launch_description = launch.LaunchDescription([
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
        ])
        self.get_logger().info("...3")
        self.launch_service.include_launch_description(launch_description)
        self.get_logger().info("...4")

        # self.status_pub.publish(RobotStatus(status=RobotStatus.SLAM))
        # self.send_set_parameters_request(RobotStatus.SLAM)

        await self.launch_service.run_async()


def main(args=None):
    rclpy.init(args=args)
    master_service = MasterAsyncService()
    master_service.run()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
