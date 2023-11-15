import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rcl_interfaces.srv import SetParameters
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from common.utils import get_start_and_end_stations
from fitrobot_interfaces.srv import WaypointFollower, TargetStation, Master, CancelNav
from fitrobot_interfaces.msg import Station, RobotStatus
from script.robot_navigator import BasicNavigator, TaskResult


class WaypointFollowerService(Node):
    def __init__(self):
        super().__init__('waypoint_follower_service')
        self.robot_status = None
        self.target_station = None
        self.start_station = None
        self.end_station = None

        self.get_logger().info("waypoint follower服務初始化")
        self.waypoint_srv = self.create_service(
            WaypointFollower,
            "waypoint_follower",
            self.waypoint_follower_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.target_station_srv = self.create_service(
            TargetStation,
            "target_station",
            self.target_station_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.cancel_nav_srv = self.create_service(
            CancelNav,
            "cancel_nav",
            self.cancel_nav_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.cli = self.wait_for_service(
            "/check_robot_status_node/set_parameters", SetParameters
        )
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.station_pub = self.create_publisher(
            Station,
            "target_station",
            qos,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.navigator = BasicNavigator()
        self.wait_for_service("/master", Master)
        self.start_station, self.end_station = get_start_and_end_stations()

        self.navigator.waitUntilNav2Active()

    def waypoint_follower_callback(self, request, response):
        self.get_logger().info(f'waypoint follower服務開始')
        station = request.station
        self.add_station(station)
        self.get_logger().info(f'waypoint follower服務結束')

        return response

    def target_station_callback(self, request, response):
        self.get_logger().info("target station服務開始")
        response.target_station = self.target_station or Station()
        self.get_logger().info("target station服務結束")
        return response

    def cancel_nav_callback(self, request, response):
        self.get_logger().info("cancel nav服務開始")
        self.navigator.cancelTask()
        response.ack = "SUCCESS"
        self.get_logger().info("cancel nav服務結束")
        return response

    def convert_station_to_pose(self, station: Station) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = station.x
        pose.pose.position.y = station.y
        pose.pose.orientation.z = station.z
        pose.pose.orientation.w = station.w

        return pose

    def add_station(self, station: Station):
        goal_stations = [station, self.end_station, self.start_station]
        goal_poses = list(map(self.convert_station_to_pose, goal_stations))
        self.navigator.followWaypoints(goal_poses)
        self.target_station = station

        self.get_logger().info(
            f"開始執行[{self.target_station.name}]"
            f"(x:{self.target_station.x}," 
            f"y:{self.target_station.y})運送任務"
        )

        i = 0
        current_status = None
        while not self.navigator.isTaskComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                if current_status != feedback.current_waypoint:
                    self.target_station = goal_stations[feedback.current_waypoint]  # feedback.current_waypoint in [0,1,2]
                    self.get_logger().info(
                        f"前往[{self.target_station.name}]"
                        f"(x:{self.target_station.x},"
                        f"y:{self.target_station.y})"
                    )
                    self.station_pub.publish(self.target_station)
                    current_status = feedback.current_waypoint

        result = self.navigator.getResult()

        self.get_logger().info(
            # result.name in [SUCCEEDED, CANCELED, FAILED, UNKNOWN]
            f"站點[{goal_stations[feedback.current_waypoint].name}]任務狀態:{result.name}"
        ) 
        # if result == TaskResult.SUCCEEDED:
        #     self.get_logger().info(f"[{goal_stations[feedback.current_waypoint].name}]運送任務完成!")
        # elif result == TaskResult.CANCELED:
        #     self.get_logger().info(f"[{goal_stations[feedback.current_waypoint].name}]運送任務取消!")
        # elif result == TaskResult.FAILED:
        #     self.get_logger().info(f"[{goal_stations[feedback.current_waypoint].name}]運送任務失敗!")
        # else:
        #     self.get_logger().info(f"[{goal_stations[feedback.current_waypoint].name}]運送任務回傳狀態不合法!")

        return

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
                pass
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    def wait_for_service(self, srv_name, srv_type):
        cli = self.create_client(srv_type, srv_name)
        while not cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"服務{srv_name}尚未啟動，等待中...")

        return cli


def main():
    rclpy.init()
    waypoint_follower_service = WaypointFollowerService()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_follower_service)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    waypoint_follower_service.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
