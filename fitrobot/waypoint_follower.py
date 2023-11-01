import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped
from common.utils import get_start_and_end_stations
from fitrobot_interfaces.srv import WaypointFollower, TargetStation
from fitrobot_interfaces.msg import Station, RobotStatus
from script.robot_navigator import BasicNavigator, NavigationResult


class WaypointFollowerService(Node):

    def __init__(self):
        super().__init__('waypoint_follower_service')
        self.robot_status = None
        self.target_station = None
        self.start_station = None
        self.end_station = None

        self.get_logger().info(f'waypoint follower服務初始化')
        self.waypoint_srv = self.create_service(WaypointFollower, "waypoint_follower", self.waypoint_follower_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.target_station_srv = self.create_service(TargetStation, "target_station", self.target_station_callback, callback_group=MutuallyExclusiveCallbackGroup())

        qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.status_pub = self.create_publisher(RobotStatus, "robot_status", qos, callback_group=MutuallyExclusiveCallbackGroup())
        self.sub = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.status_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.navigator = BasicNavigator()
        self.start_station, self.end_station = get_start_and_end_stations()

    def waypoint_follower_callback(self, request, response):
        self.get_logger().info(f'waypoint follower服務開始')

        station = request.station
        self.add_station(station)
        
        self.get_logger().info(f'waypoint follower服務結束')

        return response
    
    def target_station_callback(self, request, response):
        self.get_logger().info(f'target station服務開始')

        response.target_station = self.target_station
        
        self.get_logger().info(f'target station服務結束')
        return response
    
    def status_callback(self, msg):
        status = msg.status_list[-1].status
        if status == 2:
            self.status_pub.publish(RobotStatus(status=RobotStatus.NAV_WF_RUNNING))
        elif status == 4:
            self.status_pub.publish(RobotStatus(status=RobotStatus.NAV_WF_ARRIVED))
    
    def convert_station_to_pose(self, station: Station) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
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
        
        print(f"開始執行站點 (name:{self.target_station.name}, x:{self.target_station.x}, y:{self.target_station.y}) 運送任務")

        i = 0
        current_status = None
        while not self.navigator.isNavComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                # if current_status != self.target_station:
                if current_status != feedback.current_waypoint:
                    if feedback.current_waypoint == 0:
                        self.target_station = station
                        print(f"前往站點 (name:{self.target_station.name}, x:{self.target_station.x}, y:{self.target_station.y}) 運送任務")
                    elif feedback.current_waypoint == 1:
                        self.target_station = self.end_station
                        print(f"運送至FA Room")
                    elif feedback.current_waypoint == 2:
                        self.target_station = self.start_station
                        print(f"返回充電座")
                    current_status = feedback.current_waypoint

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('運送任務完成!')
            self.status_pub.publish(RobotStatus(status=RobotStatus.NAV_WF_COMPLETED))
        elif result == NavigationResult.CANCELED:
            print('運送任務取消!')
            self.status_pub.publish(RobotStatus(status=RobotStatus.NAV_WF_CANCEL))
        elif result == NavigationResult.FAILED:
            print('運送任務失敗!')
            self.status_pub.publish(RobotStatus(status=RobotStatus.NAV_WF_FAILED))
        else:
            print('運送任務回傳狀態不合法!')

        # self.navigator.lifecycleShutdown()

        return

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


if __name__ == '__main__':
    main()
