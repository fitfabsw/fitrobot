import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from fitrobot_interfaces.srv import WaypointFollower
from fitrobot_interfaces.msg import Station
from script.waypoint_manager import WaypointManager


class WaypointFollowerService(Node):

    def __init__(self):
        super().__init__('waypoint_follower_service')
        self.get_logger().info(f'服務初始化')
        self.srv = self.create_service(WaypointFollower, 'waypoint_follower', self.waypoint_follower_callback, callback_group=MutuallyExclusiveCallbackGroup())
        self.pub = self.create_publisher(Station, "current_station", 10, callback_group=MutuallyExclusiveCallbackGroup())
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.wpMgr = WaypointManager()

    def waypoint_follower_callback(self, request, response):
        self.get_logger().info(f'服務開始')

        station = request.station
        self.wpMgr.add_station(station)
        
        self.get_logger().info(f'服務結束')

        return response
    
    def timer_callback(self):
        if self.wpMgr and self.wpMgr.current_station:
            self.pub.publish(self.wpMgr.current_station)

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