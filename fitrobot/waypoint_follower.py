import os
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from fitrobot_interfaces.srv import WaypointFollower
from script.waypoint_follower_demo import WaypointManager


class WaypointFollowerService(Node):

    def __init__(self):
        super().__init__('waypoint_follower_service')
        self.get_logger().info(f'服務初始化')
        self.srv = self.create_service(WaypointFollower, 'waypoint_follower', self.waypoint_follower_callback)
        self.wpMgr = WaypointManager()

    def waypoint_follower_callback(self, request, response):
        self.get_logger().info(f'服務開始')

        point_list = request.point_list
        self.wpMgr.add_points(point_list)
        
        self.get_logger().info(f'服務結束')

        return response

def main():
    rclpy.init()
    waypoint_follower_service = WaypointFollowerService()
    rclpy.spin(waypoint_follower_service)
    waypoint_follower_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()