import os
from pathlib import Path
from fitrobot_interfaces.srv import ListMap
from fitrobot_interfaces.msg import MapListItem
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from fitrobot_interfaces.srv import WaypointFollower
import script.waypoint_follower_demo as wpf


class WaypointFollowerService(Node):

    def __init__(self):
        super().__init__('waypoint_follower_service')
        self.get_logger().info(f'服務初始化')
        self.srv = self.create_service(WaypointFollower, 'waypoint_follower', self.waypoint_follower_callback)

    def waypoint_follower_callback(self, request, response):
        self.get_logger().info(f'服務開始')

        points = request.point_list
        for point in points:
            self.get_logger().info(f'{point}')
        wpf.main()
        # response.map_list = self.list_files_in_folder(MAP_FOLDER)
        
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