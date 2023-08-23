import os
from pathlib import Path
from fitrobot_interfaces.srv import ListMap
from fitrobot_interfaces.msg import MapListItem
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from fitrobot_interfaces.srv import NavThroughPoses
import scripts.nav_through_poses_demo


class NavThroughPosesService(Node):

    def __init__(self):
        super().__init__('nav_through_poses_service')
        self.srv = self.create_service(NavThroughPoses, 'nav_through_poses', self.nav_through_poses_callback)

    def nav_through_poses_callback(self, request, response):
        self.get_logger().info(f'服務開始')
        # response.map_list = self.list_files_in_folder(MAP_FOLDER)
        self.get_logger().info(f'服務結束')
            
        return response

def main():
    rclpy.init()
    nav_through_poses_service = NavThroughPosesService()
    rclpy.spin(nav_through_poses_service)
    nav_through_poses_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()