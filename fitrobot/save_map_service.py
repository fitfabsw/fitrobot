import os
from pathlib import Path
from fitrobot_interfaces.srv import SaveMap
import rclpy
from rclpy.node import Node


MAP_FOLDER = Path.home().joinpath("fitrobot_map")

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SaveMap, 'save_map', self.save_map_callback)
        MAP_FOLDER.mkdir(parents=True, exist_ok=True)

    def save_map_callback(self, request, response):
        self.get_logger().info('儲存地圖到 %s' % (request.map_name))
        stream = os.popen(f"ros2 run nav2_map_server map_saver_cli -f {MAP_FOLDER.joinpath(request.map_name)}")
        stream.read()
        response.ack = "SUCCESS" if not stream.close() else "FAIL"
        self.get_logger().info(f'儲存地圖結果: {response.ack}')
        
        return response


def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()