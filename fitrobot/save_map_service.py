import os
from pathlib import Path
from fitrobot_interfaces.srv import SaveMap
import rclpy
from rclpy.node import Node


MAP_FOLDER = Path.home().joinpath("fitrobot_map")

class SaveMapService(Node):

    def __init__(self):
        super().__init__('save_map_service')
        self.srv = self.create_service(SaveMap, 'save_map', self.save_map_callback)
        MAP_FOLDER.mkdir(parents=True, exist_ok=True)

    def save_map_callback(self, request, response):
        self.get_logger().info('儲存地圖到 %s' % (request.map_name))
        map_path = MAP_FOLDER.joinpath(request.map_name)
        r1 = os.popen(f"ros2 run nav2_map_server map_saver_cli -f {map_path}")
        r1.read()
        r2 = os.popen(f"convert {map_path}.pgm -resize 100x100\! {map_path}.jpg")
        r2.read()
        response.ack = "SUCCESS" if (not r1.close() and not r2.close()) else "FAIL"
        self.get_logger().info(f'儲存地圖結果: {response.ack}')
        
        return response


def main():
    rclpy.init()
    save_map_service = SaveMapService()
    rclpy.spin(save_map_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()