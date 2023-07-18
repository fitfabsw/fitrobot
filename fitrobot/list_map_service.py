import os
from pathlib import Path
from fitrobot_interfaces.srv import ListMap
from fitrobot_interfaces.msg import MapListItem
import rclpy
from rclpy.node import Node


MAP_FOLDER = Path.home().joinpath("fitrobot_map")

class ListMapService(Node):

    def __init__(self):
        super().__init__('list_map_service')
        self.srv = self.create_service(ListMap, 'list_map', self.list_map_callback)
        MAP_FOLDER.mkdir(parents=True, exist_ok=True)

    def list_map_callback(self, request, response):
        self.get_logger().info(f'列出{MAP_FOLDER}下的地圖')
        response.map_list = self.list_files_in_folder(MAP_FOLDER)
        self.get_logger().info(f'地圖列表結果: {response.map_list}')
        
        return response

    def list_files_in_folder(self, folder_path):
        file_names = []
        for file_name in os.listdir(folder_path):
            if file_name.endswith('.yaml') and os.path.isfile(os.path.join(folder_path, file_name)):
                mli = MapListItem()
                mli.map_name = file_name
                file_names.append(mli)
        return file_names

def main():
    rclpy.init()
    list_map_service = ListMapService()
    rclpy.spin(list_map_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()