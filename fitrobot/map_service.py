import os
from pathlib import Path
from fitrobot_interfaces.srv import SaveMap, ListMap
from fitrobot_interfaces.msg import MapListItem
import rclpy
from rclpy.node import Node

MAP_FOLDER = Path.home().joinpath("fitrobot_map")


class MapService(Node):
    def __init__(self):
        super().__init__("map_service")

        # 為保存地图创建服务
        self.save_map_srv = self.create_service(
            SaveMap, "save_map", self.save_map_callback
        )

        # 为列出地图创建服务
        self.list_map_srv = self.create_service(
            ListMap, "list_map", self.list_map_callback
        )

        # 确保地图文件夹存在
        MAP_FOLDER.mkdir(parents=True, exist_ok=True)

    def save_map_callback(self, request, response):
        self.get_logger().info(f"正在保存地图到 {request.map_name}")
        map_path = MAP_FOLDER.joinpath(request.map_name)
        r1 = os.popen(f"ros2 run nav2_map_server map_saver_cli -f {map_path}")
        r1.read()
        r2 = os.popen(f"convert {map_path}.pgm -resize 100x100! {map_path}.jpg")
        r2.read()
        response.ack = "SUCCESS" if (not r1.close() and not r2.close()) else "FAIL"
        self.get_logger().info(f"保存地图结果: {response.ack}")
        return response

    def list_map_callback(self, request, response):
        self.get_logger().info(f"正在列出 {MAP_FOLDER} 下的地图")
        response.map_list = self.list_files_in_folder(MAP_FOLDER)
        self.get_logger().info(f"地图列表结果: {response.map_list}")
        return response

    def list_files_in_folder(self, folder_path):
        file_names = []
        for file_name in os.listdir(folder_path):
            if file_name.endswith(".yaml") and os.path.isfile(
                os.path.join(folder_path, file_name)
            ):
                mli = MapListItem()
                mli.map_name = file_name
                file_names.append(mli)
        return file_names


def main():
    rclpy.init()
    map_service = MapService()
    rclpy.spin(map_service)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
