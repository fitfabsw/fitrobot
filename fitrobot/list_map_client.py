import sys

from fitrobot_interfaces.srv import ListMap
import rclpy
from rclpy.node import Node


class ListMapClientAsync(Node):

    def __init__(self):
        super().__init__('list_map_client_async')
        self.cli = self.create_client(ListMap, 'list_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('地圖列表的service尚未啟動，等待中...')
        self.req = ListMap.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    list_map_client = ListMapClientAsync()
    response = list_map_client.send_request()
    # TODO: 把下面的None換成地圖縮圖
    print(list(map(lambda mli: (mli.map_name, None), response.map_list)))

    list_map_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()