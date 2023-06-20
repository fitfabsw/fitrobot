import sys

from fitrobot_interfaces.srv import SaveMap
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SaveMap, 'save_map')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('儲存地圖的service尚未啟動，等待中...')
        self.req = SaveMap.Request()

    def send_request(self, map_name):
        self.req.map_name = map_name
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(sys.argv[1])
    print(f'{response.ack}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()