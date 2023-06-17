from fitrobot_interfaces.srv import SaveMap

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(SaveMap, 'save_map', self.save_map_callback)

    def save_map_callback(self, request, response):
        self.get_logger().info('儲存地圖到 %s' % (request.map_name))
        response.ack = request.map_name

        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()