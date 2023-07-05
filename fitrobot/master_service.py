import rclpy
from fitrobot_interfaces.srv import Master
from rclpy.node import Node


class MasterService(Node):

    def __init__(self):
        super().__init__('master_service')
        self.srv = self.create_service(Master, 'master', self.master_callback)

    def master_callback(self, request, response):
        
        return response
    
    def clean_up(self):
        pass

    def run_navigation(self):
        self.clean_up()
        Node()
        pass

    def run_slam(self):
        self.clean_up()
        pass


def main():
    rclpy.init()
    master_service = MasterService()
    rclpy.spin(master_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()