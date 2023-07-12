from subprocess import PIPE, Popen

import rclpy
from rclpy.node import Node
from common.utils import get_logger


class BridgeNode(Node):
    def __init__(self):
        super().__init__('bridge_node')
        self.logger = get_logger('rosbridge')
        self.logger.debug(f'啟動bridge_node')
        p = Popen(["python3", "../scripts/bridge.py"], stdout=PIPE, stderr=PIPE)
        while p.poll() is None:
            o = p.stdout.readline()
            print(o.decode('utf-8'))
    

def main():
    rclpy.init()
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()