import random
import json

import rclpy
from rclpy.node import Node
from flask import Flask, request
from flask_socketio import send, emit, Namespace, SocketIO
from common.utils import get_logger


class ServerNode(Node):
    def __init__(self):
        super().__init__('server_node')
        
        self.logger = get_logger('rosbridge')
        self.logger.debug(f'啟動server_node')
        app = Flask(__name__)
        app.config['SECRETE_KEY'] = 'secret!'
        self.socketio = SocketIO(app, cors_allowed_origins='*', ping_timeout=25, max_http_buffer_size=500000000, ping_interval=7200, always_connect=True)
        self.socketio.on_namespace(RosBridgeNamespace('/rosbridge'))
        self.socketio.run(app, host='0.0.0.0', port='5556', allow_unsafe_werkzeug=True)


class RosBridgeNamespace(Namespace):
    def __init__(self, namespace=None):
        super().__init__(namespace)
        self.logger = get_logger('rosbridge')
        
    def on_connect(self):
        self.logger.debug(f'ROS裝置已連線, sid={request.sid}')

    def on_disconnect(self):
        self.logger.debug(f'ROS裝置已斷線, sid={request.sid}')

    def on_bridge(self, data):#收到rosbridge傳來的訊息
        # 將訊息轉拋給client
        self.logger.debug('收到樹莓派傳送的訊息，轉發給手機')
        emit('data', json.dumps(data), namespace='/rosbridge', broadcast=True)

    def on_client(self,data):#收到client傳來的訊息
        # 將訊息轉拋給rosbridge
        self.logger.debug('收到手機傳送的訊息，轉發給樹莓派')
        emit('server', data, namespace='/rosbridge', broadcast=True)
    

def main():
    rclpy.init()
    node = ServerNode()
    # rclpy.spin(node)  # ServerNode跑socketio.run之後本身就blocking，所以不確定需不需要使用spin，使用的話會變成雙重blocking
    # node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()