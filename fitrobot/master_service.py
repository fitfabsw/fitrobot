import sys
import rclpy
import signal
import psutil
from subprocess import Popen, PIPE
from fitrobot_interfaces.srv import Master
from rclpy.node import Node


class MasterService(Node):

    def __init__(self):
        super().__init__('master_service')
        self.srv = self.create_service(Master, 'master', self.master_callback)
        self.process_list = []
        p = Popen(["ros2", "launch", "fitrobot", "articubot.launch.py"], stdout=PIPE, stderr=PIPE)

    def master_callback(self, request, response):
        request_action = request.request_action
        if request_action == "slam":
            self.run_slam()
        elif request_action == "navigation":
            self.run_navigation()
        else:
            pass

        return response
    
    def clean_up(self):
        print("清理已開啟服務")
        while self.process_list:
            p = self.process_list.pop()
            print(f"清理process:{p.pid}")
            self.kill_process_and_children(p.pid)
            
    def run_navigation(self):
        print("\n啟動導航服務")
        self.clean_up()
        p = Popen(["ros2", "launch", "fitrobot", "navigation.launch.py"], stdout=PIPE, stderr=PIPE)
        self.process_list.append(p)

    def run_slam(self):
        print("\n啟動建圖服務")
        self.clean_up()
        p = Popen(["ros2", "launch", "fitrobot", "slam.launch.py"], stdout=PIPE, stderr=PIPE)
        self.process_list.append(p)


    def kill_process_and_children(self, pid):
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)

        for child in children:
            child.kill()

        parent.kill()

def signal_handler(signum, frame):
    print('signal_handler: caught signal ' + str(signum))
    if signum == signal.SIGINT.value:
        print('SIGINT')
        sys.exit(1)

def main():
    signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()
    master_service = MasterService()
    rclpy.spin(master_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()