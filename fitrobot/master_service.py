import os
import sys
import rclpy
import signal
import psutil
from ament_index_python.packages import get_package_share_directory
from subprocess import Popen, PIPE
from fitrobot_interfaces.srv import Master
from rclpy.node import Node
from rclpy.parameter import Parameter


class MasterService(Node):

    def __init__(self):
        super().__init__('master_service')
        self.srv = self.create_service(Master, 'master', self.master_callback)
        self.declare_parameter("active_nav_map", "office_res002_0914.yaml")
        self.process_list = []

        # Popen(["ros2", "run", "fitrobot", "save_map_service"], stdout=PIPE, stderr=PIPE)
        # Popen(["ros2", "run", "fitrobot", "list_map_service"], stdout=PIPE, stderr=PIPE)
        # Popen(["ros2", "run", "fitrobot", "waypoint_follower"], stdout=PIPE, stderr=PIPE)

    def master_callback(self, request, response):
        request_action = request.request_action
        request_param = request.request_param
        if request_action == "slam":
            self.run_slam()
        elif request_action == "navigation":
            self.run_navigation(map_name=request_param)
        else:
            pass

        return response

    def clean_up(self):
        self.get_logger().debug("清理已開啟服務")
        while self.process_list:
            p = self.process_list.pop()
            self.get_logger().debug(f"清理process:{p.pid}")
            self.kill_process_and_children(p.pid)

    def run_navigation(self, map_name):
        self.get_logger().debug("\n啟動導航服務")
        maploc = os.path.join(get_package_share_directory("fitrobot"), 'maps')
        self.clean_up()
        map_name_param = Parameter('active_nav_map', Parameter.Type.STRING, map_name)
        self.set_parameters([map_name_param])
        map_path = f"map:={maploc}/{map_name}"
        p = Popen(["ros2", "launch", "linorobot2_navigation", "navigation.launch.py", map_path], stdout=PIPE, stderr=PIPE)
        self.process_list.append(p)

    def run_slam(self):
        self.get_logger().debug("\n啟動建圖服務")
        self.clean_up()
        p = Popen(["ros2", "launch", "fitrobot", "slam.launch.py"], stdout=PIPE, stderr=PIPE)
        self.process_list.append(p)
        p = Popen(["ros2", "run", "usb_cam", "usb_cam_node_exe"], stdout=PIPE, stderr=PIPE)
        self.process_list.append(p)


    def kill_process_and_children(self, pid):
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)

        for child in children:
            child.kill()

        parent.kill()

# def signal_handler(signum, frame):
#     print('signal_handler: caught signal ' + str(signum))
#     if signum == signal.SIGINT.value:
#         print('SIGINT')
#         sys.exit(1)

def main():
    # signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()
    master_service = MasterService()
    rclpy.spin(master_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
