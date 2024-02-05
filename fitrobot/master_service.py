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
        robot_type = os.getenv('ROBOT_TYPE', 'lino')
        if robot_type == 'lino':
            self.declare_parameter("active_nav_map", "lino2_office_20240129.yaml")
        elif robot_type == 'artic':
            self.declare_parameter("active_nav_map", "office_res002_0523.yaml")

        self.process_list = []


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

        robot_type = os.getenv('ROBOT_TYPE', 'lino')
        if robot_type == 'lino':
            # p = Popen(["ros2", "launch", "linorobot2_navigation", "navigation.launch.py", map_path], stdout=PIPE, stderr=PIPE)
            # p = Popen(f"ros2 launch linorobot2_navigation navigation.launch.py {map_path} 2>&1 | tee /tmp/master_service_nav_log.txt", shell=True)
            p = Popen(f"ros2 launch linorobot2_navigation navigation_keepout.launch.py {map_path} 2>&1 | tee /tmp/master_service_nav_log.txt", shell=True)
        elif robot_type == 'artic':
            # p = Popen(["ros2", "launch", "articubot_one", "navigation.launch.py", map_path], stdout=PIPE, stderr=PIPE)
            p = Popen(f"ros2 launch articubot_one navigation_keepout.launch.py {map_path} 2>&1 | tee /tmp/master_service_nav_log.txt", shell=True)

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


def main():
    # signal.signal(signal.SIGINT, signal_handler)
    rclpy.init()
    master_service = MasterService()
    rclpy.spin(master_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
