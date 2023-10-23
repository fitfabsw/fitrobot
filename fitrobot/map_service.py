import os
from pathlib import Path
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from fitrobot_interfaces.srv import SaveMap, ListMap
from fitrobot_interfaces.msg import MapListItem
import rclpy
from rclpy.node import Node

MAP_FOLDER = Path.home().joinpath("fitrobot_map")


class MapService(Node):
    def __init__(self):
        super().__init__("map_service")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/custom_pose', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Check every 1 second

        self.master_srv = self.create_service(Master, 'master', self.master_callback)
        self.declare_parameter("active_nav_map", "office3_res0.02_0523.yaml")
        self.process_list = []
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
        self.clean_up()
        map_name_param = Parameter('active_nav_map', Parameter.Type.STRING, map_name)
        self.set_parameters([map_name_param])
        # map_path = f"map:=/home/pi/fitrobot_map/{map_name}"
        map_path = f"map:=/home/parallels/fitrobot_map/{map_name}"
        p = Popen(["ros2", "launch", "fitrobot", "navigation.launch.py", map_path], stdout=PIPE, stderr=PIPE)
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

    def timer_callback(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())

            # Now you have the transform from map to base_link.
            # You can use this transform in your application.

            # Let's create a PoseWithCovarianceStamped message from the transform
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.pose.position.x = transform.transform.translation.x
            pose_msg.pose.pose.position.y = transform.transform.translation.y
            pose_msg.pose.pose.position.z = transform.transform.translation.z
            pose_msg.pose.pose.orientation = transform.transform.rotation

            # Here, we leave the covariance as zeros, which means unknown.
            # You might want to fill it with your actual covariance data if you have it.

            # Now let's publish the pose
            self.pose_publisher.publish(pose_msg)
        except Exception as e:
            self.get_logger().warn('{}'.format(e))

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
    # custom_tf_listener.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
