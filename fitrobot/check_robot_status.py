from enum import Enum
import subprocess
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_py as tf2


class RobotStatus(Enum):
    standby = 0
    bringup = 1
    slam = 2
    nav_prepare = 3
    nav_standby = 4
    #
    nav_running = 11
    nav_arrived = 12
    nav_cancel = 13
    nav_failed = 14
    #
    nav_wf_running = 21
    nav_wf_arrived = 22
    nav_wf_completed = 23
    nav_wf_cancel = 24
    nav_wf_failed = 25


class TfCheckNode(Node):
    def __init__(self):
        super().__init__("check_robot_status_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_status = RobotStatus.standby
        self.timer = self.create_timer(1.0, self.status_check)
        self.pub = self.create_publisher(Int32, "robot_status", 10)
        self.get_logger().info("launch: standby")

    def check_tf(self, parent, child, timeout=0.5):
        can_trasform = self.tf_buffer.can_transform(
            parent, child, Time(), Duration(seconds=timeout)
        )
        self.tf_buffer.clear()
        return True if can_trasform else False

    def is_tf_odom_baselink_existed(self):
        return self.check_tf("odom", "base_link", 0.9)

    def is_tf_odom_map_existed(self):
        return self.check_tf("map", "odom", 0.1)

    def check_nav2_running(self):
        nodenames = self.get_node_names()
        return True if 'bt_navigator' in nodenames else False

    def status_check(self):
        try:
            if self.robot_status == RobotStatus.standby:
                if self.is_tf_odom_baselink_existed():
                    self.get_logger().info("bringup")
                    self.robot_status = RobotStatus.bringup
                return

            elif self.robot_status == RobotStatus.bringup:
                if self.check_nav2_running():
                    self.get_logger().info("nav_prepare")
                    self.robot_status = RobotStatus.nav_prepare
                elif not self.is_tf_odom_baselink_existed():
                    self.get_logger().info("standby")
                    self.robot_status = RobotStatus.standby
                return

            if self.robot_status == RobotStatus.nav_prepare:
                if self.is_tf_odom_map_existed():
                    self.get_logger().info("nav_standby")
                    self.robot_status = RobotStatus.nav_standby
                elif not self.check_nav2_running():
                    self.get_logger().info("bringup")
                    self.robot_status = RobotStatus.bringup
                return

            elif self.robot_status == RobotStatus.nav_standby:
                if not self.is_tf_odom_map_existed():
                    self.get_logger().info("nav_prepare")
                    self.robot_status = RobotStatus.nav_prepare
                return

            # elif self.robot_status == RobotStatus.bringup:
            #     pass

        except (tf2.LookupException, tf2.ExtrapolationException) as ex:
            self.get_logger().error("Transform lookup failed: {0}".format(str(ex)))


def main(args=None):
    rclpy.init(args=args)
    tf2_listener_node = TfCheckNode()
    try:
        rclpy.spin(tf2_listener_node)
    except KeyboardInterrupt:
        pass  # Handle shutdown gracefully
    finally:
        tf2_listener_node.destroy_timer(tf2_listener_node.timer)
        tf2_listener_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    """usage
    execute service node:
    ros2 run fitrobot check_robot_status

    # subsribe topic
    # ros2 topic echo /robot_status

    # example response:
    # std_srvs.srv.Trigger_Response(success=False, message='is_localized: False')

    """
    main()
