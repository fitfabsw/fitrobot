from enum import Enum
import subprocess
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import Trigger
from std_msgs.msg import Int32
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fitrobot_interfaces.msg import RobotStatus
import tf2_py as tf2


class TfCheckNode(Node):
    def __init__(self):
        super().__init__("check_robot_status_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robot_status = RobotStatus.STANDBY
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.timer = self.create_timer(1.0, self.status_check)
        self.pub = self.create_publisher(RobotStatus, "robot_status", qos)
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
        return True if "bt_navigator" in nodenames else False

    def status_check(self):
        try:
            if self.robot_status == RobotStatus.STANDBY:
                if self.is_tf_odom_baselink_existed():
                    self.get_logger().info("bringup")
                    self.robot_status = RobotStatus.BRINGUP
                    self.pub.publish(RobotStatus(status=RobotStatus.BRINGUP))
                return

            elif self.robot_status == RobotStatus.BRINGUP:
                if self.check_nav2_running():
                    self.get_logger().info("nav_prepare")
                    self.robot_status = RobotStatus.NAV_PREPARE
                    self.pub.publish(RobotStatus(status=RobotStatus.NAV_PREPARE))
                elif not self.is_tf_odom_baselink_existed():
                    self.get_logger().info("standby")
                    self.robot_status = RobotStatus.STANDBY
                    self.pub.publish(RobotStatus(status=RobotStatus.STANDBY))
                return

            if self.robot_status == RobotStatus.NAV_PREPARE:
                if self.is_tf_odom_map_existed():
                    self.get_logger().info("nav_standby")
                    self.robot_status = RobotStatus.NAV_STANDBY
                    self.pub.publish(RobotStatus(status=RobotStatus.NAV_STANDBY))
                elif not self.check_nav2_running():
                    self.get_logger().info("bringup")
                    self.robot_status = RobotStatus.BRINGUP
                    self.pub.publish(RobotStatus(status=RobotStatus.BRINGUP))
                return

            elif self.robot_status == RobotStatus.NAV_STANDBY:
                if not self.is_tf_odom_map_existed():
                    self.get_logger().info("nav_prepare")
                    self.robot_status = RobotStatus.NAV_PREPARE
                    self.pub.publish(RobotStatus(status=RobotStatus.NAV_PREPARE))
                return

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
    # execute node
    ros2 run fitrobot check_robot_status
    [INFO] [1698830984.888322671] [check_robot_status_node]: launch: standby
    [INFO] [1698830994.859154921] [check_robot_status_node]: bringup
    [INFO] [1698831006.859219936] [check_robot_status_node]: nav_prepare
    [INFO] [1698831013.862822112] [check_robot_status_node]: nav_standby

    # subsribe topic
    # ros2 topic echo /robot_status
    status: 1
    ---
    status: 3
    ---
    status: 4
    ---
    """
    main()
