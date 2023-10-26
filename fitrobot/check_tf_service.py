import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from std_srvs.srv import Trigger
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import tf2_py as tf2


class TfCheckNode(Node):
    def __init__(self):
        super().__init__("tf_check_node")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.is_localized = False
        self.create_service(Trigger, "is_localized", self.srv_localized_callback)

    def srv_localized_callback(self, request, response):
        response.success = self.is_localized
        response.message = f"is_localized: {self.is_localized}"
        return response

    def check_transform(self):
        try:
            if self.tf_buffer.can_transform(
                "odom", "map", Time(), Duration(seconds=0.5)
            ):
                self.get_logger().debug("Transform available!")
                self.is_localized = True
                self.tf_buffer.clear()
            else:
                self.get_logger().debug("Transform not available.")
                self.is_localized = False
        except (tf2.LookupException, tf2.ExtrapolationException) as ex:
            self.get_logger().error("Transform lookup failed: {0}".format(str(ex)))


def main(args=None):
    rclpy.init(args=args)
    tf2_listener_node = TfCheckNode()
    timer = tf2_listener_node.create_timer(1, tf2_listener_node.check_transform)
    try:
        # This will run continuously until the node is interrupted
        rclpy.spin(tf2_listener_node)
    except KeyboardInterrupt:
        pass  # Handle shutdown gracefully
    finally:
        tf2_listener_node.destroy_timer(timer)
        tf2_listener_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    """usage
    execute service node:
    ros2 run fitrobot check_tf_service

    call service:
    ros2 service call /is_localized std_srvs/srv/Trigger

    example response:
    std_srvs.srv.Trigger_Response(success=False, message='is_localized: False')

    """
    main()
