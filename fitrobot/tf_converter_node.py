import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped


class CustomTFListener(Node):

    def __init__(self):
        super().__init__('custom_tf_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/custom_pose', 10)

        self.timer = self.create_timer(0.5, self.timer_callback)  # Check every 500ms

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


def main(args=None):
    rclpy.init(args=args)
    custom_tf_listener = CustomTFListener()
    rclpy.spin(custom_tf_listener)
    custom_tf_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
