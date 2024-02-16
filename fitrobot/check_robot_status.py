import subprocess
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from fitrobot_interfaces.msg import RobotStatus

from std_msgs.msg import Int32
from action_msgs.msg import GoalStatus, GoalStatusArray
import tf2_py as tf2
from rclpy.parameter import Parameter

arrive_music_path = "/home/pi/garbage.wav"


"""
This is based on below waypoint follower paramters

waypoint_follower:
  ros__parameters:
    use_sim_time: True
    loop_rate: 20
    stop_on_failure: true
    waypoint_task_executor_plugin: "input_at_waypoint"
    input_at_waypoint:
      plugin: "nav2_waypoint_follower::InputAtWaypoint"
      enabled: True
      timeout: 10.0
      input_topic: ok_to_go
"""

"""
GoalStatus
-------------------
0 STATUS_UNKNOWN
1 STATUS_ACCEPTED
2 STATUS_EXECUTING
3 STATUS_CANCELING
4 STATUS_SUCCEEDED
5 STATUS_CANCELED
6 STATUS_ABORTED
"""
"""
fitrobot_interfaces/msg/RobotStatus.msg
-----------------------------
int8 STANDBY = 0
int8 BRINGUP = 1
int8 SLAM = 2
int8 NAV_PREPARE = 3

int8 NAV_READY = 10
int8 NAV_RUNNING = 11
int8 NAV_ARRIVED = 12
int8 NAV_CANCEL = 13
int8 NAV_FAILED = 14

int8 NAV_WF_RUNNING = 21
int8 NAV_WF_ARRIVED = 22
int8 NAV_WF_COMPLETED = 23
int8 NAV_WF_CANCEL = 24
int8 NAV_WF_FAILED = 25

// #define LED_STATES_OFF           0x00
// #define LED_STATES_DRIVING       0x01
// #define LED_STATES_REVERSE       0x02
// #define LED_STATES_TURN_L        0x03
// #define LED_STATES_TURN_R        0x04
// #define LED_STATES_CAUTION_ZONE  0x05
// #define LED_STATES_STANDBY       0x06
// #define LED_STATES_FAULT         0x07
// #define LED_STATES_CHARGING      0x08
// #define LED_STATES_BATTERY_LOW   0x09
"""


class RobotStatusCheckNode(Node):
    def __init__(self):
        super().__init__("check_robot_status_node")
        self.declare_parameter("fitrobot_status", RobotStatus.STANDBY)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.waypoins_following = False
        qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.timer = self.create_timer(1.0, self.status_check)
        self.status_pub = self.create_publisher(RobotStatus, "robot_status", qos)
        self.led_pub = self.create_publisher(Int32, "led_status", 10)
        self.get_logger().info("launch: standby")
        self.is_localized = False
        self.create_service(Trigger, "is_localized", self.srv_localized_callback)
        self.sub_nav_to_pose = self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self.navigate_to_pose_goal_status_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.sub_follow_wp = self.create_subscription(
            GoalStatusArray,
            "follow_waypoints/_action/status",
            self.follower_waypoints_status_callback,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.led_sub = self.create_subscription(
            RobotStatus, "robot_status", self.respond_led_status, 10
        )
        self.process = None

    def respond_led_status(self, msg):
        if msg.status in [RobotStatus.NAV_RUNNING, RobotStatus.NAV_WF_RUNNING]:
            self.get_logger().info("NAV_RUNNING!!!!!")
            self.led_pub.publish(Int32(data=1))
            print(self.process)
            if self.process:
                self.get_logger().info("AAAAAAAAAAAAAAA")
                self.process.terminate()
                self.process = None
            self.get_logger().info("NAV_RUNNING end!!!")

        elif msg.status in [RobotStatus.NAV_ARRIVED]:
            self.get_logger().info("NAV_ARRIVED!!!!!")
            self.led_pub.publish(Int32(data=0))
            self.process = subprocess.Popen(["aplay", arrive_music_path])
            # self.get_logger().info(self.process)
            print(self.process)

        elif msg.status in [RobotStatus.NAV_WF_COMPLETED]:
            self.get_logger().info("NAV_WF_COMPLETED!!!!!")
            self.led_pub.publish(Int32(data=0))

        elif msg.status in [RobotStatus.NAV_FAILED, RobotStatus.NAV_WF_FAILED]:
            self.get_logger().info("NAV_FAILED!!!!!")
            self.led_pub.publish(Int32(data=7))
            if self.process:
                self.get_logger().info("BBBBBBBBBBBBBBB")
                self.process.terminate()
                self.process = None

        elif msg.status == RobotStatus.NAV_WF_ARRIVED:
            self.get_logger().info("NAV_WF_ARRIVED!!!!!")
            self.led_pub.publish(Int32(data=6))
            self.process = subprocess.Popen(["aplay", arrive_music_path])

        elif msg.status in [RobotStatus.NAV_CANCEL, RobotStatus.NAV_WF_CANCEL]:
            self.get_logger().info("NAV_CANCEL!!!!!")
            self.led_pub.publish(Int32(data=2))
            if self.process:
                self.get_logger().info("CCCCCCCCCCCCCCC")
                self.process.terminate()

        else:
            self.led_pub.publish(Int32(data=0))

    def navigate_to_pose_goal_status_callback(self, msg):
        status = msg.status_list[-1].status
        self.get_logger().info(f"NP: {status}")
        if status == GoalStatus.STATUS_EXECUTING:
            if not self.waypoins_following:
                fitrobot_status = RobotStatus.NAV_RUNNING
                status_log = "NAV_RUNNING"
            else:
                fitrobot_status = RobotStatus.NAV_WF_RUNNING
                status_log = "NAV_WF_RUNNING"
            self.status_pub.publish(RobotStatus(status=fitrobot_status))
            self.get_logger().info(status_log)
            self.set_rs_parameter(fitrobot_status)

        elif status == GoalStatus.STATUS_SUCCEEDED:
            if not self.waypoins_following:
                fitrobot_status = RobotStatus.NAV_ARRIVED
                status_log = "NAV_ARRIVED"
            else:
                fitrobot_status = RobotStatus.NAV_WF_ARRIVED
                status_log = "NAV_WF_ARRIVED"
            self.status_pub.publish(RobotStatus(status=fitrobot_status))
            self.get_logger().info(status_log)
            self.set_rs_parameter(fitrobot_status)

        elif status == GoalStatus.STATUS_CANCELED:
            if not self.waypoins_following:
                fitrobot_status = RobotStatus.NAV_CANCEL
                self.status_pub.publish(RobotStatus(status=fitrobot_status))
                self.get_logger().info("NAV_CANCEL")
                self.set_rs_parameter(fitrobot_status)

        elif status == GoalStatus.STATUS_ABORTED:
            if not self.waypoins_following:
                fitrobot_status = RobotStatus.NAV_FAILED
                self.status_pub.publish(RobotStatus(status=fitrobot_status))
                self.get_logger().info("NAV_FAILED")
                self.set_rs_parameter(fitrobot_status)

    def follower_waypoints_status_callback(self, msg):
        status = msg.status_list[-1].status
        self.get_logger().info(f"WF: {status}")
        if status == GoalStatus.STATUS_EXECUTING:
            self.waypoins_following = True

        elif status == GoalStatus.STATUS_SUCCEEDED:
            self.waypoins_following = False
            fitrobot_status = RobotStatus.NAV_WF_COMPLETED
            self.status_pub.publish(RobotStatus(status=fitrobot_status))
            self.get_logger().info("NAV_WF_COMPLETED")
            self.set_rs_parameter(fitrobot_status)

        elif status == GoalStatus.STATUS_CANCELED:
            self.waypoins_following = False
            fitrobot_status = RobotStatus.NAV_WF_CANCEL
            self.status_pub.publish(RobotStatus(status=fitrobot_status))
            self.get_logger().info("NAV_WF_CANCEL")
            self.set_rs_parameter(fitrobot_status)

        elif status == GoalStatus.STATUS_ABORTED:
            self.waypoins_following = False
            fitrobot_status = RobotStatus.NAV_WF_FAILED
            self.status_pub.publish(RobotStatus(status=fitrobot_status))
            self.get_logger().info("NAV_WF_FAILED")
            self.set_rs_parameter(fitrobot_status)

    def srv_localized_callback(self, request, response):
        response.success = self.is_localized
        response.message = f"is_localized: {self.is_localized}"
        return response

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

    def check_slam_running(self):
        nodenames = self.get_node_names()
        return True if "slam_toolbox" in nodenames else False

    def status_check(self):
        try:
            robot_status = (
                self.get_parameter("fitrobot_status")
                .get_parameter_value()
                .integer_value
            )
            if robot_status == RobotStatus.STANDBY:
                if self.is_tf_odom_baselink_existed():
                    self.get_logger().info("bringup")
                    self.status_pub.publish(RobotStatus(status=RobotStatus.BRINGUP))
                    self.set_rs_parameter(RobotStatus.BRINGUP)
                return

            elif robot_status == RobotStatus.BRINGUP:
                if self.check_nav2_running():
                    self.get_logger().info("NAV_PREPARE")
                    self.status_pub.publish(RobotStatus(status=RobotStatus.NAV_PREPARE))
                    self.set_rs_parameter(RobotStatus.NAV_PREPARE)
                elif not self.is_tf_odom_baselink_existed():
                    self.get_logger().info("standby")
                    self.status_pub.publish(RobotStatus(status=RobotStatus.STANDBY))
                    self.set_rs_parameter(RobotStatus.STANDBY)
                return

            if robot_status == RobotStatus.NAV_PREPARE:
                if self.is_tf_odom_map_existed():
                    self.get_logger().info("NAV_READY")
                    self.is_localized = True
                    self.status_pub.publish(RobotStatus(status=RobotStatus.NAV_READY))
                    self.set_rs_parameter(RobotStatus.NAV_READY)
                elif not self.check_nav2_running():
                    self.get_logger().info("bringup")
                    self.status_pub.publish(RobotStatus(status=RobotStatus.BRINGUP))
                    self.set_rs_parameter(RobotStatus.BRINGUP)
                return

            if robot_status == RobotStatus.SLAM:
                if not self.check_slam_running():
                    self.get_logger().info("bringup")
                    self.status_pub.publish(RobotStatus(status=RobotStatus.BRINGUP))
                    self.set_rs_parameter(RobotStatus.BRINGUP)
                return

            # this is expensive, needed to be optimized
            # elif robot_status == RobotStatus.NAV_READY:
            elif robot_status in [
                RobotStatus.NAV_READY,
                # RobotStatus.NAV_RUNNING,
                RobotStatus.NAV_ARRIVED,
                RobotStatus.NAV_CANCEL,
                RobotStatus.NAV_FAILED,
                # RobotStatus.NAV_WF_RUNNING,
                RobotStatus.NAV_WF_ARRIVED,
                RobotStatus.NAV_WF_COMPLETED,
                RobotStatus.NAV_WF_CANCEL,
                RobotStatus.NAV_WF_FAILED,
            ]:
                if not self.is_tf_odom_map_existed():
                    self.get_logger().info("NAV_PREPARE")
                    self.is_localized = False
                    self.status_pub.publish(RobotStatus(status=RobotStatus.NAV_PREPARE))
                    self.set_rs_parameter(RobotStatus.NAV_PREPARE)
                return

        except (tf2.LookupException, tf2.ExtrapolationException) as ex:
            self.get_logger().error("Transform lookup failed: {0}".format(str(ex)))

    def set_rs_parameter(self, status_value):
        self.set_parameters(
            [Parameter("fitrobot_status", Parameter.Type.INTEGER, status_value)]
        )


def main(args=None):
    rclpy.init(args=args)
    tf2_listener_node = RobotStatusCheckNode()
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
    [INFO] [1698831006.859219936] [check_robot_status_node]: NAV_PREPARE
    [INFO] [1698831013.862822112] [check_robot_status_node]: NAV_READY

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
