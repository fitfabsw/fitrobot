import sys
import rclpy
from fitrobot_interfaces.msg import RobotStatus
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters
from rclpy.node import Node


server_name = "check_robot_status_node"


class ExternalRobotStatusGetSet(Node):
    def __init__(self):
        super().__init__("external_status_getset_node")
        self.cli = self.create_client(SetParameters, '/' + server_name + '/set_parameters')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_cli = self.create_client(GetParameters, '/' + server_name + '/get_parameters')
        while not self.get_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetParameters service not available, waiting again...')
        self.timer = self.create_timer(3.0, self.send_set_parameters_request)
        self.timer_get = self.create_timer(1.0, self.send_get_parameters_request)
        self.count = 0

    def send_get_parameters_request(self):
        req = GetParameters.Request()
        req.names = ["fitrobot_status"]
        future = self.get_cli.call_async(req)
        future.add_done_callback(self.on_get_future_done)

    def on_get_future_done(self, future):
        try:
            response = future.result()
            for value in response.values:
                self.get_logger().info(f'Retrieved parameter value: {value.integer_value}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_set_parameters_request(self):
        print("count: ", self.count)
        param_name = "fitrobot_status"
        # param_value =self.count % 10 + 100
        param_value =self.count
        self.count += 1
        val = ParameterValue(integer_value=param_value, type=ParameterType.PARAMETER_INTEGER)
        req = SetParameters.Request()
        # if isinstance(param_value, float):
        #     val = ParameterValue(double_value=param_value, type=ParameterType.PARAMETER_DOUBLE)
        # elif isinstance(param_value, int):
        #     val = ParameterValue(integer_value=param_value, type=ParameterType.PARAMETER_INTEGER)
        # elif isinstance(param_value, str):
        #     val = ParameterValue(string_value=param_value, type=ParameterType.PARAMETER_STRING)
        # elif isinstance(param_value, bool):
        #     val = ParameterValue(bool_value=param_value, type=ParameterType.PARAMETER_BOOL)

        req.parameters = [Parameter(name=param_name, value=val)]
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_future_done)

    def on_future_done(self, future):
        try:
            response = future.result()
            if response.results[0].successful:
                self.get_logger().info("Service call successful")
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))


def main():
    rclpy.init()
    node = ExternalRobotStatusGetSet()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_timer(node.timer)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
