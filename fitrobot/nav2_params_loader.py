import os
import yaml
import asyncio

# import aiofiles
from argparse import ArgumentParser
from io import StringIO
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

includes = [
    # "amcl",
    # "bt_navigator",
    "controller_server",
    "local_costmap",
    "global_costmap",
    # "map_server",
    # "map_saver",
    # "planner_server",
    # "smoother_server",
    # "behavior_server",
    # "waypoint_follower",
    # "velocity_smoother",
]
includes = [f"/{key}" for key in includes]

excludes = {
    "/controller_server": [
        "ros__parameters.FollowPath.motion_model",
    ],
    "/local_costmap": [],
    "/global_costmap": [],
}


def exclude_keys(target_dict, keys_to_exclude):
    for exclude_str in keys_to_exclude:
        keys = exclude_str.split(".")
        current_dict = target_dict
        for key in keys[:-1]:
            current_dict = current_dict.get(key, {})
        current_dict.pop(keys[-1], None)


def handle_costmap_params(filepath, key):
    # key = "/local_costmap" or "/global_costmap"
    key = key[1:]
    # print(f"handle_costmap_params key", key)
    with open(filepath) as f:
        f_ = f.read().replace(f"/{key}:\n  {key}:\n", f"/{key}/{key}:\n")
        with open(filepath, "w") as g:
            g.write(f_)


class Nav2ParamsLoader(Node):
    def __init__(self):
        super().__init__("nav2_params_loader")
        self.subscription = self.create_subscription(
            String, "topic", self.listener_callback, 10
        )

    def seperate_nav2_params_file(self, nav2_params):
        for key, value in nav2_params.items():
            if key in includes:
                print("seperate_nav2_params_file key", key)
                # print("value", value)
                exclude_keys(nav2_params[key], excludes[key])
                filepath = f"/tmp/{key[1:]}.yaml"
                yaml.safe_dump({key: value}, open(filepath, "w"))
                if key in ["/local_costmap", "/global_costmap"]:
                    handle_costmap_params(filepath, key)

    def remove_tmp_files(self, nav2_params):
        print("remove_tmp_files")
        # for key in nav2_params:
        #     if key in includes:
        #         print("key", key)
        # os.remove(f"/tmp/{key}.yaml")

    async def load_params(self, nav2_params):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(
                asyncio.gather(
                    *(self.load_param(key) for key in nav2_params if key in includes)
                )
            )
        finally:
            loop.close()

    async def load_param(self, key):
        print("load_param key", key)
        if key in ["/local_costmap", "/global_costmap"]:
            cmd = f"ros2 param load {key}{key} /tmp/{key[1:]}.yaml"
        else:
            cmd = f"ros2 param load {key} /tmp/{key[1:]}.yaml"
        process = await asyncio.create_subprocess_shell(
            cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        stdout, stderr = await process.communicate()
        if process.returncode != 0:
            print(f"Failure load ros2 param load {key} /tmp/{key[1:]}.yaml")
            print(f"Stderr: {stderr.decode()}")

    async def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.nav2_params_file = msg.data
        assert os.path.isfile(self.nav2_params_file)
        nav2_params = yaml.safe_load(open(self.nav2_params_file))
        nav2_params = {f"/{k}": v for k, v in nav2_params.items()}
        # print("keys", nav2_params.keys())
        self.seperate_nav2_params_file(nav2_params)

        # Create a new task to run load_params
        await self.load_params(nav2_params)
        self.remove_tmp_files(nav2_params)


def main(args=None):
    rclpy.init(args=args)
    np_subscriber = Nav2ParamsLoader()

    # Get a reference to the asyncio loop and run it until the node has completed execution.
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(rclpy.spin(np_subscriber))
    finally:
        np_subscriber.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    """
    Description:
        without set paramerter one by one using ros2 param load command, 
        this node can load nav2_params.yaml file at once.
    Usage
        ros2 run nav2_params_loader nav2_params_loader
    Example ros2 cli:
        ros2 topic pub -1 topic std_msgs/msg/String "{data: /path/to/nav2_params.yaml}"
    """
    main()
