import yaml

# input
# key = "/controller_server"

excludes = {
    "/controller_server": [
        "ros__parameters.controller_frequency",
        "ros__parameters.FollowPath.motion_model",
        "ros__parameters.general_goal_checker.stateful",
    ],
    "/local_costmap": [],
}

local_params = """/local_costmap:
  local_costmap:
    ros__parameters:
      always_send_full_costmap: true
"""


# 数据
x = {
    "/controller_server": {
        "ros__parameters": {
            "controller_frequency": 20.0,
            "min_x_velocity_threshold": 0.001,
            "min_y_velocity_threshold": 0.5,
            "FollowPath": {
                "temperature": 0.3,
                "gamma": 0.015,
                "motion_model": "DiffDrive",
            },
            "general_goal_checker": {
                "stateful": True,
                "plugin": "nav2_controller::SimpleGoalChecker",
                "xy_goal_tolerance": 0.25,
                "yaw_goal_tolerance": 0.25,
            },
        }
    }
}


def exclude_keys(target_dict, keys_to_exclude):
    for exclude_str in keys_to_exclude:
        keys = exclude_str.split(".")
        current_dict = target_dict
        for k in keys[:-1]:
            current_dict = current_dict.get(k, {})
        current_dict.pop(keys[-1], None)


# 输出结果, expect
x_pared = {
    "/controller_server": {
        "ros__parameters": {
            "min_x_velocity_threshold": 0.001,
            "min_y_velocity_threshold": 0.5,
            "FollowPath": {
                "temperature": 0.3,
                "gamma": 0.015,
            },
            "general_goal_checker": {
                "stateful": True,
                "plugin": "nav2_controller::SimpleGoalChecker",
                "xy_goal_tolerance": 0.25,
                "yaw_goal_tolerance": 0.25,
            },
        }
    }
}

# 调用函数


nav2_params_file = "/home/zealzel/simulations/nav2_params.yaml"
nav2_params = yaml.safe_load(open(nav2_params_file))
nav2_params = {f"/{k}": v for k, v in nav2_params.items()}

# key = "/controller_server"
key = "/local_costmap"
exclude_keys(nav2_params[key], excludes[key])


print(x)
