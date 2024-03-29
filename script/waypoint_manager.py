#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from script.robot_navigator import BasicNavigator, NavigationResult
from fitrobot_interfaces.msg import Station
from common.utils import get_start_and_end_stations


class WaypointManager:
    def __init__(self):
        self.current_station = None
        self.start_station = None
        self.end_station = None

        self.navigator = BasicNavigator()

        start_station, end_station = get_start_and_end_stations()
        self.set_start_station(start_station)
        self.set_end_station(end_station)
        
        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()

    def set_start_station(self, station: Station):
        # Set docking station (where the robot stays when there is no active task)
        self.start_station = station
    
    def set_end_station(self, station: Station):
        # Set the target position (where the robot send items to)
        self.end_station = station
    
    def convert_station_to_pose(self, station: Station) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = station.x
        pose.pose.position.y = station.y
        pose.pose.orientation.z = station.z
        pose.pose.orientation.w = station.w

        return pose


    def add_station(self, station: Station):
        goal_stations = []

        goal_stations.append(station)
        goal_stations.append(self.end_station)
        goal_stations.append(self.start_station)

        # nav_start = self.navigator.get_clock().now()

        goal_poses = list(map(self.convert_station_to_pose, goal_stations))
        self.navigator.followWaypoints(goal_poses)
        self.current_station = station
        
        print(f"開始執行站點 (name:{self.current_station.name}, x:{self.current_station.x}, y:{self.current_station.y}) 運送任務")

        i = 0
        while not self.navigator.isNavComplete():
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                if feedback.current_waypoint == 0:
                    self.current_station = station
                    print(f"前往站點 (name:{self.current_station.name}, x:{self.current_station.x}, y:{self.current_station.y}) 運送任務")
                elif feedback.current_waypoint == 1:
                    self.current_station = self.end_station
                    print(f"運送至FA Room")
                elif feedback.current_waypoint == 2:
                    self.current_station = self.start_station
                    print(f"返回充電座")
                # now = self.navigator.get_clock().now()

        #         # Some navigation timeout to demo cancellation
        #         if now - nav_start > Duration(seconds=600.0):
        #             self.navigator.cancelNav()

        # # Do something depending on the return code
        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print('運送任務完成!')
        elif result == NavigationResult.CANCELED:
            print('運送任務取消!')
        elif result == NavigationResult.FAILED:
            print('運送任務失敗!')
        else:
            print('運送任務回傳狀態不合法!')

        # navigator.lifecycleShutdown()

        return


if __name__ == '__main__':
    pass
