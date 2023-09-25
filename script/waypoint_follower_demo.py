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

import time
import rclpy
import queue

from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from script.robot_navigator import BasicNavigator, NavigationResult
from fitrobot_interfaces.msg import Point
from threading import Thread


'''
Basic navigation demo to go to poses.
'''


class WaypointManager:
    def __init__(self):
        self.pose_queue = queue.Queue()
        self.start_pose = None
        self.end_pose = None

        self.navigator = BasicNavigator()

        # Set our demo's initial pose
        # self.navigator.setInitialPose(initial_pose)
        self.set_end_pose(self.make_pose(-0.5,-0.5))
        self.set_start_pose(self.make_pose(0.0,0.0))

        # Wait for navigation to fully activate, since autostarting nav2
        self.navigator.waitUntilNav2Active()
        # self.consumer = Thread(target=self.consume_points)
        # self.consumer.start()

    def set_start_pose(self, p:PoseStamped):
        # Set docking station (where the robot stays when there is no active task)
        self.start_pose = p
    
    def set_end_pose(self, p:PoseStamped):
        # Set the target position (where the robot send items to)
        self.end_pose = p
    
    def make_pose(self, x:float = 0.0, y:float = 0.0)->PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        return pose
    
    # def consume_points(self):
    #     while True:
    #         if self.navigator.isNavComplete() and self.pose_queue.not_empty:
    #             print("完成")
    #             try:
    #                 pose = self.pose_queue.get()
    #                 print(f"消耗 {pose}")
    #                 self.navigator.followWaypoints([pose])
    #                 i=0
    #                 while not self.navigator.isNavComplete():
    #                     print(f"未完成{i}")
    #                     i = i + 1
    #             except Exception:
    #                 continue
    #         time.sleep(1)



    def add_points(self, point_list:[Point]):
        
        goal_poses = []
        for point in point_list:
            print(point.x, point.y)
            goal_pose = self.make_pose(point.x, point.y)
            goal_poses.append(goal_pose)
            goal_poses.append(self.end_pose)
            goal_poses.append(self.start_pose)
            self.pose_queue.put(goal_pose)

        # nav_start = self.navigator.get_clock().now()

        self.navigator.followWaypoints(goal_poses)
        poped_pose = self.pose_queue.get()
        goal_x = poped_pose.pose.position.x
        goal_y = poped_pose.pose.position.y
        print(f"開始執行站點 (x:{goal_x}, y:{goal_y}) 運送任務")

        i = 0
        while not self.navigator.isNavComplete():
        #     ################################################
        #     #
        #     # Implement some code here for your application!
        #     #
        #     ################################################

        #     # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 10 == 0:
                if feedback.current_waypoint == 0:
                    print(f"前往站點 (x:{goal_x}, y:{goal_y})")
                elif feedback.current_waypoint == 1:
                    print(f"運送至FA Room")
                elif feedback.current_waypoint == 2:
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

    # def __del__(self):
    #     self.consumer.join()

if __name__ == '__main__':
    pass
