import os
import json
from inspect import getsourcefile
from os.path import abspath
from subprocess import check_output
from pathlib import Path
from fitrobot_interfaces.srv import ListStation
from fitrobot_interfaces.msg import Station
import rclpy
from rclpy.node import Node


class ListStationService(Node):

    def __init__(self):
        super().__init__('list_station_service')
        self.srv = self.create_service(ListStation, 'list_station', self.list_station_callback)

    def list_station_callback(self, request, response):
        map_name = check_output(["ros2", "param", "get", "/master_service", "active_nav_map"]).decode("utf-8")
        map_name = map_name[17:].strip()
        station_list_file_path = Path(os.path.dirname(abspath(getsourcefile(lambda:0)))).parent.joinpath("data","station_list.json")
        station_list = []
        with open(station_list_file_path, 'r') as json_file:
            complete_station_list_json = json.loads(json_file.read())
            station_list_json = complete_station_list_json[map_name]
        for station_json in station_list_json['station_list']:
            print(station_json)
            station = Station()
            station.type = station_json["type"]
            station.name = station_json["name"]
            station.x = station_json["x"]
            station.y = station_json["y"]
            station_list.append(station)
            

        response.station_list = station_list
        
        return response


def main():
    rclpy.init()
    list_station_service = ListStationService()
    rclpy.spin(list_station_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()