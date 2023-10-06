from fitrobot_interfaces.srv import ListStation
from fitrobot_interfaces.msg import Station
import rclpy
from rclpy.node import Node
from ros2param.api import call_get_parameters
from common.utils import get_station_list


class ListStationService(Node):

    def __init__(self):
        super().__init__('list_station_service')
        self.srv = self.create_service(ListStation, 'list_station', self.list_station_callback)

    def list_station_callback(self, request, response):
        station_list = []
        station_list_json = get_station_list()
        for station_json in station_list_json:
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