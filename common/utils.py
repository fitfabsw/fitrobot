import logging
import os
import json
from pathlib import Path
from inspect import getsourcefile
from os.path import abspath
from subprocess import check_output
from fitrobot_interfaces.msg import Station


def get_logger(logger_name):
    # create logger
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.DEBUG)

    # create console handler and set level to debug
    ch = logging.StreamHandler()
    ch.setLevel(logging.DEBUG)

    # create formatter
    formatter = logging.Formatter('[%(name)s] %(message)s')

    # add formatter to ch
    ch.setFormatter(formatter)

    # add ch to logger
    logger.addHandler(ch)
    return logger

def get_station_list():
    map_name = check_output(["ros2", "param", "get", "/master_service", "active_nav_map"]).decode("utf-8")
    map_name = map_name[17:].strip()

    # call_get_parameters()
    station_list_file_path = Path(os.path.dirname(abspath(getsourcefile(lambda:0)))).parent.joinpath("data","station_list.json")

    # current_script_path = Path(__file__).resolve().parent
    # station_list_file_path = current_script_path / "data" / "station_list.json"

    station_list = []
    with open(station_list_file_path, 'r') as json_file:
        complete_station_list_json = json.loads(json_file.read())
        station_list_json = complete_station_list_json[map_name]["station_list"]
        
    return station_list_json

def get_start_and_end_stations() -> (Station, Station):
    station_list_json = get_station_list()
    
    s = list(filter(lambda x:x["type"]=="start", station_list_json))[0]
    e = list(filter(lambda x:x["type"]=="end", station_list_json))[0]
    start = Station(type=s["type"], name=s["name"], x=s["x"], y=s["y"], z=s["z"], w=s["w"])
    end = Station(type=e["type"], name=e["name"], x=e["x"], y=e["y"], z=e["z"], w=e["w"])
    return start, end

