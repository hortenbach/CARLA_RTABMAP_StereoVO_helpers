#!/bin/python3
from multiprocessing import Process
import time
import os

CARLAPATH = "path/to/carla/CarlaUE4.sh"

def carla_server():
    os.system(CARLAPATH)

def rosbridge():
    time.sleep(10)
    os.system("ros2 launch carla_ros_bridge carla_ros_bridge.launch.py timeout:=10 town:=Town03")

def carlaspawn():
    time.sleep(15)
    os.system("ros2 launch carla_spawn_objects carla_spawn_objects.launch.py town:=Town03 timeout:=10")

def manual():
    time.sleep(18)
    os.system("ros2 launch carla_manual_control carla_manual_control.launch.py")

def runInParallel(*fns):
    global proc
    for fn in fns:
        p = Process(target=fn)
        p.start()
        proc.append(p)
    for p in proc:
        p.join()

proc = []

if __name__ == '__main__':
    try:
        runInParallel(carla_server, rosbridge, carlaspawn, manual)
    except KeyboardInterrupt:
        for p in proc:
            p.terminate()
            time.sleep(3)
