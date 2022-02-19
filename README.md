# CARLA RTABMAP StereoVO helpers :floppy_disk: :pill:
Collection of notes and helpers when using rtabmap in ROS2 for Visual Odometry with CARLA Simulator

## CARLA Notes

My workflow for recording new data is basically

```
1) ./CarlaUE4.sh

2) ros2 launch carla_spawn_objects carla_spawn_objects.launch.py town:=Town03 timeout:=10

3) ros2 run carla_vo stereo_syncronizer2

4) ros2 launch carla_manual_control carla_manual_control.launch.py

5) ros2 launch stereo_rtabmap_launch.py 

6) ros2 bag record -a -o my_bag3000         
```
BUT be careful with the default config file used for carla_spawn_objects.launch.py
Theres a default MAP defined there and it will show up in your tf tree and it will interfere with
ANY localization or mapping algorithm you are working with. Just delete it from the config. 
Btw if it is not obvious at this point, in localization we dont have a map in our tf. 
Its just [odom]-->[ego_vehicle]-->[sensors]
