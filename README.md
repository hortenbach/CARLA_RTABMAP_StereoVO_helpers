# CARLA RTABMAP StereoVO helpers :floppy_disk: :pill:
Collection of notes and helpers when using rtabmap in ROS2 for Visual Odometry with CARLA Simulator

## CARLA Notes

My workflow for recording new data is

```
1) start.py

2) ros2 bag record -a -o mydataset_01         
```
BUT be careful with the default config file used for carla_spawn_objects.launch.py
Theres a default MAP defined there and it will show up in your tf tree and it will interfere with
ANY localization or mapping algorithm you are working with. Just delete it from the config. 
Btw if it is not obvious at this point, in localization we dont have a map in our tf. 
Its just [odom]-->[ego_vehicle]-->[sensors]
