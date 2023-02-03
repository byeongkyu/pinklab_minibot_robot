# minibot_bringup

## Gazebo Simulation
```shell
$ ros2 launch minibot_bringup bringup_robot_gazebo.launch.py lidar_model:=ydlidar_x2 world_name:=simple_building.world
```

## Real robot
```shell
$ ros2 launch minibot_bringup bringup_robot.launch.py lidar_model:=ydlidar_x2
```