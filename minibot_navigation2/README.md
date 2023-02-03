# minibot_navigation2

## Map Building

### For Gazebo simulation
```shell
$ ros2 launch minibot_bringup bringup_robot_gazebo.launch.py lidar_model:=ydlidar_x2 world_name:=simple_building.world
```
```shell
$ ros2 launch minibot_navigation2 map_building.launch.py use_sim_time:=true
```
```shell
$ rviz2 -d `ros2 pkg prefix minibot_navigation2`/share/minibot_navigation2/rviz/map_building.rviz
```
```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=base_controller/cmd_vel_unstamped
```