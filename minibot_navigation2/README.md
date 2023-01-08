# former_nav2

## Map Building

&nbsp;

### For Gazebo simulation
```shell
$ ros2 launch former_gazebo bringup.launch.py world_name:=office_building.world
```
```shell
$ ros2 launch former_nav2 map_building.launch.py use_sim_time:=true
```
```shell
$ rviz2 -d `ros2 pkg prefix former_nav2`/share/former_nav2/rviz/map_building.rviz
```
```shell
$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=base_controller/cmd_vel_unstamped
```