# tortoisebot_ros2_test

## Simulation
**start**
```shell
source ~/ros2_ws/install/setup.bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True
```

**fix simulation**
```shell
./fix_sim.bash
```

## Action
**compile**
```shell
cd ~/ros2_ws/ ;colcon build;source install/setup.bash
```

**start server**
```shell
source ~/ros2_ws/install/setup.bash
ros2 run tortoisebot_waypoints tortoisebot_action_server
```

**run client**
```shell
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/ ;colcon build;source install/setup.bash
```
or
```shell
ros2 action send_goal -f /tortoisebot_as tortoisebot_waypoints/action/WaypointAction "position:
  x: 0.5
  y: 0.5
  z: 0.0"
```