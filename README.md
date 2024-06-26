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
ros2 run tortoisebot_waypoints exp_action_client
```
or
```shell
ros2 action send_goal -f /tortoisebot_as tortoisebot_waypoints/action/WaypointAction "position:
  x: 0.5
  y: 0.5
  z: 0.0"
```
## TEST
build
```shell
cd ~/ros2_ws/ ;colcon build;source install/setup.bash
```
run test
```shell
colcon test --packages-select tortoisebot_waypoints --event-handler=console_direct+
```
check summary
```shell
colcon test-result --test-result-base build/tortoisebot_waypoints
```
clean tests
```
rm -r ~/ros2_ws/build/tortoisebot_waypoints/test_results
rm -r ~/ros2_ws/build/tortoisebot_waypoints/Testing
```

## RESULTS
> [!NOTE] test-pass: 
tests pass correctly if the goal is reached before and before timeout
![test-pass](pictures/ros2_pass.png)

> [!CAUTION] test-fail:
tests don't pass correctly if the goal is not reached or due to timeout
![test-fail](pictures/ros2_fail.png)
