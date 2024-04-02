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

```

**run client**
```shell
source ~/ros2_ws/install/setup.bash

```
