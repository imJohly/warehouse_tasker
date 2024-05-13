# Warehouse Tasker

![warehouser.png](images/warehouser.png)

A repo for the warehouse task allocation system for a fleet of turtlebots.

## How to install

First clone this repo through SSH into your ros2 workspace,

```
cd ~/<your ros2_ws here>/src
git clone git@github.com:imJohly/warehouse_tasker.git
```

Then build it using colcon and source it from the install directory,

```
cd ..
colcon build --packages-select warehouse_tasker --symlink-install
source install/local_setup.bash
```

It should have built successfully, but lets test it.

## Launching the test environment

To launch the test environment,

```
ros2 launch warehouse_tasker world.launch.py
```

For multi robots,

```
ros2 launch warehouse_tasker world_triple_robot.launch.py
```
