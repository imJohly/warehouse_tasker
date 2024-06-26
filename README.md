# Warehouse Tasker

![warehouser.png](images/warehouser.png)

A repo for the warehouse task allocation system for a fleet of turtlebots.

See [warehouse_tasker_interfaces](https://github.com/imJohly/warehouse_tasker_interfaces/) for info on the interfaces package.

## Dependencies
To properly run this, there a number of dependencies that must be installed first,

### Python Dependencies

The following libraries must be installed,

- scikit-learn
- opencv-contrib-python

Use the following line to install,

```
pip3 install scikit-learn opencv-contrib-python
```

### Package Dependencies

The [turtlebot3_multi_robot](https://github.com/arshadlab/turtlebot3_multi_robot) package from ashadlab and [ros_aruco_opencv](https://github.com/fictionlab/ros_aruco_opencv/tree/humble) package from fictionlab needs to be installed with the following,

The following is for humble, but should work for foxy (just replace humble).

First build and install LKH-3 and the python wrapper PyLKH with following commands,

```bash
cd
wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.10.tgz
tar xvfz LKH-3.0.10.tgz
cd LKH-3.0.10
make
sudo cp LKH /usr/local/bin
```

```bash
sudo apt install ros-humble-aruco-opencv

cd ~/<your ros2_ws here>/src
git clone -b master https://github.com/arshadlab/turtlebot3_multi_robot.git
```

Install any dependencies for these packages with the following,

```bash
cd ..
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -r -y
```

Next, build the package with a symlink and source them,

```bash
colcon build --symlink-install
source ./install/setup.bash
```

Now the workspace is ready to install the warehouse_tasker package.

## How to install

First clone this repo through SSH into your ros2 workspace,

```bash
cd ~/<your ros2_ws here>/src
git clone git@github.com:imJohly/warehouse_tasker_interfaces.git
git clone git@github.com:imJohly/warehouse_tasker.git
```

Then build it using colcon and source it from the install directory,

```bash
cd ..
colcon build --packages-select warehouse_tasker warehouse_tasker_interfaces --symlink-install
source install/local_setup.bash
```

It should have built successfully, but lets test it.

## Launching on real robots,

To launch all components for real robots,

```bash
ros2 launch warehouse_tasker real_multi_nav2_world.launch.py
```

```bash
ros2 launch warehouse_tasker real_marker_broadcaster.launch.py
```

Launch mission with the number of goals present
```bash
ros2 launch warehouse_tasker mission.launch.py goal_count:=20
```

```bash
ros2 run warehouse_tasker agent_node --ros-args -r __ns:=/tb1 -p use_door:=True
```

```bash
ros2 run warehouse_tasker agent_node --ros-args -r __ns:=/tb2 -p use_door:=True
```

Then to send goals,
```bash
ros2 service call /send_task warehouse_tasker_interfaces/srv/SendTask "{agent: '', goal: ['3', '5', '4', '7', '6']}"
```

## Launching the test environment

To launch the test environment,

```bash
ros2 launch warehouse_tasker world.launch.py
```

For multi robots,

```bash
ros2 launch warehouse_tasker world_smaller_dual_namespace.launch.py
```

To launch RVIZ inidividually for each robot,

```bash
rviz2 -d /home/callum/catkin_ws/src/turtlebot3_multi_robot/rviz/multi_nav2_default_view.rviz --ros-args -r __node:=rviz2 -r __ns:=/tb1 -r /tf:=tf -r /tf_static:=tf_static -r /goal_pose:=goal_pose -r /clicked_point:=clicked_point -r /initialpose:=initialpose
```
