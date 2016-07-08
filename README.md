# Dependencies

1. Gazebo 7
1. ROS (Indigo, Kinetic)

1. `gazebo-ros-pkgs` for Gazebo 7 (`sudo apt-get install ros-<distro>-gazebo7-ros-pkgs`)

# Install

1. Create a catkin workspace

```
source /opt/ros/kinetic/setup.bash
mkdir -p ~/catkin_gear_ws/src
cd ~/catkin_gear_ws/src
catkin_init_workspace
```

1. Download this repository

```
cd ~/catkin_gear_ws/src
git clone https://bitbucket.org/osrf/gear
```

1. Compile

```
cd ~/catkin_gear_ws
catkin_make install
```

1. Setup

```
source install/setup.sh
```

1. Run

```
roslaunch osrf_gear gripper.launch
```