# Dependencies

1. Gazebo 7
1. ROS (Indigo) (You can use Kinetic, but there be dragons...)

1. `gazebo-ros-pkgs` for Gazebo 7 (`sudo apt-get install ros-<distro>-gazebo7-ros-pkgs`)

# Install

1. Create a catkin workspace

```
source /opt/ros/indigo/setup.bash
export GEAR_WS=~/gear_ws  # set this to what ever you want
mkdir -p $GEAR_WS/src
cd $GEAR_WS/src
```

1. Download this repository, and others to build

```
cd $GEAR_WS/src
# This repository
git clone https://bitbucket.org/osrf/gear
# If you've got a custom version of gazebo, the gazebo_ros_pkgs
# git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b indigo-devel
```

1. Install catkin tools

```
cd ~
git clone https://github.com/catkin/catkin_tools.git
cd catkin_tools
sudo -H python -m pip install -U .
```

If you don't want to do this, use `catkin_make_isolated --install` instead the `catkin build` command.

1. Configure the catkin build

```
cd $GEAR_WS
# tell catkin to install the packages, rather than use the devel space
catkin config --install
```

1. Compile

```
cd $GEAR_WS
catkin build  # add the -i option to see streaming output from builds
```

1. Run

Recommendation: do this in a different shell than you build

```
cd $GEAR_WS
source ./install/setup.bash
```

You can run the example with `roslaunch`:

```
roslaunch osrf_gear ur10_example_world.launch
```
