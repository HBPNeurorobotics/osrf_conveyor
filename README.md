# Conveyorbelt from Gazebo Environment for Agile Robotics (GEAR)

GEAR is the software used by teams participating in the Agile Robotics for
Industrial Automation Competition (ARIAC) hosted by the National Institute
of Standards and Technology (NIST).

Use branch ARIAC_2017!

For testing use:

    roslaunch osrf_gear test.launch (launching gazebo with test world)

    rosrun gazebo_ros spawn_model -file osrf_gear/models/conveyor/model.sdf  -sdf -model conveyor_belt

    rosservice call /ariac/conveyor/control "state: power: 100.0" (Set the conveyor speed, only 20%-100% works)

Important: if the conveyor switches from off (0%-20%) to on (20%-100$), something (another object) has to interact with the conveyor (e.g. let an object fall on it)


![ARIAC_full.png](https://bitbucket.org/repo/pB4bBb/images/1577073220-ARIAC_full.png)





To access the source code for the version of GEAR used in ARIAC 2017, use the [`ariac_2017` branch](https://bitbucket.org/osrf/ariac/commits/branch/ariac_2017).

---
