# Kickoff

1. Simulation environment
  1. Develop a model of the manufacturing environment.
  1. Develop a conveyor belt.
1. Robot arms & grippers
  1. Choose three robot arms, and integrate with the environment.
  1. Develop a public API for the robot arm(s).
  1. Integrate LogicalCamera with guassian noise
  1. Integrate laser scanner. Model a SICK.
    1. Write a plugin that allows the laser scanner to act as a break-beam,
       proximity sensor, or light-curtain. Look at industry standards.
1. Configurable sensor placement
  1. Teams need a simple mechanism to place sensors and robot arms.
    1. Location, type, and number of sensors
    1. Type and starting location of the robot arm
    1. Make a validation tool that confirms if a configuration is plausible.
      1. We could specify a set of valid locations.
      1. This tool should also compute the cost of the configuration.
1. Gazebo rules and scoring plugin
  1. Run the different simulation tasks.
  1. Score each task.
