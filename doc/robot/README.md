# Robot Documentation

This section of the documentation contains a more in-depth specification of the robot's general software implementation. The directories & upper-level classes include:

- [Commands Directory](commands) - significance of command-structured robot implementation, as well as every command that the robot executes
- [Subsystems Directory](subsystems) - lower-level subsystem implementation and how its utility allows for higher-level commands to be effectively and efficiently executed
- [Constants Class](CONSTANTS.md) - explicit values that are referenced by robot software, such as motor IDs, PID values, and positional limits of subsystem structures
- [Robot Class](ROBOT.md) - main class of the robot with methods to run periodically and upon initialization of the robot
- [Robot Container Class](ROBOT_CONTAINER.md) - contains all the subsystem and command instances of the robot, configures triggers and autonomous commands to directly utilize all commands implemented by other classes