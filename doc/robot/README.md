# Robot Documentation

The classes within this directory are developed directly for the implementation of the robot. Through primitive or advanced action, subsystem or command, through the container class, every action that the Rowdy24 robot performs is defined here.

## Robot Class

The [`Robot`](../../src/main/java/frc/robot/Robot.java) class is the base Java class that is automatically run by the virtual machine (via [`Main`](../../src/main/java/frc/robot/Main.java)) on the RoboRIO of the robot after deployment. That means that through deployment, the physical robot executes the methods within this class. The methods can be run periodically and upon initialization of a certain operational mode. Modes include "deployment"/"robot," "disabled," "autonomous," "teleop," and "test." Each "period" or "tick" by default lasts 20 ms, so periodic functions to be called will run automatically every 20 ms.

### Methods

- [`robotInit()`](../../src/main/java/frc/robot/Robot.java#L28) - called once when the robot is initialized
- [`robotPeriodic()`](../../src/main/java/frc/robot/Robot.java#L49) - called periodically regardless of current mode
- [`disabledInit()`](../../src/main/java/frc/robot/Robot.java#L65) - called once when the robot is disabled
- [`disabledPeriodic()`](../../src/main/java/frc/robot/Robot.java#L69) - called periodically when the robot is disabled
- [`autonomousInit()`](../../src/main/java/frc/robot/Robot.java#L77) - called once when the robot enters autonomous mode
- [`autonomousPeriodic()`](../../src/main/java/frc/robot/Robot.java#L87) - called periodically when the robot is in autonomous mode
- [`teleopInit()`](../../src/main/java/frc/robot/Robot.java#L91) - called once when the robot enters teleoperated mode
- [`teleopPeriodic()`](../../src/main/java/frc/robot/Robot.java#L105) - called periodically when the robot is in teleoperated mode
- There are other methods for different robot modes that are similar in dynamic to the aforementioned mode methods

## Rowdy24 Robot Implementation
This section of the documentation contains a more in-depth specification of the robot's general software implementation. The directories & upper-level classes include:

- [Commands Directory](commands) - significance of command-structured robot implementation, as well as every command that the robot executes
- [Subsystems Directory](subsystems) - lower-level subsystem implementation and how its utility allows for higher-level commands to be effectively and efficiently executed
- [Constants Class](CONSTANTS.md) - explicit values that are referenced by robot software, such as motor IDs, PID values, and positional limits of subsystem structures
- [Robot Container Class](ROBOT_CONTAINER.md) - contains all the subsystem and command instances of the robot, configures triggers and autonomous commands to directly utilize all commands implemented by other classes