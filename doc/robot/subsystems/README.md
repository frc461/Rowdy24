# Subsystems Documentation

Each of these subsystem classes contains primary, lower-level functions that are used to implement more higher-level commands. Many of these functions utilizes motor Java classes imported as vendor dependencies for direct electric output to the robot. Each subsystem implementation allows for the conversion from software to motor control.

## Rowdy24 Subsystem Implementation
This section of the documentation contains a more in-depth specification of each subsystem's software implementation. Each page will specify the functions (methods), objectives, and control of their respective subsystem:

- [Angler](ANGLER.md) - mechanical system in the robot used to adjust the angle of the shooter
- [Elevator](ELEVATOR.md) - mechanical system in the robot with the ability to extend upward to increase its height and reach
- [Intake/Carriage](INTAKE_CARRIAGE) - integrated mechanical system in the robot to collect, store, or transfer a game piece 
- [Limelight](LIMELIGHT.md) - camera system on the robot used for vision processing and localization
- [Shooter](SHOOTER.md) - mechanical system attached to the angler to launch a game piece
- [Swerve](SWERVE.md) - drivetrain system on the robot that allows for omnidirectional movement
- [Swerve Module](SWERVE%20MODULE.md) - component of the swerve subsystem representing one wheel