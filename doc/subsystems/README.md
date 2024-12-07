# Subsystems Documentation

This section of the documentation contains a more in-depth specification of the software implementation of each subsystem. Each subsystem has its own page that will go into detail about its functions (methods), objectives, and control. The subsystem classes include:

- [Angler](ANGLER.md) - mechanical system in the robot used to adjust the angle of the shooter
- [Elevator](ELEVATOR.md) - mechanical system in the robot with the ability to extend upward to increase its height and reach
- [Intake/Carriage](INTAKECARRIAGE.md) - integrated mechanical system in the robot to collect, store, or transfer a game piece 
- [Limelight](LIMELIGHT.md) - camera system on the robot used for vision processing and localization
- [Shooter](SHOOTER.md) - mechanical system attached to the angler to launch a game piece
- [Swerve](SWERVE.md) - drivetrain system on the robot that allows for omnidirectional movement
- [Swerve Module](SWERVE%20MODULE.md) - component of the swerve subsystem representing one wheel

Each of these subsystem classes contain primary, lower-level functions that are used to implement commands.