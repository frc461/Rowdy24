# Constants Class Documentation

The `Constants` class contains all the values that are set once and do not change. Some examples are the IDs for motors, the PID values for each motor, and positional limits for each motor. These values do not change once set, are used to make the code more readable by keeping all numerical values in one place, and are applied number in the code. 

The static subclasses within the `Constants` class organize the constants by subsystem or game component and are as follows:
- `Auto` - constants relevant to control within autonomous period
- `Angler` - relevant values for the angler subsystem
- `Elevator` - relevant values for the elevator subsystem
- `IntakeCarriage` - relevant values for the intake carriage subsystem
- `Limelight` - relevant values for the Limelight camera
- `Shooter` - relevant values for the shooter subsystem
- `Swerve` - relevant values for the swerve drive subsystem, including IDs for each swerve module
