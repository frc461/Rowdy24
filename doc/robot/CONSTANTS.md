# Constants Class Documentation

The [`Constants`](../../src/main/java/frc/robot/Constants.java) class contains all the values that are set once and do not change. Some examples are the IDs for motors, the PID values for each motor, and positional limits for each motor. These values do not change once set, are used to make the code more readable by keeping all numerical values in one place, and are applied number in the code. 

The static subclasses within the `Constants` class organize the constants by subsystem or game component and are as follows:
- [`Auto`](../../src/main/java/frc/robot/Constants.java#L19) - constants relevant to control within autonomous period
- [`Angler`](../../src/main/java/frc/robot/Constants.java#L29) - relevant values for the angler subsystem
- [`Elevator`](../../src/main/java/frc/robot/Constants.java#L69) - relevant values for the elevator subsystem
- [`IntakeCarriage`](../../src/main/java/frc/robot/Constants.java#L99) - relevant values for the intake carriage subsystem
- [`Limelight`](../../src/main/java/frc/robot/Constants.java#L113) - relevant values for the Limelight camera
- [`Shooter`](../../src/main/java/frc/robot/Constants.java#L124) - relevant values for the shooter subsystem
- [`Swerve`](../../src/main/java/frc/robot/Constants.java#L160) - relevant values for the swerve drive subsystem, including IDs for each swerve module
