# Shooter Documentation

Software implementation of the Shooter subsystem on the Rowdy24 robot. The function of the shooter subsystem is to spin the shooter wheels to launch a note towards the speaker. The motors are two REV Vortexes. One motor is for the top set of wheels and the other is for the bottom.

## Implementation

### Class Information

The [`Shooter`](../../src/main/java/frc/robot/subsystems/Shooter.java) class extends the [`SubsystemBase`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) class, which helps to register commands involving the subsystem.

### Variables
The shooter is controlled by two motor controllers, one for the [top](../../src/main/java/frc/robot/subsystems/Shooter.java#L27) set of wheels and the other for the [bottom](../../src/main/java/frc/robot/subsystems/Shooter.java#L21). There is an encoder for both [top](../../src/main/java/frc/robot/subsystems/Shooter.java#L31) and [bottom](../../src/main/java/frc/robot/subsystems/Shooter.java#L25) motors and a PID controller for both [top](../../src/main/java/frc/robot/subsystems/Shooter.java#L34) and [bottom](../../src/main/java/frc/robot/subsystems/Shooter.java#L33) motors.  

Numerical metrics include [`target`](../../src/main/java/frc/robot/subsystems/Shooter.javaL49), [`error`](../../src/main/java/frc/robot/subsystems/Shooter.java#L50), and [`accuracy`](../../src/main/java/frc/robot/subsystems/Shooter.java#L51). `target` represents the speed that the motors reference to move the shooters to through electric output. `error` and `accuracy` are comparisons between encoded shooter speed and `target`, expressed as a proportion and difference, respectively.

### Methods
Many getter and setter functions are defined in the class for utility and direct electric output application. Additionally, a [`periodic`](../../src/main/java/frc/robot/subsystems/Angler.java#L41) function for the subsystem is used to update the `accuracy` and `error` variables.

The most useful methods are:
- [`shoot(double speed)`](../../src/main/java/frc/robot/subsystems/Shooter.java#L74) sets `target` value to the `speed` variable and the PID reference to `target`.
- [`setShooterSpeed(double speed)`](../../src/main/java/frc/robot/subsystems/Shooter.java#L84) sets each motor's direct output to `speed`, which is a proportion of maximum motor power (from 0 to 1).