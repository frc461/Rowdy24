# Elevator Documentation

Software implementation of the Elevator subsystem on the Rowdy24 robot. The function of the elevator subsystem is to raise or lower the carriage to score in the amp or climb. The motors are two CTRE Falcon 500s.

## Implementation

### Class Information

The [`Elevator`](../../src/main/java/frc/robot/subsystems/Elevator.java) class extends the [`SubsystemBase`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) class, which helps to register commands involving the subsystem.

### Variables
The elevator is controlled by a [motor controller](../../src/main/java/frc/robot/subsystems/Elevator.java#L26) and another one that follows it, but there is only one [PID controller](../../src/main/java/frc/robot/subsystems/Elevator.java#L38). The [follower](../../src/main/java/frc/robot/subsystems/Elevator.java#45) simply mirrors the motion of the initial motor. The elevator has a [magnetic limit switch](../../src/main/java/frc/robot/subsystems/Elevator.java#L48) at the bottom to re-zero every time it reaches the minimum height. It also has a [Servo](../../src/main/java/frc/robot/subsystems/Elevator.java#L51) that works as a clamp to hold it up after the end of the match when the motors no longer have power. 

Numerical metrics include [`target`](../../src/main/java/frc/robot/subsystems/Elevator.java#L54) and [`accuracy`](../../src/main/java/frc/robot/subsystems/Elevator.java#L55). `target` represents the encoded value that the motor references to move the angler position to through electric output. `accuracy` is the comparison between encoded angler position and `target`, expressed as a difference.

The boolean [`clamped`](../../src/main/java/frc/robot/subsystems/Elevator.java#L56) represents whether the servo is clamped or not. The boolean [`movingAboveLimitSwitch`](../../src/main/java/frc/robot/subsystems/Elevator.java#L57) represents if the elevator is moving and currently above the limit switch. 

### Methods
There are many getter methods to get the status of the various switches and motors. Additionally, a [`periodic`](../../src/main/java/frc/robot/subsystems/Elevator.java#L61) function for the subsystem is used to update the `accuracy` variable and to clamp the elevator if `clamped` is set to `true`.

The most useful methods are: 
- [`holdTarget()`](../../src/main/java/frc/robot/subsystems/Elevator.java#L111) sets the elevator's position to the target variable with a PID controller.
- [`climb()`](../../src/main/java/frc/robot/subsystems/Elevator.java#L116) makes elevator go down at half power until it is told to stop, and it clamps the elevator.
- [`moveElevator()`](../../src/main/java/frc/robot/subsystems/Elevator.java#L138) simply gives the elevator power based on the joystick value from the operator.
- [`setHeight()`](../../src/main/java/frc/robot/subsystems/Elevator.java#L152) changes the elevator's target to the specified height and calls holdTarget().