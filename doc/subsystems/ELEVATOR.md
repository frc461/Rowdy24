# Elevator Documentation

Software implementation of the Elevator subsystem on the Rowdy24 robot. The function of the elevator subsystem is to raise or lower the carriage to score in the amp or climb. The motor is two CTRE Falcon 500s.

## Implementation

### Variables
The elevator is controlled by a motor controller and another one that follows it, but there is only one PID controller. The follower simply mirrors the motion of the initial motor. The elevator has a magnetic limit switch at the bottom to re-zero every time it reaches the minimum height. It also has a Servo that works as a clamp to hold it up after the end of the match when the motors no longer have power. 

Numerical metrics include target and accuracy. Target represents the encoded value that the motor needs to move the elevator position to through electric output. Accuracy determine how close the elevator actually is to where it is supposed to be, considering possible input values to move or set the elevator to a certain position. 

The boolean clamped represents whether the servo is clamped or not. The boolean movingAboveLimitSwitch represents if the elevator is moving and currently above the limit switch. 

### Methods
There are many getter methods to get the status of the various switches and motors. 
The most useful methods are: 
- [holdTarget()](../../src/main/java/frc/robot/subsystems/Elevator.java#L109) sets the elevator's position to the target variable with a PID controller. 
- [setHeight()](../../src/main/java/frc/robot/subsystems/Elevator.java#L150), which changes the elevator's target to height and calls holdTarget().
- [moveElevator()](../../src/main/java/frc/robot/subsystems/Elevator.java#L136) simply gives the elevator power based on the joystick value from the opperator. 
- [climb()](../../src/main/java/frc/robot/subsystems/Elevator.java#L114) makes elevator go down at half power until it is told to stop and it clamps the elevator. 