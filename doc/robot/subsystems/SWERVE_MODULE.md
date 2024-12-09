# Swerve Documentation

Software implementation of the swerve module on the Rowdy24 robot. This class represents one wheel, a component of the drivetrain, on the robot. The [Swerve](SWERVE.md) subsystem is able to coordinate multiple instances of this module class to move the robot through the module class's specifications and tools/methods. Each swerve module contains two Rev NEOs, one for drive and one for rotation.

## Implementation

### Class Information

The [`SwerveModule`](../../../src/main/java/frc/robot/subsystems/SwerveModule.java) class does not extend the [`SubsystemBase`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) class, as it is only defined as a component of the Swerve subsystem and is not functional without being configured by the [`Swerve`](../../../src/main/java/frc/robot/subsystems/Swerve.java) class.

### Variables

The swerve module is identified and organized with other modules using its assigned [number](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L39) and is determined in the implementation of all four modules in the `Swerve` class. The angle [offset](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L40) of the module is determined by analyzing the angle measurement recorded by its [absolute angle encoder](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L43) (a distinct CANcoder encoder that maintains consistent angle measurement after power loss) when the module lines up with all other modules in a consistent direction.

A swerve module entails an angle [motor](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L50) along with its relative [encoder](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L51) (used for reference and control after calibration with the absolute encoder) and mechanical [controller](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L52), as well as a drive [motor](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L56) along with its relative [encoder](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L57) and mechanical [controller](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L58). A [feedforward](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L61) calculator is also used in addition to the aforementioned PID controllers to directly apply power to the drive motor.

The sole primitive numerical metric of a swerve module is [`currentReferenceAngle`](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L65), recalling the angle position that the angle relative encoder is applying power to move to.

### Methods

Many getter functions are defined in the class for utility.

The most useful methods are:
- [`resetToAbsolute()`](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L92) calibrates the angle relative encoder with the CANCoder absolute encoder in order to accurately use the angle relative encoder for reference.
- [`setDesiredState()`](../../../src/main/java/frc/robot/subsystems/SwerveModule.java#L152) sets the reference module state as specified and calculated by the `Swerve` class in coordination with all other swerve modules.