# Swerve Documentation

Software implementation of the swerve subsystem on the Rowdy24 robot. The function of the swerve subsystem is to move the robot by coordinating the swerve modules to move together. The subsystem entails four swerve modules.

## Implementation

### Class Information

The [`Swerve`](../../../src/main/java/frc/robot/subsystems/Swerve.java) class extends the [`SubsystemBase`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) class, which helps to register commands involving the subsystem. The constructor also [configures](../../src/main/java/frc/robot/subsystems/Swerve.java#L71) PathPlanner pathfinding to be optimized with Rowdy24's drivetrain when generating autonomous drivetrain commands.

### Enums

The swerve subsystem contains the [`TurretTargets`](../../../src/main/java/frc/robot/subsystems/Swerve.java#L33) enum class that is used to provide options to align with either the speaker for shooting notes or the corner of the field closest to the amp for shuttling notes.

### Variables

The swerve subsystem uses [odometry](../../../src/main/java/frc/robot/subsystems/Swerve.java#L26) to primitively track the robot's position and a [pose estimator](../../../src/main/java/frc/robot/subsystems/Swerve.java#L27) to precisely estimate the robot's position. The swerve subsystem is made up of four [swerve modules](../../../src/main/java/frc/robot/subsystems/Swerve.java#L28), each with its own motor controller and encoder. An [integrated angle PID controller](../../../src/main/java/frc/robot/subsystems/Swerve.java#L29) is also used for control in reference to a desired gyro angle of the drivetrain. The swerve subsystem also has a [gyro](../../../src/main/java/frc/robot/subsystems/Swerve.java#L30) to primitively track the robot's heading.

The sole primitive numerical metric of the swerve subsystem is [`turretToSpeakerError`](../../../src/main/java/frc/robot/subsystems/Swerve.java#L31), measuring the difference between the heading of the robot and the angle of the robot to the speaker.

A boolean field [`isConfigured`](../../../src/main/java/frc/robot/subsystems/Swerve.java#L32) is to determine whether the pose estimator should implement MegaTag2 rather than MegaTag. This logic follows as MegaTag2 requires an accurate heading from the robot, which is only obtainable with MegaTag. Thus, `isConfigured` is set to false and pose estimation implements MegaTag, and once MegaTag syncs the heading of the robot with an accuracy to the point that MegaTag2 measurements are consistent with MegaTag, `isConfigured` is set to `true` and pose estimation is pivoted to implementing MegaTag2 due to its higher consistency and precision relative to MegaTag when given an accurate heading.

### Methods

Many getter functions are defined in the class for utility. Additionally, a [`periodic`](../../../src/main/java/frc/robot/subsystems/Swerve.java#L127) function for the subsystem is used to update the odometry and pose estimator, the `turretToSpeakerError` variable, and information about each swerve module printed to the SmartDashboard.

The most useful methods are:
- [`drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop)`](../../../src/main/java/frc/robot/subsystems/Swerve.java#L140) directly sets the state of each module based on the given `translation` vector and `rotation`, changing the perspective of the vector from the current robot heading to facing the opposite alliance wall if `fieldRelative` is `true`, and ignoring PID velocity feedback if `isOpenLoop` is `true`.
- [`driveTurret(Translation2d translation, boolean fieldRelative, TurretTargets targetType)`](../../../src/main/java/frc/robot/subsystems/Swerve.java#L156) directly sets the state of each module based on the given `translation` vector, assuming open loop and ignoring PID velocity feedback, changing the perspective of the vector from the current robot heading to facing the opposite alliance wall if `fieldRelative` is `true`, and setting the target to the specified `targetType`, to the speaker or the near alliance wall.
- [`updateFusedPose()`](../../../src/main/java/frc/robot/subsystems/Swerve.java#L304) updates the pose estimator with the latest odometry data, gyro angle, and Limelight data, implementing MegaTag and MegaTag2.