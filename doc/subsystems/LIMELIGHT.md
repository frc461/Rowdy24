# Limelight Documentation

Software implementation of the Limelight "subsystem" on the Rowdy24 robot. This class processes information collected from the Limelight camera. The function of the camera is to detect AprilTags (fiducial markers) on the field and infer the robot's distance vector from them. We use the camera model [Limelight 3G](https://limelightvision.io/collections/products/products/limelight-3g) on the robot. 

## Implementation

### Class Information

As the Limelight "subsystem" does not contain any physical motors or instances on the robot for physical contribution, the [`Limelight`](../../src/main/java/frc/robot/subsystems/Limelight.java) class is used in a static context for convenient reference to processed camera data in the rest of the software. As Limelight does not have a physical presence on the robot (other than the camera itself), it does not need to extend the [`SubsystemBase`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/SubsystemBase.html) class.

### Variables

This class has one variable, [`overrideTargetNow`](../../src/main/java/frc/robot/subsystems/Limelight.java#L14), that is set by named autonomous commands to override the holonomic rotation target in the autonomous game mode. The variable determines whether the default holonomic rotation target set by PathPlanner is overridden by the rotation target to the speaker, whose value is determined by a PID Controller and localization of the robot using the `Limelight` and `Swerve` class.

### Methods

There are many methods to retrieve the robot's position on the field by processing data streamed onto network tables by the Limelight. The [LimelightHelpers.java](../../src/main/java/frc/lib/util/LimelightHelpers.java) class is used to facilitate the extraction of data. 

The most useful methods are:
- [`tagExists()`](../../src/main/java/frc/robot/subsystems/Limelight.java#L36) - returns whether a tag is in the scope of the camera.
- [`getNearestTagDistance()`](../../src/main/java/frc/robot/subsystems/Limelight.java#L44) - returns the length of the vector from the robot to the nearest tag.
- [`getSpeakerTagPose()`](../../src/main/java/frc/robot/subsystems/Limelight.java#L48) - returns the coordinate of the speaker tag with respect to the robot's team as a vector pose from the origin.
- [`configureRobotPose()`](../../src/main/java/frc/robot/subsystems/Limelight.java#L54) - configures the location of the Limelight camera offset from the center of the robot, called in the [`Swerve`](../../src/main/java/frc/robot/subsystems/Swerve.java) class constructor