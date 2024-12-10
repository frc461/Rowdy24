# Turret Documentation

The [`TurretCommand`](../../../src/main/java/frc/robot/commands/TurretCommand.java) class is used to rotationally align to the speaker while still being able to laterally move the robot. 

## Execute

The execute method of this command overrides the default Teleop Swerve command and manipulates the original rotational velocity to be sent to the drivetrain to a rotational velocity to move the robot to align to the speaker. This velocity is determined based on the robot's lateral rotational difference to the speaker applied to a PID controller.

## End

The end method of this command stops the drivetrain and implicitly resets the Teleop Swerve command to run.
