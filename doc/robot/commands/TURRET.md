# Turret Documentation

The Turret command is used to rotationally align to the speaker while still being able to laterally move the robot. 

## Execute

The Turret command execute method overrides the default Teleop Swerve command and manipulates the original rotational velocity to be sent to the drivetrain to a rotational velocity to move the robot to align to the speaker. This velocity is determined based on the robot's lateral rotational difference to the speaker applied to a PID controller.

## End

The Turret command end method stops the drivetrain and implicitly resets the Teleop Swerve command to run.
