# Teleop Commands Documentation

## Overview

The following commands are used in teleop, i.e. are only triggered by a controller to manually control the robot, specifically by joysticks, axes, or buttons on a controller. The implication of a continuous axis like a joystick is that there is no default state for the subsystems that these commands control. Since there is no state to return to once the command ends, there is only an `execute()` function for most of these commands (exceptions are noted below). Due to the manual and continuous nature of these commands, drivers and operators seldom use them.

## Teleop Angler

The [`TeleopAnglerCommand`](../../../src/main/java/frc/robot/commands/TeleopAnglerCommand.java) class is used to manually move the angler up and down with the joystick in teleop. 

## Teleop Elevator

The [`TeleopElevatorCommand`](../../../src/main/java/frc/robot/commands/TeleopElevatorCommand.java) class is used to manually move the elevator up and down with the joystick in teleop.

## Teleop Shooter

The [`TeleopShooterCommand`](../../../src/main/java/frc/robot/commands/TeleopShooterCommand.java) class is used to constantly run the shooter in teleop. Initially (the `initialize()` function), the shooter is set to a low speed. If the robot is within a certain distance of the speaker, the shooter runs at full speed (based on the robot's distance from the speaker, similarly to the Rev Up Shooter command). Otherwise, it returns to the low, idle speed.

## Teleop Swerve

The [`TeleopSwerveCommand`](../../../src/main/java/frc/robot/commands/TeleopSwerveCommand.java) class is used to manually move the robot around with the joysticks in teleop.
