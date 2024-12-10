# Rev Up Shooter Command

The [`RevUpShooterCommand`](../../../src/main/java/frc/robot/commands/RevUpShooterCommand.java) class activates the shooter mechanism. The command is triggered by a controller or called in the autonomous command.

## Execute

The execute method of this command turns on the shooter at a certain speed based on the robot's distance from the speaker. The further away the robot is, the more power that is sent to the shooter to maximize spin and aerodynamics of the note in flight.

## End

The end method of this command sets the shooter speed/power output back to zero. 
