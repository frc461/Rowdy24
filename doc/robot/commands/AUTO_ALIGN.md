# Auto Align Command

The Auto Align command aligns the angler to an accurate incline angle based on the robot's distance is from the speaker. This command is triggered by a controller or called in the autonomous command for shooting and shuttling notes. 

## Execute

The Auto Align command execute method directly sets the angler target to the encoder value that corresponds to the correct angle to aim at the speaker to shoot a note. This is achieved through a constant bi-function that relates encoder value to physical angle of the angler incline, as well as a constant function utilizing data from the Limelight to deduce the distance to the speaker to calculate the correct aiming angle.

## End

The Auto Align command end method resets the angler to its lower limit.