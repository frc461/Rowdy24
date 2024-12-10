# Auto Align Command

The Auto Align command automatically aligns the angler to the correct level based on how far away the robot is from the speaker. When the command ends, the angler is reset back to its lower limit. 

## Execute

The Auto Align command execute method directly sets the angler target to the encoder value that corresponds to the correct angle to aim at the speaker to shoot a note. This is achieved through a constant bi-function that relates encoder value to physical angle of the angler incline, as well as a constant function utilizing data from the Limelight to deduce the distance to the speaker to calculate the correct aiming angle.

## End

The Auto Align command end method resets the angler to its lower limit.