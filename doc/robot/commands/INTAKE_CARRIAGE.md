# Intake Carriage Command

The Intake Carriage command is used to turn on both the intake and carriage during teleop and auto. If the robot already has a note, it stops running the intake to prevent picking up more than one note. When the command ends the intake and carriage speeds are both set to 0.

## Initialize

The Intake Carriage command initialize method sets the intake and carriage speeds initially to a speed that is passed depending on the trigger (left/right controller trigger causes intake/outtake, respectively).

## Execute

The Intake Carriage command execute method updates the speed based on whether a note is detected within the system, i.e. it stops automatically when a note is detected.

## End

The Intake Carriage command end method resets all speeds to zero.