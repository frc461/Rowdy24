# Intake Carriage Command

The Intake Carriage command turns on both the intake and carriage. The command is triggered by a controller during teleop or called in the autonomous command. Logic is incorporated to regulate intake and carriage power to prevent rule violations, e.g. picking up multiple notes.

## Initialize

The Intake Carriage command initialize method sets the intake and carriage speeds initially to a speed that is passed depending on the trigger (left/right controller trigger causes intake/outtake, respectively).

## Execute

The Intake Carriage command execute method updates the speed based on whether a note is detected within the system, i.e. it stops automatically when a note is detected, to prevent picking up more than one note.

## End

The Intake Carriage command end method resets all speeds to zero.