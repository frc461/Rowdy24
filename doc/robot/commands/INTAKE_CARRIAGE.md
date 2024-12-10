# Intake Carriage Command

The [`IntakeCarriageCommand`](../../../src/main/java/frc/robot/commands/IntakeCarriageCommand.java) class turns on both the intake and carriage. The command is triggered by a controller during teleop or called in the autonomous command. Logic is incorporated to regulate intake and carriage power to prevent rule violations, e.g. picking up multiple notes.

## Initialize

The initialize method of this command sets the intake and carriage speeds initially to a speed that is passed depending on the trigger (left/right controller trigger causes intake/outtake, respectively).

## Execute

The execute method of this command updates the speed based on whether a note is detected within the system, i.e. it stops automatically when a note is detected, to prevent picking up more than one note.

## End

The end method of this command resets all speeds to zero.