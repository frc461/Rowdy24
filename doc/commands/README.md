# Commands Documentation

Commands represent actions that the robot can take. Commands run when scheduled, until they are interrupted or their end condition is met. Commands are represented in the command-based library by the [`Command`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html) class. 

## The Structure of a Command
Commands specify what the command will do in each of its possible states. This is done by overriding the `initialize()`, `execute()`, and `end()` methods. Additionally, a command must be able to tell the scheduler when (if ever) it has finished execution - this is done by overriding the `isFinished()` method. All of these methods are defaulted to reduce clutter in user code: `initialize()`, `execute()`, and `end()` are defaulted to simply do nothing, while `isFinished()` is defaulted to return false (resulting in a command that never finishes naturally, and will run until interrupted).

### Initialization
The [`initialize()`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#initialize()) method  marks the command start, and is called exactly once per time a command is scheduled. The initialize() method should be used to place the command in a known starting state for execution. Command objects may be reused and scheduled multiple times, so any state or resources needed for the command’s functionality should be initialized or opened in initialize (which will be called at the start of each use) rather than the constructor (which is invoked only once on object allocation). It is also useful for performing tasks that only need to be performed once per time scheduled, such as setting motors to run at a constant speed or setting the state of a solenoid actuator.

### Execution
The [`execute()`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#execute()) method is called repeatedly while the command is scheduled; this is when the scheduler’s run() method is called (this is generally done in the main robot periodic method, which runs every 20ms by default). The execute block should be used for any task that needs to be done continually while the command is scheduled, such as updating motor outputs to match joystick inputs, or using the output of a control loop.

### Ending
The [`end(bool interrupted)`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#end(boolean)) method is called once when the command ends, whether it finishes normally (i.e. `isFinished()` returned true) or it was interrupted (either by another command or by being explicitly canceled). The method argument specifies the manner in which the command ended; users can use this to differentiate the behavior of their command end accordingly. The end block should be used to “wrap up” command state in a neat way, such as setting motors back to zero or reverting a solenoid actuator to a “default” state. Any state or resources initialized in `initialize()` should be closed in `end()`.

### Specifying end conditions
The [`isFinished()`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#end(boolean)) method is called repeatedly while the command is scheduled, whenever the scheduler’s run() method is called. As soon as it returns true, the command’s `end()` method is called and it ends. The `isFinished()` method is called after the `execute()` method, so the command will execute once on the same iteration that it ends.

This section of the documentation will focus on the commands that we have created for specfic actions we want the robot to do. They include:
- [AutoAlign](AUTOALIGN.md) - command to align the angler with the speaker.
- [IntakeCarriage](INTAKECARRIAGE.md) - command to run the intake and the carriage.
- [RevUpShooter](REVUPSHOOTER.md) - command to speed up the shooter.
- [TeleopAngler](TELEOPANGLER.md) - command to manually move the angler.
- [TeleopElevator](TELEOPELEVATOR.md) - command to manually move the elevator.
- [TeleopShooter](TELEOPSHOOTER.md) - command to constantly run the shooter.
- [TeleopSwerve](TELEOPSWERVE.md) - command to manually move the robot around.
- [Turret](TURRET.md) - command to align the robot rotationally 
- [TurretTargets](TURRETTARGETS.md) - allows the robot to align to different places on the field. 