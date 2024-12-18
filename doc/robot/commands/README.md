# Commands Documentation

Commands are actions that the robot can perform, represented by the [`Command`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html) class in command-based WPILib. Commands are run by a Command Scheduler (represented by the [`CommandScheduler`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/CommandScheduler.html) class), which runs all commands registered onto it periodically (see the [`robotPeriodic()`](../../src/main/java/frc/robot/Robot.java#L49) function in the `Robot` class). To run a specific command it has to be registered onto the Command Scheduler. They are run until deregistered, either when their end condition is met or through interruption (any `Command` can be canceled with the [`cancel()`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#cancel()) method). Commands are usually directly implemented/registered in the [`RobotContainer`](../../../src/main/java/frc/robot/RobotContainer.java) class.

## Command-Structure Robot Implementation

Implementation of an FRC robot using WPILib is achieved through two possible methods; time-based and command-based. Time-based uses time to control the actions of the robot while command-based uses a scheduler for configured commands to control the actions of the robot. The command-based implementation allows for modulation and duplication of robot tasks, allowing for better task automation. Commands contain subsystem "requirements," allowing for command organization and a lower-level implementation of subsystems to be utilized in commands, which are then compiled and utilized directly in an overall container file; Rowdy24's container file is the [`RobotContainer`](../../../src/main/java/frc/robot/subsystems/Swerve.java) class.

## Events of a Command

Commands specify what the command will do in each of its possible states. This is done by overriding the `initialize()`, `execute()`, and `end()` methods. Additionally, a command must be able to tell the scheduler when (if ever) it has finished execution - this is done by overriding the `isFinished()` method. All of these methods are defaulted to reduce clutter in user code: `initialize()`, `execute()`, and `end()` are defaulted to simply do nothing, while `isFinished()` is defaulted to return false (resulting in a command that never finishes naturally, and will run until interrupted).

### Initialization

The [`initialize()`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#initialize()) method marks the command start, and is called exactly once per time a command is scheduled. The initialize() method should be used to place the command in a known starting state for execution. Command objects may be reused and scheduled multiple times, so any state or resources needed for the command’s functionality should be initialized or opened in initialize (which will be called at the start of each use) rather than the constructor (which is invoked only once on object allocation). It is also useful for performing tasks that only need to be performed once per time scheduled, such as setting motors to run at a constant speed or setting the state of a solenoid actuator.

### Execution

The [`execute()`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#execute()) method is called repeatedly while the command is scheduled; this is when the scheduler’s run() method is called (this is generally done in the main robot periodic method, which runs every 20ms by default). The execute block should be used for any task that needs to be done continually while the command is scheduled, such as updating motor outputs to match joystick inputs, or using the output of a control loop.

### Ending

The [`end(boolean interrupted)`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#end(boolean)) method is called once when the command ends, whether it finishes normally (i.e. `isFinished()` returned true) or it was interrupted (either by another command or by being explicitly canceled). The method argument specifies the manner in which the command ended; users can use this to differentiate the behavior of their command end accordingly. The end block should be used to “wrap up” command state in a neat way, such as setting motors back to zero or reverting a solenoid actuator to a “default” state. Any state or resources initialized in `initialize()` should be closed in `end()`.

### Specifying end conditions

The [`isFinished()`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#end(boolean)) method is called repeatedly while the command is scheduled, whenever the scheduler’s run() method is called. As soon as it returns true, the command’s `end()` method is called and it ends. The `isFinished()` method is called after the `execute()` method, so the command will execute once on the same iteration that it ends.

## Requirements

Think about what would happen if two commands that contradicted each other (e.g., ran opposite methods of the same subsystem) were registered at the same time. As a preventative measure, a command is associated with a set of subsystems that it requires to run. The Command Scheduler then automatically checks if any two commands require the same subsystem, and based on criteria from each command (every command has a [`withInterruptBehavior(InterruptionBehavior behavior)`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Command.html#withInterruptBehavior(edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior)) method to set its interruption priority), decide which one to safely deregister.

## Default Commands

Default commands are an attribute of a subsystem. It is a command that requires (only) that subsystem that is registered to the Command Scheduler if there aren't any other commands requiring that subsystem that are already registered. It is an optional attribute, so not every subsystem as a default command. However, it is important for manual controls, like driving the robot with joysticks, as it needs to be always registered. Every subsystem class has the [`setDefaultCommand(Command defaultCommand)`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/Subsystem.html#setDefaultCommand(edu.wpi.first.wpilibj2.command.Command)) to set its own default command. In Rowdy24 implementation, default commands for each subsystem are defined in the [`RobotContainer`](../../../src/main/java/frc/robot/RobotContainer.java#L117) class constructor.

## Rowdy24 Command Implementation

This section of the documentation will focus on the commands created for the robot to perform specific actions. Each page will include an overview and events of their respective command(s):
- [AutoAlign](AUTO_ALIGN) - command to align the angler with the speaker
- [IntakeCarriage](INTAKE_CARRIAGE) - command to run the intake and the carriage
- [RevUpShooter](REV_UP_SHOOTER) - command to speed up the shooter
- [Teleop Commands](TELEOP_COMMANDS) - commands to manually control the robot, triggered by joysticks and axes on a controller
- [Turret](TURRET.md) - command to align the robot rotationally with either the speaker to shoot notes or the near alliance wall to shuttle notes