# Robot Container Class Documentation

The [`RobotContainer`](../../src/main/java/frc/robot/RobotContainer.java) class is the executive class in the robot's software implementation. The `RobotContainer` class contains all the subsystem instances of the robot and configures triggers and autonomous commands to directly use the Command classes.

## Fields

The `RobotContainer` class contains fields that are instances of each subsystem. These fields are used throughout the container to pass into commands that are called to provide them the represented system to manipulate/control.

There are also two Xbox controller fields that represent the driver and operator controllers, respectively. Each container allows for up to fourteen primitive bindings (i.e., one button/controller action).

There are a few miscellaneous [fields](../../src/main/java/frc/robot/RobotContainer.java#L95): 
- `readyToShoot` - boolean indicator which returns `true` if the robot's current angler and shooter position are in a shooting-ready state, i.e., the angler and shooter are ready to release a note.
- `constantShooter` - boolean that represents whether the default shooter command should be empty (turning it `false` would remove the default shooter command until it turned back to `true`) - this higher-level field helps with the sound disruption of the robot when it is near the speaker. (Note: This field is technically not where it is supposed to be; it should be in the `Shooter` subsystem class.)
- `chooser` - a WPILib-provided [`SendableChooser`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/smartdashboard/SendableChooser.html) class type variable that allows for the selection of autonomous commands from the [`SmartDashboard`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/smartdashboard/SmartDashboard.html) (more explained in the [Smart Dashboard](#smart-dashboard-data) Data section).

## Constructor

The Constructor of the `RobotContainer` class sets the default command of the swerve, angler, elevator, and shooter subsystems, and runs every configuration method defined in the class (which are listed and explained in later sections).

## Register Auto Commands

The autonomous command that PathPlanner generates is made up of the commands defined in this method coupled with drivetrain commands generated by PathPlanner (which is configured in the [`Swerve`](../../src/main/java/frc/robot/subsystems/Swerve.java#L71) class). Commands that are defined in this method are combinations of the Command classes that are defined in the commands directory, automated sequentially or in parallel.

## Configure Button Bindings

Button bindings are represented by the WPILib-provided [`Trigger`](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj2/command/button/Trigger.html) class. Like autonomous commands, some commands that are defined in this method are combinations of the Command classes that are defined in the commands directory, automated in sequentially or in parallel. Comments below each controller declaration (where all final fields are declared and initialized) specifically indicate what command that each binding triggers.

## Smart Dashboard Data

Data from each subsystem can be accessed by each of their getter functions. Thus, calling those functions and passing them through `SmartDashboard` class methods allows for the driver station to show that data. Additionally, the `chooser` field uses the SmartDashboard to allow input from the driver or operator with which auto to run. PathPlanner, a vendor dependency, registers the chosen option's corresponding pre-programmed autonomous command using the PathPlanner client.
