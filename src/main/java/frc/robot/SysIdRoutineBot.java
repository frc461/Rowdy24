// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopAnglerCommand;
import frc.robot.commands.TeleopElevatorCommand;
import frc.robot.commands.TeleopSwerveCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
* This class is where the bulk of the robot should be declared. Since Command-based is a
* "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
* periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
* subsystems, commands, and button mappings) should be declared here.
*/
public class SysIdRoutineBot {
    // The robot's subsystems
    private final Angler angler = new Angler();
    private final Elevator elevator = new Elevator();
    private final Shooter shooter = new Shooter();
    private final Swerve swerve = new Swerve();

    // The driver's controller

    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController opXbox = new CommandXboxController(1);

    /**
    * Use this method to define bindings between conditions and commands. These are useful for
    * automating robot behaviors based on button and sensor input.
    *
    * <p>Should be called during {@link Robot#robotInit()}.
    *
    * <p>Event binding methods are available on the {@link Trigger} class.
    */
    public void configureBindings() {
         // Control the drive with awesome swerve drive
        swerve.setDefaultCommand(
                new TeleopSwerveCommand(
                        swerve,
                        () -> -driverXbox.getLeftY(), // Ordinate Translation
                        () -> -driverXbox.getLeftX(), // Coordinate Translation
                        () -> -driverXbox.getRightX(), // Rotation
                        driverXbox.b() // Robot-centric trigger
                )
        );

        angler.setDefaultCommand(
                new TeleopAnglerCommand(
                        angler,
                        () -> -opXbox.getLeftY()
                )
        );

        elevator.setDefaultCommand(
                new TeleopElevatorCommand(
                        elevator,
                        () -> -opXbox.getRightY()
                )
        );

        // Bind full set of SysId routine tests to buttons; a complete routine should run each of these
        // once.
        // Using bumpers as a modifier and combining it with the buttons so that we can have both sets
        // of bindings at once
        // Swerve bindings
         driverXbox
                 .a()
                 .and(driverXbox.rightBumper())
                 .whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
         driverXbox
                 .b()
                 .and(driverXbox.rightBumper())
                 .whileTrue(swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
         driverXbox
                 .x()
                 .and(driverXbox.rightBumper())
                 .whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
         driverXbox
                 .y()
                 .and(driverXbox.rightBumper())
                 .whileTrue(swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

         // Angler bindings
         driverXbox
                 .a()
                 .and(driverXbox.leftBumper())
                 .whileTrue(angler.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
         driverXbox
                 .b()
                 .and(driverXbox.leftBumper())
                 .whileTrue(angler.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
         driverXbox
                 .x()
                 .and(driverXbox.leftBumper())
                 .whileTrue(angler.sysIdDynamic(SysIdRoutine.Direction.kForward));
         driverXbox
                 .y()
                 .and(driverXbox.leftBumper())
                 .whileTrue(angler.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Elevator bindings
        driverXbox
                .a()
                .and(driverXbox.povUp())
                .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverXbox
                .b()
                .and(driverXbox.povUp())
                .whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driverXbox
                .x()
                .and(driverXbox.povUp())
                .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverXbox
                .y()
                .and(driverXbox.povUp())
                .whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse));

         // Shooter bindings
        driverXbox
                .a()
                .and(driverXbox.povDown())
                .whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        driverXbox
                .b()
                .and(driverXbox.povDown())
                .whileTrue(shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        driverXbox
                .x()
                .and(driverXbox.povDown())
                .whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverXbox
                .y()
                .and(driverXbox.povDown())
                .whileTrue(shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    /**
    * Use this to define the command that runs during autonomous.
    *
    * <p>Scheduled during {@link Robot#autonomousInit()}.
    */
    public Command getAutonomousCommand() {
         // Do nothing
         return swerve.run(() -> {});
    }
}
