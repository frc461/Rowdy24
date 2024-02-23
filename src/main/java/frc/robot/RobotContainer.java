package frc.robot;
import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private final Elevator elevator = new Elevator();
    private final Limelight limelight = new Limelight();
    private final IntakeCarriage intakeCarriage = new IntakeCarriage();
    private final Shooter shooter = new Shooter();
    private final Angler angler = new Angler();


    /* Controllers */
    public final static CommandXboxController driverXbox = new CommandXboxController(0);
    public final static CommandXboxController opXbox = new CommandXboxController(1);

    /* Variables */
    private boolean idleMode = false; // Disables/enables automatic subsystem functions (e.g. auto-intake)
    private final SendableChooser<Command> chooser;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */

    public RobotContainer() {
        // Register autonomous commands
        registerAutoCommands();

        // Configure autonomous path chooser
        chooser = AutoBuilder.buildAutoChooser("default");
        SmartDashboard.putData("Auto Choices", chooser);

        // Register default commands/controller axis commands
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

        // Configure controller button bindings
        configureButtonBindings();
    }

    public void registerAutoCommands() {
        NamedCommands.registerCommand(
                "intakeNote",
                new IntakeCarriageCommand(
                        intakeCarriage,
                        0.9,
                        1,
                        idleMode
                ).until(intakeCarriage::noteInSystem)
        );

        NamedCommands.registerCommand(
                "autoShoot",
                new ParallelCommandGroup(
                        new InstantCommand(() -> angler.setAlignedAngle(
                                limelight.getRX(),
                                limelight.getRZ(),
                                limelight.tagExists()
                        )),
                        new RevUpShooterCommand(
                                shooter,
                                limelight,
                                idleMode
                        ).until(() -> !intakeCarriage.noteInSystem()),
                        new WaitUntilCommand(shooter::minimalError).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1,
                                idleMode
                        ).until(() -> !intakeCarriage.noteInSystem()))
                )
        );

        NamedCommands.registerCommand(
                "alignAngler",
                new InstantCommand(() -> angler.setAlignedAngle(
                        limelight.getRX(),
                        limelight.getRZ(),
                        limelight.tagExists()
                ))
        );

        NamedCommands.registerCommand(
                "shoot",
                new InstantCommand(() -> shooter.overrideShooterSpeed(1.0))
        );

        NamedCommands.registerCommand(
                "carriageShoot",
                new IntakeCarriageCommand(
                        intakeCarriage,
                        0,
                        1,
                        idleMode
                ).until(() -> !intakeCarriage.noteInSystem())
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    private void configureButtonBindings() {
        /* Driver Buttons */

        /* Zero Gyro */
        driverXbox.y().onTrue(new InstantCommand(swerve::zeroGyro));

        /* Toggle idle subsystems */
        opXbox.povUp().onTrue(new SequentialCommandGroup(
                new InstantCommand(() -> idleMode = !idleMode),
                new ParallelCommandGroup(
                        (new InstantCommand(
                                () -> intakeCarriage.setIntakeIdle(idleMode),
                                intakeCarriage
                        )).withInterruptBehavior(
                                Command.InterruptionBehavior.kCancelSelf
                        ),
                        (new InstantCommand(
                                () -> shooter.setShooterIdle(idleMode),
                                shooter
                        )).withInterruptBehavior(
                                Command.InterruptionBehavior.kCancelSelf
                        )
                )
        ));

        /* Limelight Turret */
        driverXbox.leftBumper().whileTrue(
        new LimelightTurretCommand(
                limelight,
                swerve,
                () -> -driverXbox.getLeftY(), // Ordinate Translation
                () -> -driverXbox.getLeftX(), // Coordinate Translation
                driverXbox.b()) // Robot-centric trigger
        );

        /* Intake Note */
        opXbox.rightBumper().whileTrue(new IntakeCarriageCommand(
                intakeCarriage,
                0.9,
                1,
                idleMode
        ).until(intakeCarriage::noteInSystem));

        /* Intake Override */
        opXbox.b().whileTrue(new IntakeCarriageCommand(intakeCarriage, 0.9, 1, idleMode));

        /* Outtake Note */
        opXbox.leftBumper().whileTrue(new IntakeCarriageCommand(intakeCarriage, -0.9, -1, idleMode));

        /* Auto-align-and-shoot (deadband defaults to 0.5) */
        opXbox.rightTrigger().onTrue(
                new InstantCommand(() -> angler.setAlignedAngle(
                        limelight.getRX(),
                        limelight.getRZ(),
                        limelight.tagExists()
                ))
        );

        opXbox.rightTrigger().whileTrue(
                new ParallelCommandGroup(
                        new RevUpShooterCommand(shooter, limelight, idleMode),
                        new WaitUntilCommand(shooter::minimalError).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1,
                                idleMode
                        ))
                )
        );

        /* Override Shooter (deadband defaults to 0.5) */
        opXbox.leftTrigger().whileTrue(new RevUpShooterCommand(
                shooter, limelight, idleMode
                )
        );

        /* Climb */
        opXbox.povDown().whileTrue(new InstantCommand(elevator::climb));

        /* Stow Elevator Preset */
        driverXbox.rightBumper().onTrue(
            new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW))
        );

        /* Stow Elevator Preset */
        opXbox.a().onTrue(
            new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW))
        );

        /* Amp Elevator Preset */
        opXbox.y().onTrue(new InstantCommand(() ->
            elevator.setHeight(Constants.Elevator.ELEVATOR_AMP)
        ));

        /* Auto Angle Align */
        opXbox.x().onTrue(new InstantCommand(() -> angler.setAlignedAngle(
                limelight.getRX(),
                limelight.getRZ(),
                limelight.tagExists()
        )));
    }

    public void printValues() {
        // robot position
        SmartDashboard.putString("Robot Pose2d", swerve.getPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot Yaw", swerve.getYaw());
        SmartDashboard.putNumber("Robot Pitch", swerve.getPitch());
        SmartDashboard.putNumber("Robot Roll", swerve.getRoll());
        SmartDashboard.putBoolean("Beam Brake carriage", intakeCarriage.getCarriageBeamBroken());
        SmartDashboard.putBoolean("Beam Brake amp", intakeCarriage.getAmpBeamBroken());
        SmartDashboard.putBoolean("Beam Brake shooter", intakeCarriage.getShooterBeamBroken());
        SmartDashboard.putBoolean("note in system", intakeCarriage.noteInSystem());

        // elevator debug
       SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
       SmartDashboard.putNumber("Elevator Target", elevator.getTarget());
       SmartDashboard.putNumber("Elevator Power", elevator.elevatorVelocity());
       SmartDashboard.putBoolean("Elevator Limit Triggered?", elevator.elevatorSwitchTriggered());

        // limelight debug
        SmartDashboard.putNumber("Limelight Updates", limelight.getUpdates());
        SmartDashboard.putNumber("Limelight Yaw", limelight.getYaw());
        SmartDashboard.putNumber("Limelight Pitch", limelight.getPitch());
        SmartDashboard.putNumber("Limelight Roll", limelight.getRoll());
        SmartDashboard.putNumber("Limelight X", limelight.getRX());
        SmartDashboard.putNumber("Limelight Y", limelight.getRY());
        SmartDashboard.putNumber("Limelight Z", limelight.getRZ());

        // shooter debug
        SmartDashboard.putBoolean("Shooter Min Error", shooter.minimalError());
        SmartDashboard.putNumber("Shooter Left", shooter.getBottomShooterSpeed());
        SmartDashboard.putNumber("Shooter Right", shooter.getTopShooterSpeed());
        SmartDashboard.putNumber("Shooter error", shooter.getError());

        // angler debug
        SmartDashboard.putNumber("Angler encoder", angler.getPosition());
        SmartDashboard.putNumber("Angler error", angler.getError());
        SmartDashboard.putBoolean("Angler bottom triggered", angler.lowerSwitchTriggered());
        SmartDashboard.putBoolean("Angler top triggered", angler.upperSwitchTriggered());
    }

    /**
     * Passes the autonomous command retrieved from the chooser to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
