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
    private final IntakeCarriage intakeCarriage = new IntakeCarriage();
    private final Shooter shooter = new Shooter();
    private final Angler angler = new Angler();

    public final static CommandXboxController driverXbox = new CommandXboxController(0);
    /* Currently Allocated For Driver:
     * POV buttons / Dpad:
     * Up
     * Down
     * Left
     * Right
     * 
     * Triggers:
     * Left:
     * Right:
     * 
     * Joysticks:
     * Left: Translation
     * Right: Rotation
     * 
     * Bumpers:
     * Left: auto-align angler and drivetrain to april tag
     * Right: stow elevator
     * 
     * Buttons: 
     * A: 
     * B: 
     * X: 
     * Y: rezero gyro
     */  

    public final static CommandXboxController opXbox = new CommandXboxController(1);
    /* Currently Allocated For Operator:
     * POV buttons / Dpad:
     * Up - toggle auto subsystems
     * Down - auto climb
     * Left - toggle clamp
     * Right - angler layup setpoint
     * 
     * Triggers:
     * Left: Manual shooter rev
     * Right: Auto shoot
     * 
     * Joysticks:
     * Left: Manual angler
     * Right: Manual elevator
     * 
     * Bumpers:
     * Left: Outtake note 
     * Right: Intake note (with beam breaks)
     * 
     * Buttons: 
     * A: Stow elevator
     * B: Force intake
     * X: Auto angler align
     * Y: Elevator amp preset
     */


    /* Variables */
    private boolean idleMode = false; // Disables/enables automatic subsystem functions (e.g. auto-outtake)
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
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0.9,
                                1,
                                idleMode
                        ).until(() -> !intakeCarriage.getAmpBeamBroken()))
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                -0.9,
                                -1,
                                idleMode
                        ).until(intakeCarriage::getAmpBeamBroken))
        );

        NamedCommands.registerCommand(
                "waitUntilIntakeNote",
                new WaitUntilCommand(intakeCarriage::noteInSystem).andThen(new WaitCommand(0.2))
        );

        NamedCommands.registerCommand(
                "autoShoot",
                new ParallelCommandGroup(
                        new InstantCommand(() -> angler.setAlignedAngle(
                                Limelight.getRX(),
                                Limelight.getRZ(),
                                Limelight.tagExists()
                        ), angler),
                        new RevUpShooterCommand(
                                shooter,
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
                        Limelight.getRX(),
                        Limelight.getRZ(),
                        Limelight.tagExists()
                ), angler)
        );

        NamedCommands.registerCommand(
                "overrideShoot",
                new InstantCommand(() -> shooter.overrideShooterSpeed(1.0), shooter)
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
        /* Zero Gyro */
        driverXbox.y().onTrue(new InstantCommand(swerve::zeroGyro));

        /* Toggle idle subsystems */
        opXbox.povUp().onTrue(
                new InstantCommand(() -> idleMode = !idleMode)
                        .andThen(new ParallelCommandGroup(
                        new InstantCommand(
                                () -> intakeCarriage.setIntakeIdle(idleMode),
                                intakeCarriage
                        ),
                        new InstantCommand(
                                () -> shooter.setShooterIdle(idleMode),
                                shooter
                        )
                ))
        );

        /* Limelight Turret */
        driverXbox.leftBumper().whileTrue(
                new LimelightTurretCommand(
                        swerve,
                        () -> -driverXbox.getLeftY(), // Ordinate Translation
                        () -> -driverXbox.getLeftX(), // Coordinate Translation
                        driverXbox.b() // Robot-centric trigger
                )
        );

        /* Cheeky Driver Auto-align (deadband defaults to 0.5) */
        driverXbox.leftBumper().whileTrue(
                new InstantCommand(() -> angler.setAlignedAngle(
                        Limelight.getRX(),
                        Limelight.getRZ(),
                        Limelight.tagExists()
                ), angler)
        );
        
        //driverXbox.povUp().onTrue(new InstantCommand(() -> Constants.Angler.ANGLER_TRIM)));


        /* Intake Note */
        opXbox.rightBumper().whileTrue(new IntakeCarriageCommand(
                intakeCarriage,
                0.9,
                1,
                idleMode
        ).until(intakeCarriage::noteInSystem)
                .andThen(new IntakeCarriageCommand(
                        intakeCarriage,
                        0.9,
                        1,
                        idleMode
                ).until(() -> !intakeCarriage.getAmpBeamBroken()))
                .andThen(new IntakeCarriageCommand(
                        intakeCarriage,
                        -0.9,
                        -1,
                        idleMode
                ).until(intakeCarriage::getAmpBeamBroken)));

        /* Intake Override */
        opXbox.b().whileTrue(new IntakeCarriageCommand(intakeCarriage, 0.9, 1, idleMode, true));

        /* Outtake Note */
        opXbox.leftBumper().whileTrue(
                new IntakeCarriageCommand(intakeCarriage, -0.9, -1, idleMode)
        );

        /* Auto-align for auto-shoot (deadband defaults to 0.5) */
        opXbox.rightTrigger().onTrue(
                new InstantCommand(() -> angler.setAlignedAngle(
                        Limelight.getRX(),
                        Limelight.getRZ(),
                        Limelight.tagExists()
                ), angler).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        );

        /* Rev up shooter and run carriage when its up to speed */
        opXbox.rightTrigger().whileTrue(
                new ParallelCommandGroup(
                        new RevUpShooterCommand(shooter, idleMode),
                        new WaitUntilCommand(shooter::minimalError).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1,
                                idleMode
                        ))
                ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        );

        /* Auto-align (deadband defaults to 0.5) */
        opXbox.x().onTrue(
                new InstantCommand(() -> angler.setAlignedAngle(
                        Limelight.getRX(),
                        Limelight.getRZ(),
                        Limelight.tagExists()
                ), angler)
        );

        /* Override Shooter (deadband defaults to 0.5) */
        opXbox.leftTrigger().whileTrue(new RevUpShooterCommand(
                shooter, idleMode
                )
        );

        /* Climb */
        opXbox.povDown().whileTrue(
                new ClimbCommand(elevator).until(elevator::elevatorSwitchTriggered)
                        .andThen(new InstantCommand(() -> elevator.climb(true)))
                        .andThen(new WaitCommand(0.75))
                        .andThen(new InstantCommand(elevator::stopElevator))
        );

        /* Toggle clamp */
        opXbox.povLeft().onTrue(new InstantCommand(elevator::toggleClamp));

        /* Stow Elevator Preset */
        driverXbox.rightBumper().onTrue(
                new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW), elevator)
        );

        /* Stow Elevator Preset */
        opXbox.a().onTrue(
                new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW), elevator)
        );

        /* Amp Elevator Preset */
        opXbox.y().onTrue(
                new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_AMP), elevator),
                        new IntakeCarriageCommand(intakeCarriage, 0.1, 0.1, idleMode).until(elevator::elevatorNearTarget)
                )
        );

        /* Angler layup setpoint (Limelight failsafe) */
        opXbox.povRight().onTrue(new InstantCommand(() -> angler.setAngle(Constants.Angler.ANGLER_LAYUP_POSITION)));
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
       SmartDashboard.putBoolean("Servo Limit Triggered?", elevator.servoSwitchTriggered());
       SmartDashboard.putNumber("Elevator Clamp Pos", elevator.getClampPosition());

        // limelight debug
        SmartDashboard.putNumber("Limelight Updates", Limelight.getUpdates());
        SmartDashboard.putNumber("Limelight Yaw", Limelight.getYaw());
        SmartDashboard.putNumber("Limelight Pitch", Limelight.getPitch());
        SmartDashboard.putNumber("Limelight Roll", Limelight.getRoll());
        SmartDashboard.putNumber("Limelight X", Limelight.getRX());
        SmartDashboard.putNumber("Limelight Y", Limelight.getRY());
        SmartDashboard.putNumber("Limelight Z", Limelight.getRZ());
        SmartDashboard.putNumber("Limelight dist", Math.hypot(Limelight.getRZ(), Limelight.getRX()));

        // shooter debug
        SmartDashboard.putBoolean("Shooter Min Error", shooter.minimalError());
        SmartDashboard.putNumber("Shooter Left", shooter.getBottomShooterSpeed());
        SmartDashboard.putNumber("Shooter Right", shooter.getTopShooterSpeed());
        SmartDashboard.putNumber("Shooter error", shooter.getError());

        // angler debug
        SmartDashboard.putNumber("Angler encoder", angler.getPosition());
        SmartDashboard.putNumber("Angler error", angler.getError());
        SmartDashboard.putBoolean("Angler bottom triggered", angler.lowerSwitchTriggered());
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
