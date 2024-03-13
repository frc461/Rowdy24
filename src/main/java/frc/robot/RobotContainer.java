package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.util.LimelightHelpers;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;

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
     * Up: Angler trim up by 0.1 encoder counts
     * Down: Angler trim down by 0.1 encoder counts
     * Left:
     * Right:
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
     * Left: auto-align drivetrain to april tag
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
     * Up - 
     * Down - auto climb
     * Left - toggle clamp
     * Right - auto outtake
     * 
     * Triggers:
     * Left: Subwoofer auto shoot
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
     * Y: Elevator amp auto score
     */


    /* Variables */
    private final BooleanSupplier readyToShoot = () -> shooter.nearTarget() && angler.anglerNearTarget();
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

        // TODO: Test constant shooter function
        shooter.setDefaultCommand(
                new RunCommand(() -> shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED), shooter)
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
                        1
                ).until(intakeCarriage::noteInShootingSystem)
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0.9,
                                1
                        ).until(() -> !intakeCarriage.getAmpBeamBroken()))
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                -0.9,
                                -1
                        ).until(intakeCarriage::getAmpBeamBroken))
        );

        NamedCommands.registerCommand(
                "autoShoot",
                new ParallelCommandGroup(
                        new InstantCommand(() -> angler.setEncoderVal(
                                Constants.Angler.AUTO_ANGLER_AIM_EQUATION.apply(
                                        swerve.getVectorToSpeakerTarget().getY(),
                                        swerve.getVectorToSpeakerTarget().getX()
                                )
                        ), angler),
                        new RevUpShooterCommand(shooter, swerve).until(() -> !intakeCarriage.noteInShootingSystem()),
                        new WaitUntilCommand(readyToShoot).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1
                        ).until(() -> !intakeCarriage.noteInShootingSystem()))
                )
        );

        NamedCommands.registerCommand(
                "autoShootWithoutAngler",
                new ParallelCommandGroup(
                        new RevUpShooterCommand(shooter, swerve).until(() -> !intakeCarriage.noteInShootingSystem()),
                        new WaitUntilCommand(readyToShoot).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1
                        ).until(() -> !intakeCarriage.noteInShootingSystem()))
                )
        );

        NamedCommands.registerCommand(
                "startShooter",
                new RevUpShooterCommand(shooter, swerve)
        );

        NamedCommands.registerCommand(
                "alignAngler",
                new AutoAlignCommand(angler, swerve)
        );

        NamedCommands.registerCommand(
                "intakeThenShoot",
                new WaitUntilCommand(readyToShoot).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0.9,
                                1
                        ).until(intakeCarriage::noteInShootingSystem)
                                .andThen(new IntakeCarriageCommand(
                                        intakeCarriage,
                                        0.9,
                                        1
                                ).until(() -> !intakeCarriage.noteInShootingSystem())))
        );

        NamedCommands.registerCommand(
                "enableTurret",
                new InstantCommand(() -> Limelight.overrideTargetNow = true)
        );

        NamedCommands.registerCommand(
                "disableTurret",
                new InstantCommand(() -> Limelight.overrideTargetNow = false)
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
        /* Driver Controls */

        /* Zero Gyro */
        driverXbox.y().onTrue(new InstantCommand(swerve::zeroGyro));

        /* Increment Trim */
        driverXbox.povUp().onTrue(new InstantCommand(() -> Constants.Angler.ANGLER_ENCODER_OFFSET += 0.1));

        /* Decrement Trim */
        driverXbox.povDown().onTrue(new InstantCommand(() -> Constants.Angler.ANGLER_ENCODER_OFFSET -= 0.1));

        /* Limelight Turret */
        driverXbox.leftBumper().whileTrue(
                new ParallelCommandGroup(
                        new LimelightTurretCommand(
                                swerve,
                                () -> -driverXbox.getLeftY(), // Ordinate Translation
                                () -> -driverXbox.getLeftX(), // Coordinate Translation
                                driverXbox.b() // Robot-centric trigger
                        )
                )
        );

        /* Stow Elevator Preset */
        driverXbox.rightBumper().onTrue(
                new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW), elevator)
        );

        /* Op Controls */

        /* Stow Elevator Preset */
        opXbox.a().onTrue(
                new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW), elevator)
        );

        /* Intake Override */
        opXbox.b().whileTrue(new IntakeCarriageCommand(intakeCarriage, 0.9, 1, true));

        /* Auto-align */
        opXbox.x().whileTrue(new AutoAlignCommand(angler, swerve));

        /* Amp Elevator Preset */
        opXbox.y().onTrue(
                new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_AMP), elevator)
        );

        /* Amp Shoot Preset */
        opXbox.y().onFalse(
                new IntakeCarriageCommand(intakeCarriage, 0, -1)
                        .until(() -> !intakeCarriage.noteInAmpSystem())
                        .andThen(new WaitCommand(0.25))
                        .andThen(new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW), elevator))
        );

        /* Toggle clamp */
        opXbox.povLeft().onTrue(new InstantCommand(elevator::toggleClamp));

        /* Auto-Climb */
        opXbox.povDown().whileTrue(
                new FunctionalCommand(
                        () -> {},
                        () -> elevator.climb(false),
                        isFinished -> elevator.climb(true),
                        elevator::elevatorSwitchTriggered,
                        elevator
                )
                        .andThen(new WaitCommand(0.25))
                        .andThen(new InstantCommand(elevator::stopElevator))
        );

        /* Auto-Outtake Note */
        opXbox.povRight().whileTrue(
                new ParallelCommandGroup(
                        new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_OUTTAKE)),
                        new WaitUntilCommand(() -> !elevator.nearTarget())
                                .andThen(new WaitUntilCommand(elevator::nearTarget))
                                .andThen(new IntakeCarriageCommand(
                                        intakeCarriage,
                                        -0.9,
                                        -1
                                )).until(() -> !intakeCarriage.noteInAmpSystem())
                                .andThen(new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW)))
                )
        );

        /* Override Outtake Note */
        opXbox.leftBumper().whileTrue(new IntakeCarriageCommand(intakeCarriage, -0.9, -1));

        /* Intake Note */
        opXbox.rightBumper().whileTrue(new IntakeCarriageCommand(
                intakeCarriage,
                0.9,
                1
                ).until(intakeCarriage::noteInShootingSystem)
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                0.5
                        ).until(() -> !intakeCarriage.getAmpBeamBroken()))
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                -0.5
                        ).until(intakeCarriage::getAmpBeamBroken))
        );

        opXbox.leftTrigger().whileTrue(
                new ParallelCommandGroup(
                        new FunctionalCommand(
                                () -> {},
                                () -> angler.setEncoderVal(Constants.Angler.ANGLER_LAYUP_PRESET),
                                isFinished -> angler.setEncoderVal(Constants.Angler.ANGLER_LOWER_LIMIT),
                                () -> false,
                                angler
                        ),
                        new RevUpShooterCommand(shooter, swerve),
                        new WaitUntilCommand(readyToShoot).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1
                        ))
                ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        );

        /* Aim then Shoot */
        opXbox.rightTrigger().whileTrue(
                new ParallelCommandGroup(
                        new AutoAlignCommand(angler, swerve).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming),
                        new RevUpShooterCommand(shooter, swerve),
                        new WaitUntilCommand(readyToShoot).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1
                        ))
                ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        );
    }

    public void printValues() {
        // Scheduled commands
        SmartDashboard.putData("Scheduled Commands", CommandScheduler.getInstance());

        // robot position
        SmartDashboard.putString("Robot Unfused Pose2d", swerve.getPose().getTranslation().toString());
        SmartDashboard.putString("Robot Fused Pose2d", swerve.getFusedPoseEstimator().getTranslation().toString());
        SmartDashboard.putString("Robot Fused Rotation", swerve.getFusedPoseEstimator().getRotation().toString());
        SmartDashboard.putNumber("Robot Yaw", swerve.getYaw());
        SmartDashboard.putNumber("Robot Pitch", swerve.getPitch());
        SmartDashboard.putNumber("Robot Roll", swerve.getRoll());
        SmartDashboard.putString("vector to speaker", swerve.getVectorToSpeakerTarget().toString());
        SmartDashboard.putNumber("angle to speaker", swerve.getAngleToSpeakerTarget());
        SmartDashboard.putBoolean("Turret near target", swerve.turretNearTarget());
        SmartDashboard.putData("Swerve Command", swerve);

        // intake-carriage debug
        SmartDashboard.putBoolean("Beam Brake carriage", intakeCarriage.getCarriageBeamBroken());
        SmartDashboard.putBoolean("Beam Brake amp", intakeCarriage.getAmpBeamBroken());
        SmartDashboard.putBoolean("Beam Brake shooter", intakeCarriage.getShooterBeamBroken());
        SmartDashboard.putBoolean("note in shooting system", intakeCarriage.noteInShootingSystem());
        SmartDashboard.putData("Intake-Carriage Cmd", intakeCarriage);

        // elevator debug
        SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
        SmartDashboard.putNumber("Elevator Target", elevator.getTarget());
        SmartDashboard.putNumber("Elevator Power", elevator.elevatorVelocity());
        SmartDashboard.putBoolean("Elevator Limit Triggered?", elevator.elevatorSwitchTriggered());
        SmartDashboard.putBoolean("Servo Limit Triggered?", elevator.servoSwitchTriggered());
        SmartDashboard.putNumber("Elevator Clamp Pos", elevator.getClampPosition());
        SmartDashboard.putBoolean("Clamped?", elevator.isClamped());
        SmartDashboard.putData("Elevator Cmd", elevator);

        // limelight debug
        SmartDashboard.putNumber("Limelight Yaw", Limelight.getTagYaw());
        SmartDashboard.putNumber("Limelight Pitch", Limelight.getTagPitch());
        SmartDashboard.putNumber("Limelight Roll", Limelight.getTagRoll());
        SmartDashboard.putNumber("Limelight X", Limelight.getTagRX());
        SmartDashboard.putNumber("Limelight Y", Limelight.getTagRY());
        SmartDashboard.putNumber("Limelight Z", Limelight.getTagRZ());
        SmartDashboard.putNumber("Limelight dist", Math.hypot(Limelight.getTagRZ(), Limelight.getTagRX()));
        SmartDashboard.putString("botpose_helpers_pose", LimelightHelpers.getBotPose2d_wpiBlue("limelight").getTranslation().toString());

        // shooter debug
        SmartDashboard.putBoolean("Shooter Min Error", shooter.nearTarget());
        SmartDashboard.putNumber("Shooter Left", shooter.getBottomShooterSpeed());
        SmartDashboard.putNumber("Shooter Right", shooter.getTopShooterSpeed());
        SmartDashboard.putNumber("Shooter error", shooter.getError());
        SmartDashboard.putData("Shooter Cmd", shooter);

        // angler debug
        SmartDashboard.putNumber("Angler encoder", angler.getPosition());
        SmartDashboard.putNumber("Angler error", angler.getError());
        SmartDashboard.putBoolean("Angler Min Error", angler.anglerNearTarget());
        SmartDashboard.putBoolean("Angler bottom triggered", angler.lowerSwitchTriggered());
        SmartDashboard.putNumber("Angler trim offset", Constants.Angler.ANGLER_ENCODER_OFFSET);
        SmartDashboard.putData("Angler Cmd", angler);
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
