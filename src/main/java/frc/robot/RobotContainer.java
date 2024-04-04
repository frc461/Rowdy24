package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
     * Up:
     * Down:
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
     * Left Joystick Button: Angler trim down by 0.1 encoder counts
     * Right Joystick Button: Angler trim up by 0.1 encoder counts
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
     * A: Shuttle note
     * B: Force intake
     * X: Enable/disable constant shooter when close to speaker
     * Y: Elevator amp auto score
     */


    /* Variables */
    private final BooleanSupplier readyToShoot = () -> shooter.nearTarget() && angler.anglerNearTarget();
    private boolean constantShooter = true;
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

        shooter.setDefaultCommand(new TeleopShooterCommand(shooter, swerve));

        // Configure controller button bindings
        configureButtonBindings();
    }

    public void registerAutoCommands() {
        NamedCommands.registerCommand(
                "startShooterAngler",
                new ParallelCommandGroup(
                        new RevUpShooterCommand(shooter, swerve),
                        new AutoAlignCommand(angler, swerve)
                )
        );

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
                "intakeThenShoot",
                new IntakeCarriageCommand(
                        intakeCarriage,
                        0.9,
                        1
                ).until(intakeCarriage::noteInShootingSystem)
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1
                        ).until(() -> !intakeCarriage.noteInShootingSystem()))
        );

        NamedCommands.registerCommand(
                "shootThenIntakeThenShoot",
                new WaitUntilCommand(readyToShoot).andThen(new IntakeCarriageCommand(
                        intakeCarriage,
                        0.9,
                        1
                ).until(() -> !intakeCarriage.noteInShootingSystem()))
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0.9,
                                1
                        ).until(intakeCarriage::noteInShootingSystem))
                        .andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1
                        ).until(() -> !intakeCarriage.noteInShootingSystem()))
        );

        NamedCommands.registerCommand(
                "shoot",
                new WaitUntilCommand(readyToShoot).andThen(new IntakeCarriageCommand(
                        intakeCarriage,
                        0.9,
                        1
                ).until(() -> !intakeCarriage.noteInShootingSystem()))
        );

        NamedCommands.registerCommand(
                "enableTurret",
                new InstantCommand(() -> Limelight.overrideTargetNow = true)
        );

        NamedCommands.registerCommand(
                "turretRealign",
                new LimelightTurretCommand(
                        swerve,
                        () -> 0,
                        () -> 0,
                        () -> false
                ).until(swerve::turretNearTarget)
        );

        NamedCommands.registerCommand(
                "disableTurret",
                new InstantCommand(() -> Limelight.overrideTargetNow = false)
        );
    }

    public void zeroGyro() {
        swerve.zeroGyro(swerve.getFusedPoseEstimator().getRotation().getDegrees());
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

        /* Sync pose at subwoofer center */
        driverXbox.x().onTrue(new InstantCommand(() -> swerve.setFusedPoseEstimator(Limelight.isRedAlliance() ? new Pose2d(15.29, 5.55, new Rotation2d()) : new Pose2d(1.25, 5.55, new Rotation2d(180)))));

        /* Increment Trim */
        driverXbox.rightStick().onTrue(new InstantCommand(() -> Constants.Angler.ANGLER_ENCODER_OFFSET += 0.1));

        /* Decrement Trim */
        driverXbox.leftStick().onTrue(new InstantCommand(() -> Constants.Angler.ANGLER_ENCODER_OFFSET -= 0.1));

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

        /* Shuttle note */
        opXbox.a().whileTrue(
                new ParallelCommandGroup(
                        new FunctionalCommand(
                                () -> {},
                                () -> angler.setEncoderVal(Constants.Angler.ANGLER_SHUTTLE_PRESET),
                                isFinished -> angler.setEncoderVal(Constants.Angler.ANGLER_LOWER_LIMIT),
                                () -> false,
                                angler
                        ),
                        new FunctionalCommand(
                                () -> shooter.setShooterSpeed(Constants.Shooter.SHUTTLE_SHOOTER_POWER),
                                () -> {},
                                isFinished -> shooter.setShooterSpeed(0),
                                () -> false,
                                shooter
                        ),
                        new WaitCommand(0.5).andThen(new IntakeCarriageCommand(
                                intakeCarriage,
                                0,
                                1
                        ))
                ).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
        );

        /* Intake Override */
        opXbox.b().whileTrue(new IntakeCarriageCommand(intakeCarriage, 0.9, 1, true));

        /* Toggle constant shooter */
        opXbox.x().onTrue(
                new InstantCommand(() -> constantShooter = !constantShooter).andThen(
                        new InstantCommand(shooter::removeDefaultCommand).onlyIf(() -> !constantShooter)
                ).andThen(
                        new InstantCommand(() -> shooter.setDefaultCommand(
                                new TeleopShooterCommand(shooter, swerve).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
                        )).onlyIf(() -> constantShooter)
                )
        );

        opXbox.x().onFalse(
                new InstantCommand(() -> shooter.setShooterSpeed(0.0), shooter).onlyIf(() -> !constantShooter)
        );

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

        opXbox.leftStick().whileTrue(
                new RevUpShooterCommand(shooter, swerve)
        );

        /* Farther Shot */
        opXbox.povUp().whileTrue(
                new ParallelCommandGroup(
                        new FunctionalCommand(
                                () -> {},
                                () -> angler.setEncoderVal(5.91),
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
                        .andThen(new WaitCommand(0.75))
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

        /* Subwoofer Auto-Shoot */
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
        SmartDashboard.putNumber("dist", swerve.getVectorToSpeakerTarget().getNorm());
        SmartDashboard.putNumber("angle to speaker", swerve.getAngleToSpeakerTarget());
        SmartDashboard.putNumber("turret error", swerve.getTurretError());
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
        SmartDashboard.putNumber("Num tags", Limelight.getNumTag());
        SmartDashboard.putNumber("Tag ID", LimelightHelpers.getFiducialID("limelight"));
        SmartDashboard.putString("botpose_helpers_pose", LimelightHelpers.getBotPose2d_wpiBlue("limelight").getTranslation().toString());
        SmartDashboard.putString("botpose_helpers_pose2", LimelightHelpers.getBotPose2d_orb_wpiBlue("limelight").getTranslation().toString());

        // shooter debug
        SmartDashboard.putBoolean("Shooter Min Error", shooter.nearTarget());
        SmartDashboard.putNumber("Shooter Left", shooter.getBottomShooterSpeed());
        SmartDashboard.putNumber("Shooter Right", shooter.getTopShooterSpeed());
        SmartDashboard.putNumber("Shooter error", shooter.getError());
        SmartDashboard.putData("Shooter Cmd", shooter);
        SmartDashboard.putBoolean("Constant shooter?", constantShooter);

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