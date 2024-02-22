package frc.robot;
import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    public final static Joystick driver = new Joystick(0);
    public final static Joystick operator = new Joystick(1);

    /* Operate Controls */
    private final int anglerAxis = XboxController.Axis.kLeftY.value;
    private final int elevatorAxis = XboxController.Axis.kRightY.value;
    private final int revShoot = XboxController.Axis.kLeftTrigger.value;
    private final int autoAlignRevShoot = XboxController.Axis.kRightTrigger.value;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Buttons */
    private final JoystickButton operatorStow = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton overrideIntake = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton elevatorAmp = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton alignAngler = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton outtake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    private final POVButton toggleIdleMode = new POVButton(operator, 0);
    private final POVButton operatorNinety = new POVButton(operator, 90);
    private final POVButton toggleClamp = new POVButton(operator, 180);
    private final POVButton operatorTwoSeventy = new POVButton(operator, 270);

    /* Driver Buttons */
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton outtakeButtonDriver = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton driverLimelight = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverStow = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final POVButton driverZero = new POVButton(driver, 0);
    private final POVButton driverNinety = new POVButton(driver, 90);
    private final POVButton driverOneEighty = new POVButton(driver, 180);
    private final POVButton driverTwoSeventy = new POVButton(driver, 270);

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
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> driver.getRawAxis(rotationAxis),
                        robotCentric
                )
        );

        angler.setDefaultCommand(
                new TeleopAnglerCommand(
                        angler,
                        () -> -operator.getRawAxis(anglerAxis)
                )
        );

        elevator.setDefaultCommand(
                new TeleopElevatorCommand(
                        elevator,
                        () -> -operator.getRawAxis(elevatorAxis)
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
        zeroGyro.onTrue(new InstantCommand(swerve::zeroGyro));

        toggleIdleMode.onTrue(new SequentialCommandGroup(
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

        driverLimelight.whileTrue(
        new LimelightTurretCommand(
                limelight,
                swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                robotCentric)
        );

        intake.whileTrue(new IntakeCarriageCommand(
                intakeCarriage,
                0.9,
                1,
                idleMode
        ).until(intakeCarriage::noteInSystem));

        outtake.whileTrue(new IntakeCarriageCommand(intakeCarriage, -0.9, -1, idleMode));

        overrideIntake.whileTrue(new IntakeCarriageCommand(intakeCarriage, 0.9, 1, idleMode));

        new Trigger(() -> operator.getRawAxis(autoAlignRevShoot) > Constants.TRIGGER_DEADBAND).onTrue(
                new InstantCommand(() -> angler.setAlignedAngle(
                        limelight.getRX(),
                        limelight.getRZ(),
                        limelight.tagExists()
                ))
        );

        new Trigger(() -> operator.getRawAxis(autoAlignRevShoot) > Constants.TRIGGER_DEADBAND).whileTrue(
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

        new Trigger(() -> operator.getRawAxis(revShoot) > Constants.TRIGGER_DEADBAND).whileTrue(new RevUpShooterCommand(
                shooter, limelight, idleMode
                )
        );
        
        toggleClamp.onTrue(new InstantCommand(()-> elevator.setClamp())); //toggle clamp

        driverStow.onTrue(
            new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW))
        );

        operatorStow.onTrue(
            new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW))
        );

        elevatorAmp.onTrue(new InstantCommand(() ->
            elevator.setHeight(Constants.Elevator.ELEVATOR_AMP)
        ));

        alignAngler.onTrue(new InstantCommand(() -> angler.setAlignedAngle(
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
