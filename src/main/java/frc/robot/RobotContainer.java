package frc.robot;

import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
    private final int revShooter = XboxController.Axis.kRightTrigger.value;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Buttons */
    private final JoystickButton operatorStowButton = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton elevatorAmp = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);

    private final JoystickButton outtakeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    private final POVButton toggleAutoSubsystems = new POVButton(operator, 0);
    private final POVButton operatorNinety = new POVButton(operator, 90);
    private final POVButton operatorOneEighty = new POVButton(operator, 180);
    private final POVButton operatorTwoSeventy = new POVButton(operator, 270);

    /* Driver Buttons */
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton outtakeButtonDriver = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton driverLimelight = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverStowButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final POVButton driverZero = new POVButton(driver, 0);
    private final POVButton driverNinety = new POVButton(driver, 90);
    private final POVButton driverOneEighty = new POVButton(driver, 180);
    private final POVButton driverTwoSeventy = new POVButton(driver, 270);

    /* LED Initialization */
    private final DigitalOutput lightEight = new DigitalOutput(8);
    private final DigitalOutput lightNine = new DigitalOutput(9);

    /* Variables */
    private final EventLoop eventLoop = new EventLoop();
    private boolean autoSubsystems = true; //Disables/enables automatic subsystem functions (e.g. auto-intake)
    private final SendableChooser<Command> chooser;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */

    public RobotContainer() {
        // Configure named commands
        NamedCommands.registerCommand("intake", new AutoIntake(intakeCarriage));
        NamedCommands.registerCommand("shoot", new AutoShooter(shooter));

        // Configure default commands
        swerve.setDefaultCommand(
                new TeleopSwerve(
                        swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        robotCentric
                )
        );

        angler.setDefaultCommand(
        new TeleopAngler(
        angler,
        () -> -operator.getRawAxis(anglerAxis)
        )
        );

        // elevator.setDefaultCommand(
        // new TeleopElevator(
        // elevator,
        // () -> -operator.getRawAxis(elevatorAxis)
        // )
        // );

        // Configure the button bindings
        configureButtonBindings();

        // Configure auto
        chooser = AutoBuilder.buildAutoChooser("defaultAuto");
        SmartDashboard.putData("Auto Choices", chooser);

    }

    public void periodic() {
        eventLoop.poll();
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

        toggleAutoSubsystems.onTrue(new InstantCommand(() -> autoSubsystems = !autoSubsystems));

        driverLimelight.whileTrue(new TeleopLimelightTurret(
                limelight,
                swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                robotCentric)
        );

        intakeButton.whileTrue(Commands.parallel(
                new InstantCommand(() -> intakeCarriage.setIntakeSpeed(0.75)),
                new InstantCommand(() -> intakeCarriage.setCarriageSpeed(1))
        ));

        intakeButton.whileFalse(Commands.parallel(
                new InstantCommand(() ->  intakeCarriage.setIntakeSpeed(/*autoSubsystems ? -0.15 : */0)),
                new InstantCommand(() -> intakeCarriage.setCarriageSpeed(0))
        ));

        outtakeButton.whileTrue(Commands.parallel(
                new InstantCommand(() -> intakeCarriage.setIntakeSpeed(-0.75)),
                new InstantCommand(() -> intakeCarriage.setCarriageSpeed(-1))
        ));

        outtakeButton.whileFalse(Commands.parallel(
                new InstantCommand(() -> intakeCarriage.setIntakeSpeed(/*autoSubsystems ? -0.15 : */0)),
                new InstantCommand(() -> intakeCarriage.setCarriageSpeed(0))
        ));

        outtakeButtonDriver.whileTrue(new InstantCommand(() -> intakeCarriage.overrideIntakeSpeed(-0.75)));

        outtakeButtonDriver.whileFalse(new InstantCommand(() -> intakeCarriage.setIntakeSpeed(/*autoSubsystems ? -0.15 : */0)));

        BooleanEvent revShooterPressed = operator.axisGreaterThan(revShooter, Constants.TRIGGER_DEADBAND, eventLoop);
        revShooterPressed.ifHigh(
                () -> shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED +
                        limelight.getRZ() * Constants.Shooter.DISTANCE_MULTIPLIER, false)
        );

        BooleanEvent revShooterNotPressed = revShooterPressed.negate();
        revShooterNotPressed.ifHigh(
                () -> shooter.shoot(autoSubsystems ? Constants.Shooter.IDLE_SHOOTER_SPEED: 0, true)
        );

        driverStowButton.onTrue(
                new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW))
        );

        operatorStowButton.onTrue(
                new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW))
        );

        elevatorAmp.onTrue(new InstantCommand(() ->
                elevator.setHeight(Constants.Elevator.ELEVATOR_AMP)
        ));
        operatorX.onTrue(new InstantCommand(()->angler.tiltShooter(0.9)));
    }

    // smartdashboard prints
    public void printValues() {
        // robot position
        SmartDashboard.putString("Robot Pose2d", swerve.getPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot Yaw", swerve.getYaw());
        SmartDashboard.putNumber("Robot Pitch", swerve.getPitch());
        SmartDashboard.putNumber("Robot Roll", swerve.getRoll());

        // elevator debug
        // SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
        // SmartDashboard.putNumber("Elevator Target", elevator.getTarget());
        // SmartDashboard.putNumber("Elevator Power", elevator.elevatorPower());
        // SmartDashboard.putBoolean("Elevator Limit Triggered?",
        // elevator.elevatorSwitchTriggered());

        // limelight debug
        SmartDashboard.putNumber("Limelight Updates", limelight.getUpdates());
        SmartDashboard.putNumber("Limelight Yaw", limelight.getYaw());
        SmartDashboard.putNumber("Limelight Pitch", limelight.getPitch());
        SmartDashboard.putNumber("Limelight Roll", limelight.getRoll());
        SmartDashboard.putNumber("Limelight X", limelight.getRX());
        SmartDashboard.putNumber("Limelight Y", limelight.getRY());
        SmartDashboard.putNumber("Limelight Z", limelight.getRZ());

        //shooter debug
        SmartDashboard.putNumber("Shooter Left", shooter.getLeftShooterSpeed());
        SmartDashboard.putNumber("Shooter Right", shooter.getRightShooterSpeed());

        SmartDashboard.putNumber("Angler encoder", angler.getEncoder());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */

    public Command getAutonomousCommand() {
        // TODO: work on paths
        return chooser.getSelected();
    }
}
