package frc.robot;
import edu.wpi.first.wpilibj2.command.*;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    //private final Elevator elevator = new Elevator();
    private final Limelight limelight = new Limelight();
    private final IntakeCarriage intakeCarriage = new IntakeCarriage();
    private final Shooter shooter = new Shooter();
    // private final Angler angler = new Angler();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Operate Controls */
    private final int anglerAxis = XboxController.Axis.kLeftY.value;
    private final int elevatorAxis = XboxController.Axis.kRightY.value;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Buttons */
    private final JoystickButton elevatorStow = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton shootButton = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton revShooter = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton elevatorAmp = new JoystickButton(operator, XboxController.Button.kY.value);

    private final JoystickButton outtakeButton = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton intakeButton = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    private final POVButton operatorZero = new POVButton(operator, 0);
    private final POVButton operatorNinety = new POVButton(operator, 90);
    private final POVButton operatorOneEighty = new POVButton(operator, 180);
    private final POVButton operatorTwoSeventy = new POVButton(operator, 270);

    /* Driver Buttons */
    private final JoystickButton driverA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driverLimelight = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driverX = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);

    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton driverRightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    private final POVButton driverZero = new POVButton(driver, 0);
    private final POVButton driverNinety = new POVButton(driver, 90);
    private final POVButton driverOneEighty = new POVButton(driver, 180);
    private final POVButton driverTwoSeventy = new POVButton(driver, 270);

    /* LED Initialization */
    private final DigitalOutput lightEight = new DigitalOutput(8);
    private final DigitalOutput lightNine = new DigitalOutput(9);

    /* Variables */

    /**
    * The container for the robot. Contains subsystems, IO devices, and commands.
    */

    public RobotContainer() {
        NamedCommands.registerCommand("intake", new AutoIntake(intakeCarriage));
        
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                robotCentric
            )
        );
        
        // angler.setDefaultCommand(
        //     new TeleopAngler(
        //         angler,
        //         () -> -operator.getRawAxis(anglerAxis)
        //     )
        // );

        // elevator.setDefaultCommand(
        //     new TeleopElevator(
        //             elevator,
        //         () -> -operator.getRawAxis(elevatorAxis)
        //     )
        // );

        // Configure the button bindings
        configureButtonBindings();
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

        driverLimelight.whileTrue(new TeleopLimelightTurret(
                limelight,
                swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                robotCentric)
        );

        intakeButton.whileTrue(Commands.parallel(new InstantCommand(()-> intakeCarriage.setIntakeSpeed(0.75)),
        new InstantCommand(()-> intakeCarriage.setCarriageSpeed(1))));

        intakeButton.whileFalse(Commands.parallel(new InstantCommand(()-> intakeCarriage.setIntakeSpeed(-0.15)),
        new InstantCommand(()-> intakeCarriage.setCarriageSpeed(0))));

        outtakeButton.whileTrue(Commands.parallel(new InstantCommand(()-> intakeCarriage.setIntakeSpeed(-0.75)),
        new InstantCommand(()-> intakeCarriage.setCarriageSpeed(-1))));

        outtakeButton.whileFalse(Commands.parallel(new InstantCommand(()-> intakeCarriage.setIntakeSpeed(-0.15)),
        new InstantCommand(()-> intakeCarriage.setCarriageSpeed(0))));

        // TODO: add button to set elevator to shooting preset as well as amp and stow (there are three levels)
        // elevatorAmp.onTrue(new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_AMP)));
        // elevatorStow.onTrue(new InstantCommand(() -> elevator.setHeight(Constants.Elevator.ELEVATOR_STOW)));

        // TODO: verify this trig
        revShooter.whileTrue(
            new InstantCommand(
                () -> shooter.shoot(Constants.Shooter.BASE_SHOOTER_SPEED) //+ limelight.getRZ()*Constants.Shooter.DISTANCE_MULTIPLIER)
        ));

        revShooter.whileFalse(
                new InstantCommand(()-> shooter.shoot(0))
        ); // TODO: could make the shooter run at nonzero speed all the time
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
        // SmartDashboard.putBoolean("Elevator Limit Triggered?", elevator.elevatorSwitchTriggered());

        // limelight debug
        SmartDashboard.putNumber("Limelight Updates", limelight.getUpdates());
        SmartDashboard.putNumber("Limelight Yaw", limelight.getYaw());
        SmartDashboard.putNumber("Limelight Pitch", limelight.getPitch());
        SmartDashboard.putNumber("Limelight Roll", limelight.getRoll());
        SmartDashboard.putNumber("Limelight X", limelight.getRX());
        SmartDashboard.putNumber("Limelight Y", limelight.getRY());
        SmartDashboard.putNumber("Limelight Z", limelight.getRZ());
        SmartDashboard.putNumber("Shooter Left", shooter.getLeftShooterSpeed());
        SmartDashboard.putNumber("Shooter Right", shooter.getRightShooterSpeed());

        

        // SmartDashboard.putNumber("Angler encoder", angler.getEncoder());
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */

    public Command getAutonomousCommand() {
        return null;
    }
}
