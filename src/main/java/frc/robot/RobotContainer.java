package frc.robot;

import edu.wpi.first.wpilibj2.command.*;

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
    private final Elevator elevator = new Elevator();
    private final Limelight limelight = new Limelight();

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    /* Operate Controls */
    private final int wristAxis = XboxController.Axis.kLeftY.value;
    private final int elevatorAxis = XboxController.Axis.kRightY.value;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Operator Buttons */
    // TODO: change generic button locations to respective functions
    private final JoystickButton operatorA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton operatorB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton operatorX = new JoystickButton(operator, XboxController.Button.kX.value);
    private final JoystickButton operatorY = new JoystickButton(operator, XboxController.Button.kY.value);

    private final JoystickButton operatorLeftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton operatorRightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

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
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                () -> -driver.getRawAxis(rotationAxis),
                robotCentric
            )
        );

        elevator.setDefaultCommand(
            new TeleopElevator(
                    elevator,
                () -> -operator.getRawAxis(elevatorAxis)
            )
        );

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

        driverLimelight.whileTrue(new LimelightFollow(
                limelight,
                swerve,
                () -> -driver.getRawAxis(translationAxis),
                () -> -driver.getRawAxis(strafeAxis),
                robotCentric)
        );

        /* Operator Buttons */
    }

    // smartdashboard prints
    public void printValues() {
        // robot position
        SmartDashboard.putString("Robot Pose2d", swerve.getPose().getTranslation().toString());
        SmartDashboard.putNumber("Robot Yaw", swerve.getYaw());
        SmartDashboard.putNumber("Robot Pitch", swerve.getPitch());
        SmartDashboard.putNumber("Robot Roll", swerve.getRoll());

        // elevator debug
        SmartDashboard.putNumber("Elevator Position", elevator.getPosition());
        SmartDashboard.putNumber("Elevator Target", elevator.getTarget());
        SmartDashboard.putNumber("Elevator Power", elevator.elevatorPower());
        SmartDashboard.putBoolean("Elevator Limit Triggered?", elevator.elevatorSwitchTriggered());

        // limelight debug
        SmartDashboard.putNumber("Limelight Updates", limelight.getUpdates());
        SmartDashboard.putNumber("Limelight Yaw", limelight.getYaw());
        SmartDashboard.putNumber("Limelight Pitch", limelight.getPitch());
        SmartDashboard.putNumber("Limelight Roll", limelight.getRoll());
        SmartDashboard.putNumber("Limelight X", limelight.getRX());
        SmartDashboard.putNumber("Limelight Y", limelight.getRY());
        SmartDashboard.putNumber("Limelight Z", limelight.getRZ());
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
