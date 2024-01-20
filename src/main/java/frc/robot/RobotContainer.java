// this is a comment
package frc.robot;

import edu.wpi.first.wpilibj2.command.*;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.AutoChooser;
import frc.robot.autos.eventMap;
// import frc.robot.autos.AutoChooser;
// import frc.robot.autos.AutoTrajectories;
//import frc.robot.autos.eventMap;
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
  public final Swerve s_Swerve = new Swerve();
  private final Elevator s_Elevator = new Elevator();
  private final Limelight limelight = new Limelight();
  private final AutoChooser chooser = new AutoChooser();

  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // op controls
  private final int wristAxis = XboxController.Axis.kLeftY.value;
  private final int elevatorAxis = XboxController.Axis.kRightY.value;

  private final JoystickButton e_presButton_0 = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton e_presButton_1 = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton e_presButton_2 = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton e_presButton_3 = new JoystickButton(operator, XboxController.Button.kB.value);

  private final POVButton w_preset_0 = new POVButton(operator, 0);
  private final POVButton w_preset_1 = new POVButton(operator, 90);
  private final POVButton w_preset_2 = new POVButton(operator, 180);
  private final POVButton operator_stowButton = new POVButton(operator, 270);

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton driver_stowButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton driver_limelightButton = new JoystickButton(driver, XboxController.Button.kB.value);

  private final POVButton driver_stowButton2 = new POVButton(operator, 270);
  // private final JoystickButton xModeButton = new JoystickButton(driver,
  // XboxController.Button.kX.value);

  /* LED Initialization Code */
  private DigitalOutput LEDzero = new DigitalOutput(8);
  private DigitalOutput LEDone = new DigitalOutput(9);

  /* Variables */

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

    s_Elevator.setDefaultCommand(
        new TeleopElevator(
            s_Elevator,
            () -> -operator.getRawAxis(elevatorAxis)));

    limelight.setDefaultCommand(
        new TeleopLimelight(limelight)
    );

    // Configure the button bindings
    configureButtonBindings();

    SmartDashboard.putData("Auto Choices", chooser.getAutoChooser());
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
    /* Driver Buttons (and op buttons) */

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    //driver_limelightButton.whileTrue(Commands.sequence(new LimelightFollow(limelight, s_Swerve)));
    driver_limelightButton.whileTrue(new LimelightFollow(
            limelight,
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

  }

  public void printValues() { // all of the smartdashboard prints:
    SmartDashboard.putNumber("balanceP", 0.03);
    // SmartDashboard.getNumber("balanceI", elevatorAxis);
    // SmartDashboard.getNumber("balanceD", elevatorAxis);
    SmartDashboard.putNumber("bpftX", limelight.botPoseX);
    SmartDashboard.putNumber("bpftZ", limelight.botPoseZ);
    SmartDashboard.putNumber("Limelight updates", limelight.updates);
    SmartDashboard.putBoolean("Pov pressed", e_presButton_0.getAsBoolean());
    SmartDashboard.putNumber("Elevator Position", s_Elevator.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Target", s_Elevator.getTarget());
    SmartDashboard.putBoolean("elevator limit triggered?", s_Elevator.elevatorSwitchTriggered());
    SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw());
    SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch());
    SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll());
    SmartDashboard.putNumber("elevatorPower", s_Elevator.elevatorPower());
    SmartDashboard.putNumber("rollLimelight", limelight.getRoll());
    SmartDashboard.putNumber("yawLimelight", limelight.getYaw());
    SmartDashboard.putNumber("pitchLimelight", limelight.getPitch());

    // SmartDashboard.putNumber("Pid off",
    // chooser.getPIDController().getPositionError());

     SmartDashboard.putNumber("RX", limelight.getRX());
     SmartDashboard.putNumber("RY", limelight.getRY());
     SmartDashboard.putNumber("RZ", limelight.getRZ());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    return chooser.getCommand();
  }
}
