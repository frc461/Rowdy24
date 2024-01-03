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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
  private final Intake s_Intake = new Intake();
  private final Wrist s_Wrist = new Wrist(s_Elevator.getEncoder());
  private final Limelight limelight = new Limelight();
  private final eventMap map = new eventMap(s_Swerve, s_Intake, s_Wrist, s_Elevator);
  // private final AutoTrajectories trajectories = new AutoTrajectories();
  // private final AutoChooser chooser = new AutoChooser(trajectories,
  // map.getMap(), s_Swerve, s_Intake, s_Wrist, s_Elevator);

  public double intakeVec = 0;

  public Command autoCode = Commands.sequence(new PrintCommand("no auto selected"));

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
  boolean driveStatus = false;

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

    s_Wrist.setDefaultCommand(
        new TeleopWrist(
            s_Wrist,
            () -> -operator.getRawAxis(wristAxis)));

    s_Intake.setDefaultCommand(
        new TeleopIntake(
            s_Intake,
            operator));

    limelight.setDefaultCommand(
        new TeleopLimelight(
            limelight));

    // Configure the button bindings
    configureButtonBindings();

    // SmartDashboard.putData("Auto Choices", chooser.getAutoChooser());

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

    SmartDashboard.putData("Pathfind to Pickup Pos", AutoBuilder.pathfindToPose(
        new Pose2d(14.0, 6.5, Rotation2d.fromDegrees(0)), 
        new PathConstraints(
          4.0, 4.0, 
          Units.degreesToRadians(360), Units.degreesToRadians(540)
        ), 
        0, 
        2.0
      ));
      
    w_preset_0.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT)));
    w_preset_1.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_MID_LIMIT)));
    w_preset_2.onTrue(new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_LOWER_LIMIT)));

    e_presButton_0.onTrue( // Preset to score high cone
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(true)),
            new InstantCommand(() -> LEDone.set(true)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighScore)),
            new WaitCommand(.25),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighConeScore)))

    );

    e_presButton_1.onTrue( // Preset to score mid cone
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(true)),
            new InstantCommand(() -> LEDone.set(true)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorMidScore)),
            new WaitCommand(.1),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristMidConeScore))));

    e_presButton_2.onTrue( // Preset to pick up fallen cone
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(false)),
            new InstantCommand(() -> LEDone.set(true)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorConePickup)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristConePickup))));

    e_presButton_3.onTrue( // Preset to pick up cone from single substation
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(false)),
            new InstantCommand(() -> LEDone.set(true)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristConePickup2))));

    w_preset_0.onTrue( // Preset to score high cube
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(true)),
            new InstantCommand(() -> LEDone.set(true)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorHighCubeScore)),
            new WaitCommand(.25),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristHighCubeScore))));

    w_preset_1.onTrue( // Preset to pick up cube
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(true)),
            new InstantCommand(() -> LEDone.set(false)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristCubePickup))
        // Set lights Purple
        ));

    w_preset_2.onTrue( // Preset to score mid cube
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(true)),
            new InstantCommand(() -> LEDone.set(true)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorMidCubeScore)),
            new WaitCommand(.1),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.wristMidCubeScore))));

    operator_stowButton.onTrue(
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(false)),
            new InstantCommand(() -> LEDone.set(false)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))));

    driver_stowButton2.onTrue(
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(false)),
            new InstantCommand(() -> LEDone.set(false)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))));

    driver_stowButton.onTrue(
        Commands.sequence(
            new InstantCommand(() -> LEDzero.set(false)),
            new InstantCommand(() -> LEDone.set(false)),
            new InstantCommand(() -> s_Elevator.setHeight(Constants.elevatorBot)),
            new InstantCommand(() -> s_Wrist.setRotation(Constants.WRIST_UPPER_LIMIT))));

    // driver_AutoBalance.onTrue(new InstantCommand(() -> s_Swerve.autoBalance()));
    // driver_limelightButton.onTrue(
    //         new InstantCommand(() -> s_Swerve.resetOdometry(
    //             // new Pose2d(
    //             //     limelight.botPoseZ, -limelight.botPoseX, Rotation2d.fromDegrees(limelight.botPose[4])))));
    //             new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));

    // driver_limelightButton.onTrue(limelight.getTagCommand());
    driver_limelightButton.whileTrue(
    Commands.sequence(
    new PrintCommand("start\n"),
    new InstantCommand(() -> s_Swerve.resetOdometry(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)))),
    // new Pose2d(
    // limelight.botPoseZ, -limelight.botPoseX,
    // Rotation2d.fromDegrees(limelight.botPose[4]),
    // )
    // )
    // ),
    new PrintCommand("about to drive\n"),

    limelight.getTagCommand(),
    new PrintCommand("done\n")
    )
    );

    // Commands.sequence(
    // new InstantCommand(() -> limelight.refreshValues()),
    // new InstantCommand(() -> s_Swerve.resetOdometry(
    // new Pose2d(
    // limelight.botPoseZ, -limelight.botPoseX, Rotation2d.fromDegrees(0)
    // )
    // )
    // ),
    // new SwerveControllerCommand (
    // limelight.testTraj(s_Swerve.getYaw()),
    // s_Swerve::getPose,
    // Constants.Swerve.swerveKinematics,
    // new PIDController(0.6, 0, 0),
    // new PIDController(0.7, 0, 0.1),
    // new ProfiledPIDController(0.2, 0, 0,
    // Constants.AutoConstants.kThetaControllerConstraints),
    // s_Swerve::setModuleStates,
    // s_Swerve
    // )
    // )
    // );

    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    // xModeButton.whileTrue(new InstantCommand(()-> s_Swerve.setXMode()));

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
    SmartDashboard.putNumber("Wrist Position", s_Wrist.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist Target", s_Wrist.getTarget());
    SmartDashboard.putBoolean("cube beam broken?: ", s_Intake.cubeBeamBroken());
    SmartDashboard.putBoolean("cone beam broken?", s_Intake.coneBeamBroken());
    SmartDashboard.putNumber("intake speed", s_Intake.getSpeed());
    SmartDashboard.putNumber("yaw", s_Swerve.gyro.getYaw());
    SmartDashboard.putNumber("pitch", s_Swerve.gyro.getPitch());
    SmartDashboard.putNumber("roll", s_Swerve.gyro.getRoll());
    SmartDashboard.putNumber("elevatorPower", s_Elevator.elevatorPower());

    // SmartDashboard.putNumber("Pid off",
    // chooser.getPIDController().getPositionError());

    // SmartDashboard.putNumber("RX", s_Limelight.getRX());
    // SmartDashboard.putNumber("RY", s_Limelight.getRY());
    // SmartDashboard.putNumber("RZ", s_Limelight.getRZ());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {

    PathConstraints constraints = new PathConstraints(
            Constants.Swerve.maxSpeed, Constants.Swerve.maxAccel, 
            Constants.Swerve.maxAngularVelocity, Units.degreesToRadians(720));

    //AutoBuilder.pathfindToPose(new Pose2d(2, 0, Rotation2d.fromDegrees(0)), constraints, 0, 0);

        // Create a list of bezier points from poses. Each pose represents one waypoint. 
        // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
        s_Swerve.resetOdometry(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
        s_Swerve.gyro.setYaw(0);
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        //new Pose2d(2.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(-limelight.botPoseZ, -limelight.botPoseX, Rotation2d.fromDegrees(0))
        );
        
        // Create the path using the bezier points created above
        PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            constraints,
            new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
        );


       return AutoBuilder.pathfindThenFollowPath(path, constraints, 3);


    // Constants.gyroOffset = s_Swerve.gyro.getPitch();
    // // s_Swerve.zeroGyro();
    // s_Swerve.gyro.setYaw(180);
    // // s_Swerve.resetOdometry(new Pose2d(
    // // limelight.getRX(), limelight.getRZ(), Rotation2d.fromDegrees(0))
    // // );
    // s_Swerve.resetOdometry(new Pose2d(
    //     10, 10, Rotation2d.fromDegrees(0)));
    // return (new SwerveControllerCommand(
    //     limelight.testTraj(s_Swerve.getYaw()),
    //     s_Swerve::getPose,
    //     Constants.Swerve.swerveKinematics,
    //     new PIDController(1, 0.05, 0.01),
    //     new PIDController(1, 0.05, 0.01),
    //     new ProfiledPIDController(1, 0.05, 0, Constants.AutoConstants.kThetaControllerConstraints),
    //     s_Swerve::setModuleStates,
    //     s_Swerve));

    // chooser.getCommand();
  }
}
