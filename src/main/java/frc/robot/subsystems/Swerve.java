package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;

import java.util.Optional;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator fusedPoseEstimator;
    private final SwerveModule[] swerveMods;
    private final PIDController limelightRotController;
    public final Pigeon2 gyro;
    private double turretError;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.PIGEON_ID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        swerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.SWERVE_MODULE_CONSTANTS),
            new SwerveModule(1, Constants.Swerve.Mod1.SWERVE_MODULE_CONSTANTS),
            new SwerveModule(2, Constants.Swerve.Mod2.SWERVE_MODULE_CONSTANTS),
            new SwerveModule(3, Constants.Swerve.Mod3.SWERVE_MODULE_CONSTANTS)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(
                Constants.Swerve.SWERVE_KINEMATICS,
                getHeading(),
                getModulePositions()
        );

        limelightRotController = new PIDController(
                Constants.Limelight.LIMELIGHT_P,
                Constants.Limelight.LIMELIGHT_I,
                Constants.Limelight.LIMELIGHT_D
        );
        limelightRotController.enableContinuousInput(Constants.Swerve.MINIMUM_ANGLE, Constants.Swerve.MAXIMUM_ANGLE);

        AutoBuilder.configureHolonomic(
                this::getFusedPoseEstimator, // Robot pose supplier
                this::setFusedPoseEstimator, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier.
                                                                                             // MUST BE ROBOT RELATIVE
                speeds -> {
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    SwerveModuleState[] swerveModuleStates =
                            Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(speeds);
                    setModuleStates(swerveModuleStates);
                },
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants( // Translation PID constants
                                Constants.Auto.AUTO_DRIVE_P,
                                Constants.Auto.AUTO_DRIVE_I,
                                Constants.Auto.AUTO_DRIVE_D
                        ),
                        new PIDConstants( // Rotation PID constants
                                Constants.Auto.AUTO_ANGLE_P,
                                Constants.Auto.AUTO_ANGLE_I,
                                Constants.Auto.AUTO_ANGLE_D
                        ),
                        Constants.Swerve.MAX_SPEED, // Max module speed, in m/s
                        Constants.Swerve.CENTER_TO_WHEEL, // Drive base radius in meters. Distance from robot center to
                                                          // furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                ),
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this // Reference to this subsystem to set requirements
        );

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);

        fusedPoseEstimator = new SwerveDrivePoseEstimator(
            Constants.Swerve.SWERVE_KINEMATICS,
            getHeading(), getModulePositions(),
            getPose()
        );

        turretError = 1.0;
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getHeading(), getModulePositions());
        updateFusedPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
        turretError = Math.abs(getAngleToSpeakerTarget() - getFusedPoseEstimator().getRotation().getDegrees()) > 180 ?
                Math.abs(getAngleToSpeakerTarget() - getFusedPoseEstimator().getRotation().getDegrees()) - 360 :
                Math.abs(getAngleToSpeakerTarget() - getFusedPoseEstimator().getRotation().getDegrees());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Absolute", mod.getAbsoluteAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Relative", mod.getAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Position", mod.getPosition().distanceMeters);
        }
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getHeading()
                ) : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation
                )
        );
        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    public void driveTurret(Translation2d translation, boolean fieldRelative) {
        // aligns with speaker April Tag
        drive(
                translation,
                limelightRotController.calculate(
                        getFusedPoseEstimator().getRotation().getDegrees(),
                        getAngleToSpeakerTarget()
                ) * Constants.Swerve.MAX_ANGULAR_VELOCITY,
                fieldRelative,
                true
        );
    }

    public double getYaw() {
        return (Constants.Swerve.INVERT_GYRO) ?
                Constants.Swerve.MAXIMUM_ANGLE - (gyro.getYaw().getValueAsDouble()) :
                gyro.getYaw().getValueAsDouble();
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getYaw());
    }

    public double getPitch() {
        return gyro.getPitch().getValueAsDouble();
    }

    public double getRoll() {
        return gyro.getRoll().getValueAsDouble();
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getFusedPoseEstimator(){
        return fusedPoseEstimator.getEstimatedPosition();
    }

    public void setFusedPoseEstimator(Pose2d newPose){
        fusedPoseEstimator.resetPosition(getHeading(), getModulePositions(), newPose);
        resetOdometry(newPose);
    }

    public Translation2d getVectorToSpeakerTarget() {
        Translation2d fusedPose = getFusedPoseEstimator().getTranslation();
        Translation2d speakerTagPose = Limelight.getSpeakerTagPose().getTranslation();
        return fusedPose.minus(speakerTagPose).unaryMinus();
    }

    public double getAngleToSpeakerTarget() {
        return getVectorToSpeakerTarget().getAngle().getDegrees();
    }

    public double getTurretError() {
        return turretError;
    }

    public boolean turretNearTarget() {
        return Math.abs(turretError) < Constants.Swerve.TURRET_ACCURACY_REQUIREMENT;
    }

    public void resetFusedPose(){
        setFusedPoseEstimator(new Pose2d());
    }

    public void updateFusedPose(Pose2d limelightPose){
        fusedPoseEstimator.update(getHeading(), getModulePositions());

        if (Limelight.tagExists()) {
            fusedPoseEstimator.addVisionMeasurement(
                    limelightPose,
                    Timer.getFPGATimestamp()
                            - LimelightHelpers.getLatency_Pipeline("limelight") / 1000.0
                            - LimelightHelpers.getLatency_Capture("limelight") / 1000.0
            );
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Optional<Rotation2d> getRotationTargetOverride() { // only for auto
        if (Limelight.overrideTargetNow) {
            return Optional.of(Rotation2d.fromDegrees(getAngleToSpeakerTarget()));
        }
        return Optional.empty();
    }

    public void zeroGyro() {
        gyro.setYaw(Constants.Swerve.GYRO_OFFSET);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.MAX_SPEED);

        for (SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, true);
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }
}