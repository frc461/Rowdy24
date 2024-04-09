package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LimelightHelpers;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveDrivePoseEstimator fusedPoseEstimator;
    private final SwerveModule[] swerveMods;
    private final PIDController limelightRotController;
    public final Pigeon2 gyro;
    private double turretError;
    private boolean headingConfigured;

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
                newPose -> {
                    this.setFusedPoseEstimator(newPose);
                    this.setOdometry(newPose);
                }, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> Constants.Swerve.SWERVE_KINEMATICS.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier.
                                                                                             // MUST BE ROBOT RELATIVE
                speeds -> {
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    if (speeds.omegaRadiansPerSecond != 0.0) {
                        speeds.omegaRadiansPerSecond = Limelight.overrideTargetNow ? limelightRotController.calculate(
                                getFusedPoseEstimator().getRotation().getDegrees(),
                                getAngleToSpeakerTarget()
                        ) * Constants.Swerve.MAX_ANGULAR_VELOCITY : speeds.omegaRadiansPerSecond;
                    }

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
                Limelight::isRedAlliance, // Boolean supplier that controls when the path will be mirrored for the red alliance
                                  // This will flip the path being followed to the red side of the field.
                                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                this // Reference to this subsystem to set requirements
        );

        Limelight.configureRobotPose();

        fusedPoseEstimator = new SwerveDrivePoseEstimator(
                Constants.Swerve.SWERVE_KINEMATICS,
                getHeading(), getModulePositions(),
                getPose(),
                VecBuilder.fill(0.2, 0.2, Units.degreesToRadians(2.0)),
                VecBuilder.fill(0.6, 0.6, Units.degreesToRadians(360.0))
        );

        turretError = 0.0;
        headingConfigured = false;
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getHeading(), getModulePositions());
        updateFusedPose();
        turretError = getVectorToSpeakerTarget().getAngle().minus(getFusedPoseEstimator().getRotation()).getDegrees();

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
                        Limelight.isRedAlliance() ? getHeading() : getHeading().rotateBy(Rotation2d.fromDegrees(180))
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

    public Translation2d getVectorToSpeakerTarget() {
        Translation2d fusedPose = getFusedPoseEstimator().getTranslation();
        Translation2d speakerTagPose = Limelight.getSpeakerTagPose().getTranslation()
                .plus(new Translation2d(
                        Limelight.isRedAlliance() ? Constants.Limelight.X_DEPTH_OFFSET : -Constants.Limelight.X_DEPTH_OFFSET,
                        Limelight.isRedAlliance() ? Constants.Limelight.Y_DEPTH_OFFSET : -Constants.Limelight.Y_DEPTH_OFFSET
                ));
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

    public boolean isHeadingConfigured() {
        return headingConfigured;
    }

    public void setOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    public void resetOdometry() {
        setOdometry(new Pose2d());
    }

    public void setFusedPoseEstimator(Pose2d newPose){
        fusedPoseEstimator.resetPosition(getHeading(), getModulePositions(), newPose);
        setOdometry(newPose);
    }

    public void resetFusedPose(){
        setFusedPoseEstimator(new Pose2d());
    }

    public void updateFusedPose(){
        fusedPoseEstimator.update(getHeading(), getModulePositions());
        Pose2d limelightPose;
        double angVel = gyro.getRate();

        LimelightHelpers.setRobotOrientation(
                "limelight",
                fusedPoseEstimator.getEstimatedPosition().getRotation().getDegrees(),
                0,
                0,
                0,
                0,
                0
        );

        if (!headingConfigured) {
            limelightPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            Pose2d limelightPoseDiff = Limelight.getBotPoseBlueMegaTag2("limelight");
            if (Limelight.tagExists() && Limelight.getNearestTagDist() < 2.0) {
                fusedPoseEstimator.addVisionMeasurement(
                        limelightPose,
                        Timer.getFPGATimestamp()
                                - LimelightHelpers.getLatency_Pipeline("limelight") / 1000.0
                                - LimelightHelpers.getLatency_Capture("limelight") / 1000.0,
                        VecBuilder.fill(0.0001, 0.0001, Units.degreesToRadians(0.1))
                );
                if (limelightPose.minus(limelightPoseDiff).getTranslation().getNorm() < 0.02) {
                    headingConfigured = true;
                }
            }
            return;
        }

        if (Limelight.tagExists() && Math.abs(angVel) < 720.0) {
            limelightPose = Limelight.getBotPoseBlueMegaTag2("limelight");
            fusedPoseEstimator.addVisionMeasurement(
                    limelightPose,
                    Timer.getFPGATimestamp()
                            - LimelightHelpers.getLatency_Pipeline("limelight") / 1000.0
                            - LimelightHelpers.getLatency_Capture("limelight") / 1000.0,
                    VecBuilder.fill(0.6, 0.6, Units.degreesToRadians(360.0))
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

    public void zeroGyro() {
        zeroGyro(Constants.Swerve.GYRO_OFFSET);
    }

    public void zeroGyro(double rotation) {
        gyro.setYaw(rotation);
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

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : swerveMods) {
            mod.resetToAbsolute();
        }
    }
}