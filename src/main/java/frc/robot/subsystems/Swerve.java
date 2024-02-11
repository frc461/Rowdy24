package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveMods;
    public final Pigeon2 gyro;
    final Field2d field = new Field2d();

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

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
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
                        Constants.Swerve.MAX_SPEED - 2, // Max module speed, in m/s
                        Constants.Swerve.CENTER_TO_WHEEL, // Drive base radius in meters. Distance from robot center to
                                                          // furthest module.
                        new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
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
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getHeading(), getModulePositions());
        field.setRobotPose(swerveOdometry.getPoseMeters());

        for (SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Absolute", mod.getAbsoluteAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Relative", mod.getAngle().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Position", mod.getPosition().distanceMeters);
        }
    }

    public Field2d getField2d() {
        return field;
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

    public double getYaw() {
        return (Constants.Swerve.INVERT_GYRO) ?
                Constants.MAXIMUM_ANGLE - (gyro.getYaw().getValueAsDouble()) :
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

    // TODO: FIX SWERVE SCUFFEDNESS
    public void zeroGyro() {
        gyro.setYaw(-90);
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