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
import frc.robot.AlphaBotConstants;
import frc.robot.Configuration;
import frc.robot.RobotConstants;
import frc.robot.RobotIdentity;

public class Swerve extends SubsystemBase {
    private final SwerveDriveOdometry swerveOdometry;
    private final SwerveModule[] swerveMods;
    public final Pigeon2 gyro;
    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();
    final Field2d field = new Field2d();

    // TODO: Once robot is finished, swerve gyro angles will need to be fixed

    public Swerve() {
        gyro = new Pigeon2(configuration.pigeon_id);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        swerveMods = new SwerveModule[] {
            new SwerveModule(0, configuration.swerve_module_constants_0),
            new SwerveModule(1, configuration.swerve_module_constants_1),
            new SwerveModule(2, configuration.swerve_module_constants_2),
            new SwerveModule(3, configuration.swerve_module_constants_3)
        };

        /*
         * By pausing init for a second before setting module offsets, we avoid a bug
         * with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(
                configuration.swerve_kinematics,
                getHeading(),
                getModulePositions()
        );

        AutoBuilder.configureHolonomic(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                () -> configuration.swerve_kinematics.toChassisSpeeds(getModuleStates()), // ChassisSpeeds supplier.
                                                                                             // MUST BE ROBOT RELATIVE
                speeds -> {
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                    SwerveModuleState[] swerveModuleStates =
                            configuration.swerve_kinematics.toSwerveModuleStates(speeds);
                    setModuleStates(swerveModuleStates);
                },
                new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                                 // Constants class
                        new PIDConstants( // Translation PID constants
                                configuration.auto_drive_p,
                                configuration.auto_drive_i,
                                configuration.auto_drive_d
                        ),
                        new PIDConstants( // Rotation PID constants
                                configuration.auto_angle_p,
                                configuration.auto_angle_i,
                                configuration.auto_angle_d
                        ),
                        configuration.max_speed - 2, // Max module speed, in m/s
                        configuration.center_to_wheel, // Drive base radius in meters. Distance from robot center to
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
        SwerveModuleState[] swerveModuleStates = configuration.swerve_kinematics.toSwerveModuleStates(
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
        return (configuration.invert_gyro) ?
                AlphaBotConstants.MAXIMUM_ANGLE - (gyro.getYaw().getValueAsDouble()) :
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

    public void zeroGyro() {
        gyro.setYaw(-90);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, configuration.max_speed);

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