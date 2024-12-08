package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkUtil.Usage;
import frc.lib.util.CANSparkUtil;
import frc.lib.math.OptimizeModuleState;
import frc.robot.Constants;

public class SwerveModule {
    public int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d currentReferenceAngle;

    private final CANSparkMax angleMotor;
    private final CANSparkMax driveMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder relativeAngleEncoder;
    private final CANcoder absoluteAngleEncoder;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;
    private final SimpleMotorFeedforward feedforward;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Absolute Angle Encoder Config */
        absoluteAngleEncoder = new CANcoder(moduleConstants.cancoderID);
        absoluteAngleEncoder.getConfigurator().apply(new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs().withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                        .withSensorDirection(Constants.Swerve.CANCODER_SENSOR_DIRECTION)
        ));

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        relativeAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        feedforward = new SimpleMotorFeedforward(
                Constants.Swerve.DRIVE_S, Constants.Swerve.DRIVE_V, Constants.Swerve.DRIVE_A
        );

        currentReferenceAngle = Rotation2d.fromDegrees(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = OptimizeModuleState.optimize(desiredState, getAngle());
        setSpeed(desiredState, isOpenLoop);
        setAngle(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
                    desiredState.speedMetersPerSecond,
                    ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond)
            );
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01)) ?
                currentReferenceAngle :
                desiredState.angle; //Prevent rotating module if speed is less than 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        currentReferenceAngle = angle;
    }

    public void resetToAbsolute(){
        double absolutePosition = getAbsoluteAngle().getDegrees() - angleOffset.getDegrees();
        relativeAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleMotor(){
        angleMotor.restoreFactoryDefaults();
        CANSparkUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT);
        angleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
        angleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);
        relativeAngleEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);
        angleController.setP(Constants.Swerve.ANGLE_P);
        angleController.setI(Constants.Swerve.ANGLE_I);
        angleController.setD(Constants.Swerve.ANGLE_D);
        angleController.setFF(Constants.Swerve.ANGLE_F);
        angleMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.restoreFactoryDefaults();
        CANSparkUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT);
        driveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        driveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR);
        driveController.setP(Constants.Swerve.DRIVE_P);
        driveController.setI(Constants.Swerve.DRIVE_I);
        driveController.setD(Constants.Swerve.DRIVE_D);
        driveController.setFF(Constants.Swerve.DRIVE_F);
        driveMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    // everything below here is fine.

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(relativeAngleEncoder.getPosition());
    }

    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromRotations(absoluteAngleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
                driveEncoder.getVelocity(),
                getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                getAngle()
        );
    }
}