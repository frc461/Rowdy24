package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
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
import frc.robot.Robot;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;
import frc.robot.constants.Configuration;

public class SwerveModule {
    public int moduleNumber;
    private final Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private final CANSparkMax angleMotor;
    private final CANSparkMax driveMotor;


    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder integratedAngleEncoder;
    private final CANcoder angleEncoder;

    private final SparkPIDController driveController;
    private final SparkPIDController angleController;
    private final SimpleMotorFeedforward feedforward;

    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = angleMotor.getEncoder();
        angleController = angleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = driveMotor.getEncoder();
        driveController = driveMotor.getPIDController();
        configDriveMotor();

        feedforward = new SimpleMotorFeedforward(
                configuration.drive_s, configuration.drive_v, configuration.drive_a
        );

        lastAngle = Rotation2d.fromDegrees(getAbsoluteAngle().getDegrees() - angleOffset.getDegrees());
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
            double percentOutput = desiredState.speedMetersPerSecond / configuration.max_speed;
            driveMotor.set(percentOutput);
        } else {
            driveController.setReference(
            desiredState.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getAbsoluteAngle(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (configuration.max_speed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    public void resetToAbsolute(){
        double absolutePosition = getAbsoluteAngle().getDegrees() - angleOffset.getDegrees();
        integratedAngleEncoder.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){
        angleEncoder.getConfigurator().apply(new CANcoderConfiguration());
        angleEncoder.getPosition().setUpdateFrequency(10);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){

        angleMotor.restoreFactoryDefaults();
        CANSparkUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
        angleMotor.setSmartCurrentLimit(configuration.angle_continuous_supply_current_limit);
        angleMotor.setInverted(configuration.angle_motor_invert);
        angleMotor.setIdleMode(configuration.angle_neutral_mode);
        integratedAngleEncoder.setPositionConversionFactor(configuration.angle_conversion_factor);
        angleController.setP(configuration.angle_p);
        angleController.setI(configuration.angle_i);
        angleController.setD(configuration.angle_d);
        angleController.setFF(configuration.angle_f);
        angleMotor.enableVoltageCompensation(configuration.voltage_comp);
        angleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor() {

        driveMotor.restoreFactoryDefaults();
        CANSparkUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(configuration.drive_continuous_supply_current_limit);
        driveMotor.setInverted(configuration.drive_motor_invert);
        driveMotor.setIdleMode(configuration.drive_neutral_mode);
        driveEncoder.setVelocityConversionFactor(configuration.drive_conversion_velocity_factor);
        driveEncoder.setPositionConversionFactor(configuration.drive_conversion_position_factor);
        driveController.setP(configuration.drive_p);
        driveController.setI(configuration.drive_i);
        driveController.setD(configuration.drive_d);
        driveController.setFF(configuration.drive_f);
        driveMotor.enableVoltageCompensation(configuration.voltage_comp);
        driveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    // everything below here is fine.

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