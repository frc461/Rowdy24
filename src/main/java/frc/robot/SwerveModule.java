package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;


    private RelativeEncoder driveEncoder;
    private RelativeEncoder integratedAngleEncoder;
    private CANCoder angleEncoder;

    private SparkMaxPIDController driveController;
    private SparkMaxPIDController angleController;

    private SwerveModulePosition currentPosition = new SwerveModulePosition();
    private SwerveModuleState currentState = new SwerveModuleState();

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.DRIVE_KS, Constants.Swerve.DRIVE_KV, Constants.Swerve.DRIVE_KA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        integratedAngleEncoder = mAngleMotor.getEncoder();
        angleController = mAngleMotor.getPIDController();
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = mDriveMotor.getEncoder();
        driveController = mDriveMotor.getPIDController();
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.MAX_SPEED;
            mDriveMotor.set(percentOutput);    
        }
        else {
            driveController.setReference(
            desiredState.speedMetersPerSecond,
            ControlType.kVelocity,
            0,
            feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    public void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleController.setReference(angle.getDegrees(), ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    }

    private void configAngleMotor(){
        
        mAngleMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mAngleMotor, Usage.kPositionOnly);
        mAngleMotor.setSmartCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_CURRENT_LIMIT);
        mAngleMotor.setInverted(Constants.Swerve.ANGLE_MOTOR_INVERT);
        mAngleMotor.setIdleMode(Constants.Swerve.ANGLE_NEUTRAL_MODE);
        integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.ANGLE_CONVERSION_FACTOR);
        angleController.setP(Constants.Swerve.ANGLE_KP);
        angleController.setI(Constants.Swerve.ANGLE_KI);
        angleController.setD(Constants.Swerve.ANGLE_KD);
        angleController.setFF(Constants.Swerve.ANGLE_KF);
        mAngleMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
        mAngleMotor.burnFlash();
        resetToAbsolute();
    }

    private void configDriveMotor(){        

        mDriveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(mDriveMotor, Usage.kAll);
        mDriveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_CURRENT_LIMIT);
        mDriveMotor.setInverted(Constants.Swerve.DRIVE_MOTOR_INVERT);
        mDriveMotor.setIdleMode(Constants.Swerve.DRIVE_NEUTRAL_MODE);
        driveEncoder.setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_VELOCITY_FACTOR);
        driveEncoder.setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_POSITION_FACTOR);
        driveController.setP(Constants.Swerve.ANGLE_KP);
        driveController.setI(Constants.Swerve.ANGLE_KI);
        driveController.setD(Constants.Swerve.ANGLE_KD);
        driveController.setFF(Constants.Swerve.ANGLE_KF);
        mDriveMotor.enableVoltageCompensation(Constants.Swerve.VOLTAGE_COMP);
        mDriveMotor.burnFlash();
        driveEncoder.setPosition(0.0);
    }

    //everything below here is fine.

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