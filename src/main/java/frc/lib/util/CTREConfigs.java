package frc.lib.util;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import frc.robot.Constants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();

        angleSupplyLimit.withSupplyCurrentLimitEnable(Constants.Swerve.ANGLE_ENABLE_SUPPLY_CURRENT_LIMIT);
        angleSupplyLimit.withSupplyCurrentLimit(Constants.Swerve.ANGLE_CONTINUOUS_SUPPLY_CURRENT_LIMIT);
        angleSupplyLimit.withSupplyCurrentThreshold(Constants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_LIMIT);
        angleSupplyLimit.withSupplyTimeThreshold(Constants.Swerve.ANGLE_PEAK_SUPPLY_CURRENT_DURATION);

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.ANGLE_P;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.ANGLE_I;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.ANGLE_D;
        swerveAngleFXConfig.Slot0.kV = Constants.Swerve.ANGLE_F;
        swerveAngleFXConfig.CurrentLimits = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();

        driveSupplyLimit.withSupplyCurrentLimitEnable(Constants.Swerve.DRIVE_ENABLE_SUPPLY_CURRENT_LIMIT);
        driveSupplyLimit.withSupplyCurrentLimit(Constants.Swerve.DRIVE_CONTINUOUS_SUPPLY_CURRENT_LIMIT);
        driveSupplyLimit.withSupplyCurrentThreshold(Constants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_LIMIT);
        driveSupplyLimit.withSupplyTimeThreshold(Constants.Swerve.DRIVE_PEAK_SUPPLY_CURRENT_DURATION);

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.DRIVE_P;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.DRIVE_I;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.DRIVE_D;
        swerveDriveFXConfig.Slot0.kV = Constants.Swerve.DRIVE_F;
        swerveDriveFXConfig.CurrentLimits = driveSupplyLimit;
        swerveDriveFXConfig.OpenLoopRamps = new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(
                Constants.Swerve.OPEN_LOOP_RAMP);
        swerveDriveFXConfig.ClosedLoopRamps = new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(
                Constants.Swerve.CLOSED_LOOP_RAMP);
        
        /* Swerve CANCoder Configuration */
        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        sensorConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf);
        sensorConfigs.withSensorDirection(Constants.Swerve.CANCODER_SENSOR_DIRECTION);
        swerveCanCoderConfig.MagnetSensor = sensorConfigs;
    }
}