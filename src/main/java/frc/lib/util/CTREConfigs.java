package frc.lib.util;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import frc.robot.constants.Configuration;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;
    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();

        angleSupplyLimit.withSupplyCurrentLimitEnable(configuration.angle_enable_supply_current_limit);
        angleSupplyLimit.withSupplyCurrentLimit(configuration.angle_continuous_supply_current_limit);
        angleSupplyLimit.withSupplyCurrentThreshold(configuration.angle_peak_supply_current_limit);
        angleSupplyLimit.withSupplyTimeThreshold(configuration.angle_peak_supply_current_duration);

        swerveAngleFXConfig.Slot0.withKP(configuration.angle_p);
        swerveAngleFXConfig.Slot0.withKI(configuration.angle_i);
        swerveAngleFXConfig.Slot0.withKD(configuration.angle_d);
        swerveAngleFXConfig.Slot0.withKV(configuration.angle_f);
        swerveAngleFXConfig.withCurrentLimits(angleSupplyLimit);

        /* Swerve Drive Motor Configuration */
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();

        driveSupplyLimit.withSupplyCurrentLimitEnable(configuration.drive_enable_supply_current_limit);
        driveSupplyLimit.withSupplyCurrentLimit(configuration.drive_continuous_supply_current_limit);
        driveSupplyLimit.withSupplyCurrentThreshold(configuration.drive_peak_supply_current_limit);
        driveSupplyLimit.withSupplyTimeThreshold(configuration.drive_peak_supply_current_duration);

        swerveDriveFXConfig.Slot0.withKP(configuration.drive_p);
        swerveDriveFXConfig.Slot0.withKI(configuration.drive_i);
        swerveDriveFXConfig.Slot0.withKD(configuration.drive_d);
        swerveDriveFXConfig.Slot0.withKV(configuration.drive_f);
        swerveDriveFXConfig.withCurrentLimits(driveSupplyLimit);
        swerveDriveFXConfig.withOpenLoopRamps(
                new OpenLoopRampsConfigs().withDutyCycleOpenLoopRampPeriod(configuration.open_loop_ramp)
        );
        swerveDriveFXConfig.withClosedLoopRamps(
                new ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(configuration.closed_loop_ramp)
        );
        
        /* Swerve CANCoder Configuration */
        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        sensorConfigs.withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1);
        sensorConfigs.withSensorDirection(configuration.cancoder_sensor_direction);
        swerveCanCoderConfig.withMagnetSensor(sensorConfigs);
    }
}