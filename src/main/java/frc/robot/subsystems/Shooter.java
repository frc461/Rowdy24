package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configuration;
import frc.robot.RobotConstants;
import frc.robot.RobotIdentity;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex bottomShooter;
    private final CANSparkFlex topShooter;

    private final SparkPIDController bottomController, topController;

    private final RelativeEncoder bottomEncoder;
    private final RelativeEncoder topEncoder;
    private double target;
    private double error;    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    public Shooter() {
        bottomShooter = new CANSparkFlex(configuration.bottom_shooter_id, MotorType.kBrushless);
        bottomShooter.restoreFactoryDefaults();
        bottomShooter.setSmartCurrentLimit(configuration.shooter_current_limit);
        bottomShooter.setInverted(!configuration.shooter_invert);
        bottomEncoder = bottomShooter.getEncoder();

        topShooter = new CANSparkFlex(configuration.top_shooter_id, MotorType.kBrushless);
        topShooter.restoreFactoryDefaults();
        topShooter.setSmartCurrentLimit(configuration.shooter_current_limit);
        topShooter.setInverted(!configuration.shooter_invert);
        topEncoder = topShooter.getEncoder();

        topController = topShooter.getPIDController();
        bottomController = bottomShooter.getPIDController();

        topController.setP(configuration.shooter_p);
        topController.setI(configuration.shooter_i);
        topController.setD(configuration.shooter_d);
        topController.setFF(configuration.shooter_ff);

        bottomController.setP(configuration.shooter_p);
        bottomController.setI(configuration.shooter_i);
        bottomController.setD(configuration.shooter_d);
        bottomController.setFF(configuration.shooter_ff);
        
        bottomShooter.burnFlash();
        topShooter.burnFlash();

        target = 0.0;
        error = Math.abs(target - (getBottomShooterSpeed() + getTopShooterSpeed()) / 2);
    }

    @Override
    public void periodic() {
        error = Math.abs(target - (getBottomShooterSpeed() + getTopShooterSpeed()) / 2);
    }

    public double getBottomShooterSpeed() {
        return bottomEncoder.getVelocity();
    }

    public double getTopShooterSpeed() {
        return topEncoder.getVelocity();
    }

    public double getError() {
        return error;
    }

    public void shoot(double speed) {
        target = speed;
        topController.setReference(speed, ControlType.kVelocity, 0, configuration.shooter_ff);
        bottomController.setReference(speed, ControlType.kVelocity, 0, configuration.shooter_ff);
    }

    public boolean minimalError() {
        return error < configuration.shooter_error_tolerance && (getBottomShooterSpeed() + getTopShooterSpeed()) / 2 > 5000;
    }

    public void setShooterIdle(boolean idleMode) {
        bottomShooter.set(idleMode ? configuration.idle_shooter_speed : 0);
        topShooter.set(idleMode ? configuration.idle_shooter_speed : 0);
    }

    public void overrideShooterSpeed(double speed) {
        topShooter.set(speed);
        bottomShooter.set(speed);
    }
}