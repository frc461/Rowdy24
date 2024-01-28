package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase;

public class Shooter extends SubsystemBase {

    private final CANSparkFlex leftShooter;
    private final CANSparkFlex rightShooter;

    private final CANSparkMax feeder;

    private final SparkPIDController leftPidController;
    private final SparkPIDController rightPidController;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private double currentSpeed = 0;

    public Shooter() {
        leftShooter = new CANSparkFlex(Constants.Shooter.LEFT_SHOOTER_ID, MotorType.kBrushless);
        leftShooter.restoreFactoryDefaults();
        leftShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        leftShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        leftEncoder = leftShooter.getExternalEncoder(7168);

        rightShooter = new CANSparkFlex(Constants.Shooter.RIGHT_SHOOTER_ID, MotorType.kBrushless);
        rightShooter.restoreFactoryDefaults();
        rightShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        rightShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        rightEncoder = rightShooter.getExternalEncoder(7168);

        leftPidController = leftShooter.getPIDController();
        leftPidController.setP(Constants.Shooter.SHOOTER_P);
        leftPidController.setI(Constants.Shooter.SHOOTER_I);
        leftPidController.setD(Constants.Shooter.SHOOTER_D);

        rightPidController = rightShooter.getPIDController();
        rightPidController.setP(Constants.Shooter.SHOOTER_P);
        rightPidController.setI(Constants.Shooter.SHOOTER_I);
        rightPidController.setD(Constants.Shooter.SHOOTER_D);

        leftPidController.setOutputRange(0, 1);
        rightPidController.setOutputRange(0, 1);

        leftShooter.burnFlash();
        rightShooter.burnFlash();

        feeder = new CANSparkMax(Constants.Shooter.FEEDER_ID, MotorType.kBrushed);
        feeder.restoreFactoryDefaults();
        feeder.setSmartCurrentLimit(Constants.Shooter.FEEDER_CURRENT_LIMIT);
        feeder.setInverted(Constants.Shooter.FEEDER_INVERT);
    }

    @Override
    public void periodic() {
        // if (getLeftShooterSpeed() >= currentSpeed - 50 && getRightShooterSpeed() >= currentSpeed - 50) {
        //     feeder.set(0.5);
        // } else {
        //     feeder.set(0);
        // }
    }

    public void shoot(double speed) {
        if (speed <= 0) {
            leftShooter.set(0);
            rightShooter.set(0);
            currentSpeed = 0;
        } else {
            leftPidController.setReference(speed, CANSparkBase.ControlType.kVelocity);
            rightPidController.setReference(speed, CANSparkBase.ControlType.kVelocity);
            currentSpeed = speed;
            // leftShooter.set(speed);
            // rightShooter.set(speed);
        }

    }

    public double getLeftShooterSpeed() {
        return leftEncoder.getVelocity();
    }

    public double getRightShooterSpeed() {
        return rightEncoder.getVelocity();
    }

}
