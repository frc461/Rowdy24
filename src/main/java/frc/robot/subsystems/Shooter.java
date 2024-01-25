package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final CANSparkMax leftShooter = new CANSparkMax(Constants.Shooter.LEFT_SHOOTER_ID, MotorType.kBrushless);
    private final CANSparkMax rightShooter = new CANSparkMax(Constants.Shooter.RIGHT_SHOOTER_ID, MotorType.kBrushless);

    private final SparkMaxPIDController leftPidController = leftShooter.getPIDController();
    private final SparkMaxPIDController rightPidController = rightShooter.getPIDController();


    private final RelativeEncoder leftEncoder = leftShooter.getEncoder();
    private final RelativeEncoder rightEncoder = rightShooter.getEncoder();

    public Shooter() {
        leftShooter.restoreFactoryDefaults();
        leftShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        leftShooter.setInverted(Constants.Shooter.SHOOTER_INVERT);
        rightShooter.restoreFactoryDefaults();
        rightShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        rightShooter.setInverted(Constants.Shooter.SHOOTER_INVERT);

        leftPidController.setP(Constants.Shooter.SHOOTER_P);
        leftPidController.setI(Constants.Shooter.SHOOTER_I);
        leftPidController.setD(Constants.Shooter.SHOOTER_D);

        rightPidController.setP(Constants.Shooter.SHOOTER_P);
        rightPidController.setI(Constants.Shooter.SHOOTER_I);
        rightPidController.setD(Constants.Shooter.SHOOTER_D);
    }

    public void shoot(double speed) {
        //TODO make sure this works lolololol
        leftPidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
        rightPidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

}
