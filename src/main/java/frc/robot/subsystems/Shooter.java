package frc.robot.subsystems;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final CANSparkFlex leftShooter;
    private final CANSparkFlex rightShooter;

    private final SparkPIDController leftPidController;
    private final SparkPIDController rightPidController;


    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    public Shooter() {
        leftShooter = new CANSparkFlex(Constants.Shooter.LEFT_SHOOTER_ID, MotorType.kBrushless);
        leftShooter.restoreFactoryDefaults();
        leftShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        leftShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        leftEncoder = leftShooter.getEncoder();

        rightShooter = new CANSparkFlex(Constants.Shooter.RIGHT_SHOOTER_ID, MotorType.kBrushless);
        rightShooter.restoreFactoryDefaults();
        rightShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        rightShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        rightEncoder = rightShooter.getEncoder();

        leftPidController = leftShooter.getPIDController();
        leftPidController.setP(Constants.Shooter.SHOOTER_P);
        leftPidController.setI(Constants.Shooter.SHOOTER_I);
        leftPidController.setD(Constants.Shooter.SHOOTER_D);

        rightPidController = rightShooter.getPIDController();
        rightPidController.setP(Constants.Shooter.SHOOTER_P);
        rightPidController.setI(Constants.Shooter.SHOOTER_I);
        rightPidController.setD(Constants.Shooter.SHOOTER_D);

        //leftPidController.setOutputRange(1, 1);
        //rightPidController.setOutputRange(1, 1);

        leftShooter.burnFlash();
        rightShooter.burnFlash();
    }

    public void shoot(double speed) {
        //TODO make sure this works lolololol
        if (speed <= 0) {
            leftShooter.set(0);
            rightShooter.set(0);
        } else {
            leftPidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
            rightPidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
            leftPidController.setOutputRange(0, 1);
            // leftShooter.set(speed);
            // rightShooter.set(speed);
        }
        
    }

    public double getLeftShooterSpeed(){
        return leftEncoder.getVelocity();
    }
    public double getRightShooterSpeed(){
        return rightEncoder.getVelocity();
    }

}
