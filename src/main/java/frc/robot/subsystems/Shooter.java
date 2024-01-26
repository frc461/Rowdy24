package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;

    private final SparkMaxPIDController leftPidController;
    private final SparkMaxPIDController rightPidController;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final CANSparkMax shooterAngler;
    private final PIDController anglerPID;
    private final RelativeEncoder anglerEncoder;
    private double target;

    public Shooter() {
        leftShooter = new CANSparkMax(Constants.Shooter.LEFT_SHOOTER_ID, MotorType.kBrushless);
        leftShooter.restoreFactoryDefaults();
        leftShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        leftShooter.setInverted(Constants.Shooter.SHOOTER_INVERT);
        leftEncoder = leftShooter.getEncoder();

        rightShooter = new CANSparkMax(Constants.Shooter.RIGHT_SHOOTER_ID, MotorType.kBrushless);
        rightShooter.restoreFactoryDefaults();
        rightShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        rightShooter.setInverted(Constants.Shooter.SHOOTER_INVERT);
        rightEncoder = rightShooter.getEncoder();

        leftPidController = leftShooter.getPIDController();
        leftPidController.setP(Constants.Shooter.SHOOTER_P);
        leftPidController.setI(Constants.Shooter.SHOOTER_I);
        leftPidController.setD(Constants.Shooter.SHOOTER_D);

        rightPidController = rightShooter.getPIDController();
        rightPidController.setP(Constants.Shooter.SHOOTER_P);
        rightPidController.setI(Constants.Shooter.SHOOTER_I);
        rightPidController.setD(Constants.Shooter.SHOOTER_D);

        shooterAngler = new CANSparkMax(Constants.Shooter.ANGLER_ID, MotorType.kBrushless);
        shooterAngler.restoreFactoryDefaults();
        shooterAngler.setSmartCurrentLimit(Constants.Shooter.ANGLER_CURRENT_LIMIT);
        shooterAngler.setInverted(Constants.Shooter.ANGLER_INVERT);
        anglerPID = new PIDController(0, 0, 0);
        anglerPID.setP(Constants.Shooter.ANGLER_P);
        anglerPID.setI(Constants.Shooter.ANGLER_I);
        anglerPID.setD(Constants.Shooter.ANGLER_D);
        anglerEncoder = shooterAngler.getEncoder();

        target = Constants.Shooter.ANGLER_UPPER_LIMIT;
    }

    public void shoot(double speed) {
        //TODO make sure this works lolololol
        leftPidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
        rightPidController.setReference(speed, CANSparkMax.ControlType.kVelocity);
    }

    public void tiltShooter(double angle) {
        //TODO: not sure if this is the correct conversion
        double angleToRot = (angle*42)/(360);

        if (angleToRot < anglerEncoder.getPosition() && anglerEncoder.getPosition() < Constants.Shooter.ANGLER_LOWER_LIMIT) {
            angleToRot = Constants.Shooter.ANGLER_LOWER_LIMIT;
        } else if (angleToRot > anglerEncoder.getPosition() && anglerEncoder.getPosition() > Constants.Shooter.ANGLER_UPPER_LIMIT) {
            angleToRot = Constants.Shooter.ANGLER_UPPER_LIMIT;
        }
        target = angleToRot;
        shooterAngler.set(anglerPID.calculate(anglerEncoder.getPosition(), angleToRot));
    }

    public void holdTilt() {
        if(target < Constants.Shooter.ANGLER_LOWER_LIMIT){
            target = Constants.Shooter.ANGLER_LOWER_LIMIT;
        }
        shooterAngler.set(anglerPID.calculate(anglerEncoder.getPosition(), target));
    }

}
