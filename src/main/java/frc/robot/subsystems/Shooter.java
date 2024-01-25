package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    private final CANSparkMax leftShooter = new CANSparkMax(Constants.Shooter.LEFT_SHOOTER_ID, MotorType.kBrushless);
    private final CANSparkMax rightShooter = new CANSparkMax(Constants.Shooter.RIGHT_SHOOTER_ID, MotorType.kBrushless);

    private final SparkMaxPIDController leftPidController = leftShooter.getPIDController();
    private final SparkMaxPIDController rightPidController = rightShooter.getPIDController();


    private final RelativeEncoder leftEncoder = leftShooter.getEncoder();
    private final RelativeEncoder rightEncoder = rightShooter.getEncoder();

    private final CANSparkMax shooterAngler = new CANSparkMax(Constants.Shooter.ANGLER_ID, MotorType.kBrushless);
    private final SparkMaxPIDController anglerPID = shooterAngler.getPIDController();
    private final RelativeEncoder anglerEncoder = shooterAngler.getEncoder();


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

        shooterAngler.restoreFactoryDefaults();
        shooterAngler.setSmartCurrentLimit(Constants.Shooter.ANGLER_CURRENT_LIMIT);
        shooterAngler.setInverted(Constants.Shooter.ANGLER_INVERT);
        anglerPID.setP(Constants.Shooter.ANGLER_P);
        anglerPID.setI(Constants.Shooter.ANGLER_I);
        anglerPID.setD(Constants.Shooter.ANGLER_D);
    }

    public void shoot(double speed) {
        //TODO make sure this works lolololol
        leftPidController.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
        rightPidController.setReference(speed, CANSparkMax.ControlType.kDutyCycle);
    }

    public void tiltShooter(double angle) {
        anglerPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

}
