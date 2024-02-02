package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeCarriage;

public class Shooter extends SubsystemBase {

    private final IntakeCarriage intakeCarriage = new IntakeCarriage();

    private final CANSparkMax leftShooter;
    private final CANSparkMax rightShooter;

    private final SparkPIDController leftController, rightController;

    //private final CANSparkMax feeder;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    double currentSpeed = 0;

    public Shooter() {
        leftShooter = new CANSparkMax(Constants.Shooter.LEFT_SHOOTER_ID, MotorType.kBrushless);
        leftShooter.restoreFactoryDefaults();
        leftShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        leftShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        leftEncoder = leftShooter.getEncoder();

        rightShooter = new CANSparkMax(Constants.Shooter.RIGHT_SHOOTER_ID, MotorType.kBrushless);
        rightShooter.restoreFactoryDefaults();
        rightShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        rightShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        rightEncoder = rightShooter.getEncoder();

        rightController = rightShooter.getPIDController();
        leftController = leftShooter.getPIDController();

        rightController.setP(Constants.Shooter.SHOOTER_P);
        rightController.setI(Constants.Shooter.SHOOTER_I);
        rightController.setD(Constants.Shooter.SHOOTER_D);
        rightController.setFF(Constants.Shooter.SHOOTER_FF);

        leftController.setP(Constants.Shooter.SHOOTER_P);
        leftController.setI(Constants.Shooter.SHOOTER_I);
        leftController.setD(Constants.Shooter.SHOOTER_D);
        leftController.setFF(Constants.Shooter.SHOOTER_FF);
        
        leftShooter.burnFlash();
        rightShooter.burnFlash();

        // feeder = new CANSparkMax(Constants.Shooter.FEEDER_ID, MotorType.kBrushed);
        // feeder.restoreFactoryDefaults();
        // feeder.setSmartCurrentLimit(Constants.Shooter.FEEDER_CURRENT_LIMIT);
        // feeder.setInverted(Constants.Shooter.FEEDER_INVERT);
    }

    @Override
    public void periodic() {
        if (getLeftShooterSpeed() >= currentSpeed - Constants.Shooter.SHOOTER_SPEED_TOLERANCE && getRightShooterSpeed() >= currentSpeed - Constants.Shooter.SHOOTER_SPEED_TOLERANCE) {
            intakeCarriage.setCarriageSpeed(0.5);
        } else {
            intakeCarriage.setCarriageSpeed(0);
        }
    }

    public double getLeftShooterSpeed() {
        return leftEncoder.getVelocity();
    }

    public double getRightShooterSpeed() {
        return rightEncoder.getVelocity();
    }

    public void shoot(double speed, boolean idleShooter) {
        // //is the shooter up to speed? if so, alert the operator
        // //TODO: this might be problematic in auto...
        // if (leftEncoder.getVelocity() <= speed + Constants.Shooter.SHOOTER_SPEED_TOLERANCE && leftEncoder.getVelocity() >= speed - Constants.Shooter.SHOOTER_SPEED_TOLERANCE){
        //     SmartDashboard.putBoolean("Shooter Ready", true);
        //     RobotContainer.operator.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        // } else{
        //     RobotContainer.operator.setRumble(GenericHID.RumbleType.kBothRumble, 0);
        //     SmartDashboard.putBoolean("Shooter Ready", false);
        // }
        currentSpeed = speed;
            
        if (idleShooter) {
            leftShooter.set(speed);
            rightShooter.set(speed);
        } else {
            rightController.setReference(speed, ControlType.kVelocity, 0, Constants.Shooter.SHOOTER_FF);
            leftController.setReference(speed, ControlType.kVelocity, 0, Constants.Shooter.SHOOTER_FF);
        }
    }
}
