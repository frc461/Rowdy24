package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {

    private final CANSparkFlex leftShooter;
    private final CANSparkFlex rightShooter;

    private final CANSparkMax feeder;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    double currentSpeed = 0;

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

        leftShooter.burnFlash();
        rightShooter.burnFlash();

        feeder = new CANSparkMax(Constants.Shooter.FEEDER_ID, MotorType.kBrushed);
        feeder.restoreFactoryDefaults();
        feeder.setSmartCurrentLimit(Constants.Shooter.FEEDER_CURRENT_LIMIT);
        feeder.setInverted(Constants.Shooter.FEEDER_INVERT);
    }

    @Override
    public void periodic() {
        // if (getLeftShooterSpeed() >= currentSpeed - Constants.Shooter.SHOOTER_SPEED_TOLERANCE && getRightShooterSpeed() >= currentSpeed - Constants.Shooter.SHOOTER_SPEED_TOLERANCE) {
        //     feeder.set(0.5);
        // } else {
        //     feeder.set(0);
        // }
    }

    public void shoot(double speed) {
        leftShooter.set(speed);
        rightShooter.set(speed);
        currentSpeed = speed * 6750;

        //is the shooter up to speed? if so, alert the operator
        //TODO: this might be problematic in auto...
        if (leftEncoder.getVelocity() <= speed + Constants.Shooter.SHOOTER_SPEED_TOLERANCE && leftEncoder.getVelocity() >= speed - Constants.Shooter.SHOOTER_SPEED_TOLERANCE){
            SmartDashboard.putBoolean("Shooter Ready", true);
            RobotContainer.operator.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
        } else{
            RobotContainer.operator.setRumble(GenericHID.RumbleType.kBothRumble, 0);
            SmartDashboard.putBoolean("Shooter Ready", false);
        }
    }

    public double getLeftShooterSpeed() {
        return leftEncoder.getVelocity();
    }

    public double getRightShooterSpeed() {
        return rightEncoder.getVelocity();
    }

}
