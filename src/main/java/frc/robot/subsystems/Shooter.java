package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final CANSparkFlex bottomShooter;
    private final CANSparkFlex topShooter;

    private final SparkPIDController bottomController, topController;

    private final RelativeEncoder bottomEncoder;
    private final RelativeEncoder topEncoder;
    double currentSpeed = 0;

    public Shooter() {
        bottomShooter = new CANSparkFlex(Constants.Shooter.BOTTOM_SHOOTER_ID, MotorType.kBrushless);
        bottomShooter.restoreFactoryDefaults();
        bottomShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        bottomShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        bottomEncoder = bottomShooter.getEncoder();

        topShooter = new CANSparkFlex(Constants.Shooter.TOP_SHOOTER_ID, MotorType.kBrushless);
        topShooter.restoreFactoryDefaults();
        topShooter.setSmartCurrentLimit(Constants.Shooter.SHOOTER_CURRENT_LIMIT);
        topShooter.setInverted(!Constants.Shooter.SHOOTER_INVERT);
        topEncoder = topShooter.getEncoder();

        topController = topShooter.getPIDController();
        bottomController = bottomShooter.getPIDController();

        topController.setP(Constants.Shooter.SHOOTER_P);
        topController.setI(Constants.Shooter.SHOOTER_I);
        topController.setD(Constants.Shooter.SHOOTER_D);
        topController.setFF(Constants.Shooter.SHOOTER_FF);

        bottomController.setP(Constants.Shooter.SHOOTER_P);
        bottomController.setI(Constants.Shooter.SHOOTER_I);
        bottomController.setD(Constants.Shooter.SHOOTER_D);
        bottomController.setFF(Constants.Shooter.SHOOTER_FF);
        
        bottomShooter.burnFlash();
        topShooter.burnFlash();
    }

    @Override
    public void periodic() {
        // if (getLeftShooterSpeed() >= currentSpeed - Constants.Shooter.SHOOTER_SPEED_TOLERANCE && getRightShooterSpeed() >= currentSpeed - Constants.Shooter.SHOOTER_SPEED_TOLERANCE) {
        //     intakeCarriage.setCarriageSpeed(0.5);
        // } else {
        //     intakeCarriage.setCarriageSpeed(0);
        // }
    }

    public double getLeftShooterSpeed() {
        return bottomEncoder.getVelocity();
    }

    public double getRightShooterSpeed() {
        return topEncoder.getVelocity();
    }

    public void shoot(double speed) {
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
        topController.setReference(speed, ControlType.kVelocity, 0, Constants.Shooter.SHOOTER_FF);
        bottomController.setReference(speed, ControlType.kVelocity, 0, Constants.Shooter.SHOOTER_FF);
    }

    public void setShooterIdle(boolean idleMode) {
        bottomShooter.set(idleMode ? Constants.Shooter.IDLE_SHOOTER_SPEED : 0);
        topShooter.set(idleMode ? Constants.Shooter.IDLE_SHOOTER_SPEED : 0);
    }

    public void overrideShooterSpeed(double speed) {
        topShooter.set(speed);
        bottomShooter.set(speed);
    }
}