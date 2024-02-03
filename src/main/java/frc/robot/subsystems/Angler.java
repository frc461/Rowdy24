package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Angler extends SubsystemBase {
    private final CANSparkMax angler;
    private double target;
    private PIDController pidController;
    private RelativeEncoder encoder;

    public Angler() {
        angler = new CANSparkMax(Constants.Shooter.ANGLER_ID, MotorType.kBrushless);
        angler.setSmartCurrentLimit(Constants.Shooter.ANGLER_CURRENT_LIMIT);
        angler.setInverted(Constants.Shooter.ANGLER_INVERT); 

        pidController = new PIDController(0, 0, 0);
        pidController.setP(Constants.Shooter.ANGLER_P);
        pidController.setI(Constants.Shooter.ANGLER_I);
        pidController.setD(Constants.Shooter.ANGLER_D);

        encoder = angler.getEncoder();
        encoder.setPosition(0);
        
        while(!lowerSwitchTriggered()) {
            angler.set(-0.5);
        }

        target = Constants.Shooter.ANGLER_LOWER_LIMIT;
    }


    public boolean lowerSwitchTriggered() { 
        return angler.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public boolean upperSwitchTriggered() {
        return angler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public void checkLimitSwitches() {
        if (upperSwitchTriggered()) {
            encoder.setPosition(Constants.Shooter.ANGLER_UPPER_LIMIT);
        }
        if (lowerSwitchTriggered()) {
            encoder.setPosition(Constants.Shooter.ANGLER_LOWER_LIMIT);
        }
    }

    public void holdTilt() {
        angler.set(pidController.calculate(encoder.getPosition(), target));
    }

    public void moveAngle(double axisValue) {
        target = encoder.getPosition();
        if (axisValue < 0 && lowerSwitchTriggered()){
            target = Constants.Shooter.ANGLER_LOWER_LIMIT;
            holdTilt();
        } else if (axisValue > 0 && upperSwitchTriggered()) {
            target = Constants.Shooter.ANGLER_UPPER_LIMIT;
            holdTilt();
        }else{
            angler.set(axisValue);
        }
    }

    public void setAngle(double angle) {
        if (angle < encoder.getPosition() && lowerSwitchTriggered()) {
            encoder.setPosition(Constants.Shooter.ANGLER_LOWER_LIMIT);
            angle = Constants.Shooter.ANGLER_LOWER_LIMIT;
        } else if (angle > encoder.getPosition() && encoder.getPosition() > Constants.Shooter.ANGLER_UPPER_LIMIT) {
            angle = Constants.Shooter.ANGLER_UPPER_LIMIT;
        }
        angler.set(pidController.calculate(encoder.getPosition(), angle));
        target = angle;
    }

    public double getEncoder() {
        return encoder.getPosition(); 
    }
}