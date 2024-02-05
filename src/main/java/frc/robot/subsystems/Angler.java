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
    private final PIDController pidController;
    private final RelativeEncoder encoder;
    private double target;

    public Angler() {
        angler = new CANSparkMax(Constants.Angler.ANGLER_ID, MotorType.kBrushless);

        angler.restoreFactoryDefaults();
        angler.setSmartCurrentLimit(Constants.Angler.ANGLER_CURRENT_LIMIT);
        angler.setInverted(Constants.Angler.ANGLER_INVERT);

        encoder = angler.getEncoder();
        encoder.setPosition(Constants.Angler.ANGLER_LOWER_LIMIT);
        
        while(!lowerSwitchTriggered()) {
            angler.set(-0.5);
        }

        pidController = new PIDController(
                Constants.Angler.ANGLER_P,
                Constants.Angler.ANGLER_I,
                Constants.Angler.ANGLER_D
        );

        target = encoder.getPosition();
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getTarget() {
        return target;
    }

    public double elevatorPower() {
        return angler.getAppliedOutput();
    }

    public boolean lowerSwitchTriggered() { 
        return angler.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public boolean upperSwitchTriggered() {
        return angler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public void checkLimitSwitches() {
        if (upperSwitchTriggered()) {
            encoder.setPosition(Constants.Angler.ANGLER_UPPER_LIMIT);
        }
        if (lowerSwitchTriggered()) {
            encoder.setPosition(Constants.Angler.ANGLER_LOWER_LIMIT);
        }
    }

    public void holdTilt() {
        angler.set(pidController.calculate(encoder.getPosition(), target));
    }

    public void moveAngle(double axisValue) {
        target = encoder.getPosition();
        if (axisValue < 0 && lowerSwitchTriggered()){
            target = Constants.Angler.ANGLER_LOWER_LIMIT;
            holdTilt();
        } else if (axisValue > 0 && upperSwitchTriggered()) {
            target = Constants.Angler.ANGLER_UPPER_LIMIT;
            holdTilt();
        }else{
            angler.set(axisValue);
        }
    }

    public void setAngle(double angle) {
        if (angle < encoder.getPosition() && lowerSwitchTriggered()) {
            encoder.setPosition(Constants.Angler.ANGLER_LOWER_LIMIT);
            angle = Constants.Angler.ANGLER_LOWER_LIMIT;
        } else if (angle > encoder.getPosition() && encoder.getPosition() > Constants.Angler.ANGLER_UPPER_LIMIT) {
            angle = Constants.Angler.ANGLER_UPPER_LIMIT;
        }
        angler.set(pidController.calculate(encoder.getPosition(), angle));
        target = angle;
    }
}