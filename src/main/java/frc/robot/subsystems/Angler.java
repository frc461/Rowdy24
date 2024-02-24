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
    private final SparkLimitSwitch lowerLimitSwitch, upperLimitSwitch;
    private final PIDController pidController;
    private final RelativeEncoder encoder;
    private double target, error;

    public Angler() {
        angler = new CANSparkMax(Constants.Angler.ANGLER_ID, MotorType.kBrushless);
        angler.restoreFactoryDefaults();
        angler.setSmartCurrentLimit(Constants.Angler.ANGLER_CURRENT_LIMIT);
        angler.setInverted(Constants.Angler.ANGLER_INVERT);
        encoder = angler.getEncoder();

        lowerLimitSwitch = angler.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        upperLimitSwitch = angler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        pidController = new PIDController(
                Constants.Angler.ANGLER_P,
                Constants.Angler.ANGLER_I,
                Constants.Angler.ANGLER_D
        );

        target = 0.0;
        error = 1000;
    }

    @Override
    public void periodic() {
        error = Math.abs(target - getPosition());
    }

    public double getPosition() { 
        return encoder.getPosition();
    }

    public double getTarget() {
        return target;
    }

    public double anglerPower() {
        return angler.getAppliedOutput();
    }

    public double getError() {
        return error;
    }

    public boolean lowerSwitchTriggered() { 
        return lowerLimitSwitch.isPressed();
    }

    public boolean upperSwitchTriggered() {
        return upperLimitSwitch.isPressed();
    }

    public void checkLimitSwitches() {
        if (upperSwitchTriggered()) {
            encoder.setPosition(Constants.Angler.ANGLER_UPPER_LIMIT);
        }
        if (lowerSwitchTriggered()) {
            encoder.setPosition(Constants.Angler.ANGLER_LOWER_LIMIT);
        }
    }

    public void holdTarget() {
        checkLimitSwitches();
        angler.set(pidController.calculate(encoder.getPosition(), target));
    }

    public void moveAngle(double axisValue) {
        target = encoder.getPosition();
        checkLimitSwitches();
        if (axisValue < 0 && lowerSwitchTriggered()) {
            target = Constants.Angler.ANGLER_LOWER_LIMIT;
            holdTarget();
        } else if (axisValue > 0 && upperSwitchTriggered()) {
            target = Constants.Angler.ANGLER_UPPER_LIMIT;
            holdTarget();
        } else {
            angler.set(axisValue);
        }
    }

    public void setAngle(double encoderVal) {
        checkLimitSwitches();
        encoderVal = (encoderVal < encoder.getPosition() && lowerSwitchTriggered()) ?
                Constants.Angler.ANGLER_LOWER_LIMIT :
                (encoderVal > encoder.getPosition() && upperSwitchTriggered()) ?
                Constants.Angler.ANGLER_UPPER_LIMIT : encoderVal;
        target = encoderVal;
        holdTarget();
    }

    public void setAlignedAngle(double x, double z, boolean tag) {
        double dist = Math.hypot(x, z);
        if (tag) {
            if (dist > Constants.Angler.UPPER_BOUND_LIMIT) {
                setAngle(Math.min(
                        34.8 - 11.5 * dist + 1.04 * Math.pow(dist, 2), Constants.Angler.ANGLER_UPPER_LIMIT
                ));
            } else {
                setAngle(Math.min(
                        31.4 - 10.6 * dist + 0.984 * Math.pow(dist, 2), Constants.Angler.ANGLER_UPPER_LIMIT
                ));
            }
        } else {
            target = getPosition();
        }
    }
}