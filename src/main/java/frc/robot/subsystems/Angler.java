package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Angler extends SubsystemBase {
    private final CANSparkMax angler;
    private final RelativeEncoder encoder;
    private final SparkPIDController anglerPIDController;
    private final SparkLimitSwitch lowerMagnetLimitSwitch, upperMagnetLimitSwitch;
//    private final DigitalInput lowerLimitSwitch;
    private double target, error;

    public Angler() {
        angler = new CANSparkMax(Constants.Angler.ANGLER_ID, MotorType.kBrushless);
        angler.restoreFactoryDefaults();
        angler.setSmartCurrentLimit(Constants.Angler.ANGLER_CURRENT_LIMIT);
        angler.setInverted(Constants.Angler.ANGLER_INVERT);
        encoder = angler.getEncoder();

        anglerPIDController = angler.getPIDController();
        anglerPIDController.setP(Constants.Angler.ANGLER_P);
        anglerPIDController.setI(Constants.Angler.ANGLER_I);
        anglerPIDController.setD(Constants.Angler.ANGLER_D);
        angler.burnFlash();

        lowerMagnetLimitSwitch = angler.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
//        lowerLimitSwitch = new DigitalInput(Constants.Angler.ANGLER_LOWER_LIMIT_SWITCH);
        upperMagnetLimitSwitch = angler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        target = 0.0;
        error = Math.abs(target - getPosition());

        encoder.setPosition(0.0); // temp solution
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
        return lowerMagnetLimitSwitch.isPressed();
    }

    public void checkLimitSwitches() {
//        if (lowerSwitchTriggered()) {
//            encoder.setPosition(Constants.Angler.ANGLER_LOWER_LIMIT);
//        }
    }

    public void holdTarget() {
        checkLimitSwitches();
        anglerPIDController.setReference(target, CANSparkBase.ControlType.kPosition);
    }

    public void moveAngle(double axisValue) {
        checkLimitSwitches();
        if (axisValue < 0 && lowerSwitchTriggered()) {
            target = Constants.Angler.ANGLER_LOWER_LIMIT;
            holdTarget();
        } else if (axisValue > 0 && getPosition() > Constants.Angler.ANGLER_UPPER_LIMIT) {
            target = Constants.Angler.ANGLER_UPPER_LIMIT;
            holdTarget();
        } else {
            angler.set(axisValue);
            target = encoder.getPosition();
        }
    }

    public void setAngle(double encoderVal) {
        checkLimitSwitches();
        encoderVal = (encoderVal < encoder.getPosition() && lowerSwitchTriggered()) ?
                Constants.Angler.ANGLER_LOWER_LIMIT :
                (encoderVal > encoder.getPosition() && getPosition() > Constants.Angler.ANGLER_UPPER_LIMIT) ?
                Constants.Angler.ANGLER_UPPER_LIMIT : encoderVal;
        target = encoderVal;
        holdTarget();
    }

    public void setAlignedAngle(double x, double z, boolean tag) {
        if (tag) {
//            setAngle(Math.min(
//                    (dist < Constants.Angler.AVG_BOUND_LIMIT) ?
//                            Constants.Angler.AVG_BOUND_CONSTANT + Constants.Angler.AVG_BOUND_LINEAR_COEFFICIENT * dist + Constants.Angler.AVG_BOUND_SQUARED_COEFFICIENT * Math.pow(dist, 2) :
//                            Constants.Angler.UPPER_BOUND_CONSTANT + Constants.Angler.UPPER_BOUND_LINEAR_COEFFICIENT * dist + Constants.Angler.UPPER_BOUND_SQUARED_COEFFICIENT * Math.pow(dist, 2) - 1,
//                    Constants.Angler.ANGLER_UPPER_LIMIT
//            ));
            setAngle(Math.min(Constants.Angler.AUTO_ANGLER_AIM_EQUATION.apply(x, z), Constants.Angler.ANGLER_UPPER_LIMIT + Constants.Angler.ANGLER_TRIM));
        }
    }
}