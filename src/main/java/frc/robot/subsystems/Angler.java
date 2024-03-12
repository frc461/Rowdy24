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
    private final DigitalInput lowerLimitSwitch;
    private double target, error, accuracy;

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
        lowerLimitSwitch = new DigitalInput(Constants.Angler.ANGLER_LOWER_LIMIT_SWITCH);
        upperMagnetLimitSwitch = angler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        target = 0.0;
        error = 0.0;
        accuracy = 1.0;
    }

    @Override
    public void periodic() {
        error = Math.abs(target - getPosition());
        // TODO: change accuracy + create accuracy check using turret
        accuracy = Limelight.tagExists() ? (target > getPosition()) ?
                getPosition() / target : target / getPosition() : -1.0;
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
        return !lowerLimitSwitch.get();
    }

    public void checkLimitSwitches() {
       if (lowerSwitchTriggered()) {
           encoder.setPosition(Constants.Angler.ANGLER_LOWER_LIMIT);
       }
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

    public void setEncoderVal(double encoderVal) {
        checkLimitSwitches();
        encoderVal = Math.max(
                Constants.Angler.ANGLER_LOWER_LIMIT,
                Math.min(encoderVal, Constants.Angler.ANGLER_UPPER_LIMIT)
        );
        target = encoderVal;
        holdTarget();
    }
}