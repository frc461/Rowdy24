package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Software implementation of the Angler subsystem on the Rowdy24 robot. The function of the
 * angler subsystem is to raise or lower the angle of the shooter to aim for the speaker target
 * and prepare to shoot a note. The motor controller is a Rev NEO 550.
 */
public class Angler extends SubsystemBase {
    /** Represents the motor controller of the angler. */
    private final CANSparkMax angler;

    /** The motor's integrated encoder for motor data conversion. */
    private final RelativeEncoder encoder;

    /** The motor's integrated PID controller. */
    private final SparkPIDController anglerPIDController;

    /** Represents an external limit switch placed at the lower mechanical limit of the angler. */
    private final DigitalInput lowerLimitSwitch;

    /** The target position of the motor. */
    private double target;

    /**
     * The error of the angler, which is the difference between target
     * position and current position.
     */
    private double error;

    /**
     * The accuracy of the angler, which is the percentage similarity between
     * motor position and current position.
     */
    private double accuracy;

    /**
     * Constructs an angler instance.
     *
     * <p> CAN ID, current limit, inverted boolean, PID constants, and digital input channel
     * constants defined in {@link Constants} are used to initialize the angler motor controller,
     * set its current limit, inverted status, and PID controller, and configure the lower limit
     * switch, respectively. Additionally, the motor controller is set to factory defaults, and
     * the encoder is initialized. The target and error are initialized to 0.0,
     * and the accuracy is initialized to 1.0.
     */
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

        lowerLimitSwitch = new DigitalInput(Constants.Angler.ANGLER_LOWER_LIMIT_SWITCH);

        target = 0.0;
        error = 0.0;
        accuracy = 1.0;
    }

    /**
     * Runnable method that is run periodically as defined by {@link SubsystemBase}.
     *
     * <p> The angler's error and accuracy are updated in this periodic function.
     * The error is calculated as the difference between the target position and current position.
     * The accuracy is calculated as the quotient of the higher and the lower of the target position
     * and current position.
     */
    @Override
    public void periodic() {
        error = Math.abs(target - getPosition());
        accuracy = target > getPosition() ? getPosition() / target : target / getPosition();
    }

    /**
     * Get the position of the angler measured by rotations of the motor.
     *
     * @return Number of rotations of the motor.
     */
    public double getPosition() { 
        return encoder.getPosition();
    }

    /**
     * Get the angler's target position.
     *
     * @return Target position.
     * @unused This method is not used in actual implementation.
     */
    public double getTarget() {
        return target;
    }

    /**
     * Get the angler's motor applied output.
     *
     * @return Motor applied output.
     * @unused This method is not used in actual implementation.
     */
    public double anglerPower() {
        return angler.getAppliedOutput();
    }

    /**
     * Get the angler's error.
     *
     * @return Error.
     */
    public double getError() {
        return error;
    }

    /**
     * Get the state of the limit switch.
     *
     * @return Whether the limit switch is triggered.
     */
    public boolean lowerSwitchTriggered() {
        return !lowerLimitSwitch.get();
    }

    /**
     * Determine the accuracy of the angler in relation to the target. Determined if
     * the accuracy variable is greater than a constant defined in {@link Constants}.
     *
     * @return Whether the angler is near the target.
     */
    public boolean anglerNearTarget() {
        return accuracy > Constants.Angler.ANGLER_ACCURACY_REQUIREMENT;
    }

    /**
     * Calibrates the angler by resetting the encoded motor position to the lower bound
     * position defined in {@link Constants} if the limit switch is triggered.
     */
    public void checkLimitSwitches() {
       if (lowerSwitchTriggered()) {
           encoder.setPosition(Constants.Angler.ANGLER_LOWER_LIMIT);
       }
    }

    /**
     * Holds the angler at the target position of the angler using the PID controller. It
     * guarantees calibration by checking the limit switch.
     */
    public void holdTarget() {
        checkLimitSwitches();
        anglerPIDController.setReference(target, CANSparkBase.ControlType.kPosition);
    }

    /**
     * Moves the angler as a percentage of output and maintains position at the mechanical
     * limits using the PID controller. It guarantees calibration by checking the limit switch.
     *
     * @param axisValue Percentage output.
     */
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

    /**
     * Sets the target position of the angler and moves/holds the angler to/at the target position
     * using the PID controller. It guarantees calibration by checking the limit switch.
     *
     * @param encoderVal Target position bounded by mechanical limits.
     */
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