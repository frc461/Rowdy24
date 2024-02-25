package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private final PIDController elevatorPIDController;
    private final DigitalInput elevatorSwitch;
    public final Servo elevatorClamp;
    private double target;
    private boolean clamped;

    public Elevator() {
        elevator = new TalonFX(Constants.Elevator.ELEVATOR_ID);
        elevator.setNeutralMode(NeutralModeValue.Brake);
        elevator.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(NeutralModeValue.Brake))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT)));

        elevatorPIDController = new PIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D
        );

        elevatorSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_LIMIT_SWITCH);

        elevatorClamp = new Servo(Constants.Elevator.ELEVATOR_SERVO_PORT);
        elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);

        target = 0.0;
        clamped = false; // disables/enables clamp
    }

    public double getPosition() {
        return elevator.getPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
    }

    public double elevatorVelocity() {
        return elevator.getVelocity().getValueAsDouble();
    }

    public boolean elevatorSwitchTriggered() {
        return !elevatorSwitch.get();
    }

    public void checkLimitSwitch() {
        if (elevatorSwitchTriggered()) {
            elevator.setPosition(Constants.Elevator.ELEVATOR_LOWER_LIMIT);
        }
    }

    public void holdTarget() {
        checkLimitSwitch();
        elevator.set(elevatorPIDController.calculate(getPosition(), target));
    }

    public void climb() {
        checkLimitSwitch();
        if (elevatorSwitchTriggered()) {
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdTarget();
            setClamp(true);
        } else {
            elevator.set(-0.2); //TODO this is sketch
        }
    }

    // boolean toggles for specific subsystems are meant to be in the subsystem class, not robot container
    public void toggleClamp() {
        setClamp(!clamped);
    }

    public void setClamp(boolean toggle) {
        clamped = toggle;
        elevatorClamp.set(clamped ?
                Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS :
                Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS
        );
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdTarget();
        } else if (axisValue > 0 && getPosition() >= Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            target = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
            holdTarget();
        } else {
            elevator.set(axisValue);
            target = getPosition();
        }
    }

    public void setHeight(double height) {
        checkLimitSwitch();
        height = Math.max(Constants.Elevator.ELEVATOR_LOWER_LIMIT, height);
        height = Math.min(Constants.Elevator.ELEVATOR_UPPER_LIMIT, height);
        target = height;
        holdTarget();
    }
}