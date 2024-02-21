package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.variants.PracticeConstants;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private final PIDController pidController;
    private final DigitalInput elevatorSwitch = new DigitalInput(PracticeConstants.Elevator.ELEVATOR_LIMIT_SWITCH);
    private final Servo elevatorClamp = new Servo(PracticeConstants.Elevator.ELEVATOR_SERVO_PORT);
    private double target;

    public Elevator() {
        elevator = new TalonFX(Constants.Elevator.ELEVATOR_ID);

        elevator.setNeutralMode(NeutralModeValue.Brake);
        var talonFXConfigurator = elevator.getConfigurator();
        talonFXConfigurator.apply(new VoltageConfigs().withPeakForwardVoltage(60));
        talonFXConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        talonFXConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        talonFXConfigurator.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT));

        pidController = new PIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D
        );

        target = 0.0;
    }

    public double getPosition() {
        return elevator.getRotorPosition().getValueAsDouble();
    }

    public double getTarget() {
        return target;
    }

    public double elevatorVelocity() {
        return elevator.getRotorVelocity().getValueAsDouble();
    }

    public boolean elevatorSwitchTriggered() {
        return !elevatorSwitch.get();
    }

    public void checkLimitSwitch() {
        if (elevatorSwitchTriggered()) {
            elevator.setPosition(Constants.Elevator.ELEVATOR_LOWER_LIMIT);
        }
    }

    public void setClamp(boolean isClamped) {
        if (isClamped) {
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS);
        }
        else {
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);
        }
    }
    public void holdTarget() {
        checkLimitSwitch();
        elevator.set(pidController.calculate(elevator.getRotorPosition().getValueAsDouble(), target));
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdTarget();
        } else if (axisValue > 0 && elevator.getRotorPosition().getValueAsDouble() >= Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            target = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
            holdTarget();
        } else {
            elevator.set(axisValue);
            target = elevator.getRotorPosition().getValueAsDouble();
        }
    }

    public void setHeight(double height) {
        checkLimitSwitch();
        if (height < elevator.getRotorPosition().getValueAsDouble() && elevatorSwitchTriggered()) {
            height = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
        } else if (height > elevator.getRotorPosition().getValueAsDouble() && elevator.getRotorPosition().getValueAsDouble() > Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            height = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
        }
        target = height;
        holdTarget();
    }
}