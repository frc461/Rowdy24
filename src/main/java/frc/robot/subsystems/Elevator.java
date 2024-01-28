package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax elevator;
    private final PIDController pidController = new PIDController(
            Constants.Elevator.ELEVATOR_P,
            Constants.Elevator.ELEVATOR_I,
            Constants.Elevator.ELEVATOR_D);
    private final RelativeEncoder encoder;
    private final DigitalInput elevatorSwitch;
    private double position;
    private double target;

    public Elevator() {
        elevator = new CANSparkMax(Constants.Elevator.ELEVATOR_ID, MotorType.kBrushless);
        encoder = elevator.getEncoder();
        position = encoder.getPosition();
        target = encoder.getPosition();
        elevatorSwitch = new DigitalInput(3); // limit switch that re-zeros the elevator encoder

        elevator.restoreFactoryDefaults();
        elevator.setSmartCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT);
        elevator.setInverted(Constants.Elevator.ELEVATOR_INVERT);
    }

    @Override
    public void periodic() {
        position = encoder.getPosition();
    }

    public double getPosition() {
        return position;
    }

    public double getTarget() {
        return target;
    }

    public double elevatorPower() {
        return elevator.getAppliedOutput();
    }

    public boolean elevatorSwitchTriggered() {
        return !elevatorSwitch.get();
    }

    public void checkLimitSwitches() {
        if (elevatorSwitchTriggered()) {
            encoder.setPosition(Constants.Elevator.ELEVATOR_LOWER_LIMIT);
        }
    }

    public void holdHeight() {
        elevator.set(pidController.calculate(position, target));
    }

    public void moveElevator(double axisValue) {
        target = position;
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdHeight();
            return;
        } else if (axisValue > 0 && position >= Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            target = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
            holdHeight();
            return;
        }
        elevator.set(axisValue);
    }

    public void setHeight(double height) {
        if (height < encoder.getPosition() && elevatorSwitchTriggered()) {
            encoder.setPosition(Constants.Elevator.ELEVATOR_LOWER_LIMIT);
            height = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
        } else if (height > encoder.getPosition() && encoder.getPosition() > Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            height = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
        }
        elevator.set(pidController.calculate(encoder.getPosition(), height));
        target = height;
    }
}
