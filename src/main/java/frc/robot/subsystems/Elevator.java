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
    private final PIDController pidController;
    private final RelativeEncoder encoder;
    private final DigitalInput elevatorSwitch;
    private double target;

    public Elevator() {
        elevator = new CANSparkMax(Constants.Elevator.ELEVATOR_ID, MotorType.kBrushless);
        elevatorSwitch = new DigitalInput(4); // limit switch that re-zeros the elevator encoder

        elevator.restoreFactoryDefaults();
        elevator.setSmartCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT);
        elevator.setInverted(Constants.Elevator.ELEVATOR_INVERT);

        encoder = elevator.getEncoder();
        encoder.setPosition(Constants.Elevator.ELEVATOR_LOWER_LIMIT);

        while (!elevatorSwitchTriggered()) {
            elevator.set(-0.5);
        }

        pidController = new PIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D
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
        elevator.set(pidController.calculate(encoder.getPosition(), target));
    }

    public void moveElevator(double axisValue) {
        target = encoder.getPosition();
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdHeight();
            return;
        } else if (axisValue > 0 && encoder.getPosition() >= Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
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