package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Elevator extends SubsystemBase{
    private final CANSparkMax elevator = new CANSparkMax(31, MotorType.kBrushless);
    private final PIDController pidController = new PIDController(Constants.ELEVATOR_P, Constants.ELEVATOR_I, Constants.ELEVATOR_D);
    private final RelativeEncoder encoder = elevator.getEncoder();
    private final DigitalInput elevatorSwitch = new DigitalInput(3); //limit switch that re-zeros the elevator encoder;
    private double position;
    private double target;

    public Elevator() {
        position = encoder.getPosition();
        target = encoder.getPosition();

        elevator.restoreFactoryDefaults();
        elevator.setSmartCurrentLimit(70);
        elevator.setInverted(true);
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
            encoder.setPosition(0);
        }
    }

    public void holdHeight() {
        elevator.set(pidController.calculate(position, target));
    }

    public void moveElevator(double axisValue) {
        target = position;
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = 0;
            holdHeight();
            return;
        }
        else if (axisValue > 0 && position >= Constants.ELEVATOR_UPPER_LIMIT) {
            target = Constants.ELEVATOR_UPPER_LIMIT;
            holdHeight();
            return;
        }
        elevator.set(axisValue);
    }
}
