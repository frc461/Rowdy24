package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final CANSparkMax elevator;
    private final PIDController pidController;
    private final RelativeEncoder encoder;
    private final DigitalInput elevatorSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_LIMIT_SWITCH);
    private final Servo elevatorClamp = new Servo(Constants.Elevator.ELEVATOR_SERVO_PORT);
    private double target;

    public Elevator() {
        elevator = new CANSparkMax(Constants.Elevator.ELEVATOR_ID, MotorType.kBrushless);
        elevator.restoreFactoryDefaults();
        elevator.setSmartCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT);
        elevator.setInverted(Constants.Elevator.ELEVATOR_INVERT);
        encoder = elevator.getEncoder();

        pidController = new PIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D
        );

        target = 0.0;
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

    public void checkLimitSwitch() {
        if (elevatorSwitchTriggered()) {
            encoder.setPosition(Constants.Elevator.ELEVATOR_LOWER_LIMIT);
        }
    }

    public void holdTarget() {
        checkLimitSwitch();
        elevator.set(pidController.calculate(encoder.getPosition(), target));
    }

    public void setClamp(boolean isClamped){
        if (isClamped) elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS); else elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdTarget();
        } else if (axisValue > 0 && encoder.getPosition() >= Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            target = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
            holdTarget();
        } else {
            elevator.set(axisValue);
            target = encoder.getPosition();
        }
    }

    public void setHeight(double height) {
        checkLimitSwitch();
        if (height < encoder.getPosition() && elevatorSwitchTriggered()) {
            height = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
        } else if (height > encoder.getPosition() && encoder.getPosition() > Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            height = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
        }
        target = height;
        holdTarget();
    }
}