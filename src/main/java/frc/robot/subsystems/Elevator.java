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
import frc.robot.AlphaBotConstants;
import frc.robot.Configuration;
import frc.robot.RobotConstants;
import frc.robot.RobotIdentity;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private final PIDController pidController;
    private final DigitalInput elevatorSwitch = new DigitalInput(AlphaBotConstants.Elevator.ELEVATOR_LIMIT_SWITCH);
    private final Servo elevatorClamp = new Servo(AlphaBotConstants.Elevator.ELEVATOR_SERVO_PORT);
    private double target;
    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    public Elevator() {
        elevator = new TalonFX(configuration.elevator_id);

        elevator.setNeutralMode(NeutralModeValue.Brake);
        var talonFXConfigurator = elevator.getConfigurator();
        talonFXConfigurator.apply(new VoltageConfigs().withPeakForwardVoltage(60));
        talonFXConfigurator.apply(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
        talonFXConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        talonFXConfigurator.apply(new CurrentLimitsConfigs().withSupplyCurrentLimit(configuration.elevator_current_limit));

        pidController = new PIDController(
                configuration.elevator_p,
                configuration.elevator_i,
                configuration.elevator_d
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
            elevator.setPosition(configuration.elevator_lower_limit);
        }
    }

    public void setClamp(boolean isClamped) {
        if (isClamped) {
            elevatorClamp.set(configuration.elevator_servo_clamped_pos);
        }
        else {
            elevatorClamp.set(configuration.elevator_servo_unclamped_pos);
        }
    }
    public void holdTarget() {
        checkLimitSwitch();
        elevator.set(pidController.calculate(elevator.getRotorPosition().getValueAsDouble(), target));
    }

    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = configuration.elevator_lower_limit;
            holdTarget();
        } else if (axisValue > 0 && elevator.getRotorPosition().getValueAsDouble() >= configuration.elevator_upper_limit) {
            target = configuration.elevator_upper_limit;
            holdTarget();
        } else {
            elevator.set(axisValue);
            target = elevator.getRotorPosition().getValueAsDouble();
        }
    }

    public void setHeight(double height) {
        checkLimitSwitch();
        if (height < elevator.getRotorPosition().getValueAsDouble() && elevatorSwitchTriggered()) {
            height = configuration.elevator_lower_limit;
        } else if (height > elevator.getRotorPosition().getValueAsDouble() && elevator.getRotorPosition().getValueAsDouble() > configuration.elevator_upper_limit) {
            height = configuration.elevator_upper_limit;
        }
        target = height;
        holdTarget();
    }
}