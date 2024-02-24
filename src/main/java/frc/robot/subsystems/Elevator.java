package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private final PIDController upperPidController, lowerPidController;
    private final DigitalInput elevatorSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_LIMIT_SWITCH);
    public final Servo elevatorClamp = new Servo(Constants.Elevator.ELEVATOR_SERVO_PORT);
    
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
        Constants.Elevator.ELEVATOR_FF_KS, 
        Constants.Elevator.ELEVATOR_FF_KG,
        Constants.Elevator.ELEVATOR_FF_KV,
        Constants.Elevator.ELEVATOR_FF_KA
    );

    private double target;
    boolean clamped, limitHitOnce = false;

    public Elevator() {
        elevator =  new TalonFX(Constants.Elevator.ELEVATOR_ID);
        elevator.setNeutralMode(NeutralModeValue.Brake);
        var config = new TalonFXConfiguration();
        config.Voltage = new VoltageConfigs().withPeakForwardVoltage(6);
        config.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        config.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT);

        elevator.getConfigurator().apply(config);
        
        lowerPidController = new PIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D
                // new TrapezoidProfile.Constraints(Constants.Elevator.ELEVATOR_MAX_VELOCITY, Constants.Elevator.ELEVATOR_MAX_ACCEL)
        );

        
        upperPidController = new PIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D
                // new TrapezoidProfile.Constraints(Constants.Elevator.ELEVATOR_MAX_VELOCITY, Constants.Elevator.ELEVATOR_MAX_ACCEL)
        );

        elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);

        target = 0.0;
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
            limitHitOnce = true;
            elevator.setPosition(Constants.Elevator.ELEVATOR_LOWER_LIMIT);
        }
    }

    public void holdTarget() {
        checkLimitSwitch();
        if(!limitHitOnce) {
            elevator.set(0);
            return;
        }
        if (elevator.getPosition().getValueAsDouble() > Constants.Elevator.UPPER_STAGE_THRESHOLD) { //if on upper stage use higher PID
            double pid = upperPidController.calculate(getPosition(), target);
            double ff = 0; // elevatorFeedforward.calculate(upperPidController.getSetpoint().velocity);
            elevator.set(ff + pid);
        } else { //otherwise use lower PID
            double pid = lowerPidController.calculate(getPosition(), target);
            double ff = 0; // elevatorFeedforward.calculate(lowerPidController.getSetpoint().velocity);
            elevator.set(ff + pid);   
        }
    }

    public void climb(){
        if(!elevatorSwitchTriggered()){
            elevator.set(-0.2); //TODO this is sketch
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);
        }else{
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdTarget();
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS);
        }
    }

    public void setClamp() {
        if (!clamped) {
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS);
            clamped = true;
        } else {
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);
            clamped = false;
        }
    }
    public void moveElevator(double axisValue) {
        checkLimitSwitch();
        if (axisValue < 0 && elevatorSwitchTriggered()) {
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdTarget();
        } else if (axisValue > 0 && elevator.getPosition().getValueAsDouble() >= Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            target = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
            holdTarget();
        } else {
            elevator.set(axisValue);
            target = getPosition();
        }
    }

    public void setHeight(double height) {
        checkLimitSwitch();
        if (height < elevator.getPosition().getValueAsDouble() && elevatorSwitchTriggered()) {
            height = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
        } else if (height > elevator.getPosition().getValueAsDouble() && elevator.getPosition().getValueAsDouble() > Constants.Elevator.ELEVATOR_UPPER_LIMIT) {
            height = Constants.Elevator.ELEVATOR_UPPER_LIMIT;
        }
        target = height;
        holdTarget();
    }
}