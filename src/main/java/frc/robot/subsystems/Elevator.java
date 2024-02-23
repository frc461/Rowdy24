package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator;
    private final ProfiledPIDController pidController, upperPidController, lowerPidController;
    private final DigitalInput elevatorSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_LIMIT_SWITCH);
    private final Servo elevatorClamp = new Servo(Constants.Elevator.ELEVATOR_SERVO_PORT);
    private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(
        Constants.Elevator.ELEVATOR_FF_KS, 
        Constants.Elevator.ELEVATOR_FF_KG,
        Constants.Elevator.ELEVATOR_FF_KV,
        Constants.Elevator.ELEVATOR_FF_KA);
    private double target;
    boolean clamped, limitHitOnce = false;

    public Elevator() {
        elevator =  new TalonFX(Constants.Elevator.ELEVATOR_ID);
        elevator.setNeutralMode(NeutralModeValue.Brake);
        var config = new TalonFXConfiguration();
        config.Voltage = new VoltageConfigs().withPeakForwardVoltage(6);
        config.MotorOutput = new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive).withNeutralMode(NeutralModeValue.Brake);
        config.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT);
        TrapezoidProfile.Constraints trapConstraints = new TrapezoidProfile.Constraints(0.5,0.2);

        elevator.getConfigurator().apply(config);
        
        pidController = new ProfiledPIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D,
                trapConstraints
        );
        
        lowerPidController = new ProfiledPIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D,
                trapConstraints
        );

        
        upperPidController = new ProfiledPIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D,
                trapConstraints
        );

        elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);

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
            limitHitOnce = true;
            elevator.setPosition(Constants.Elevator.ELEVATOR_LOWER_LIMIT);
        }
    }

    public void holdTarget() {
        checkLimitSwitch();
        if(elevator.getRotorPosition().getValueAsDouble() > Constants.Elevator.UPPER_STAGE_THRESHOLD){ //if on upper stage use higher PID
            if (limitHitOnce) {
                double pid = upperPidController.calculate(elevator.getRotorPosition().getValueAsDouble(), target);
                double ff = elevatorFeedforward.calculate(upperPidController.getSetpoint().velocity);
            elevator.set(ff + pid);
            
            }
        }else{ //otherwise use lower PID
            if (limitHitOnce) {
                double pid = lowerPidController.calculate(elevator.getRotorPosition().getValueAsDouble(), target);
                double ff = elevatorFeedforward.calculate(lowerPidController.getSetpoint().velocity);
            elevator.set(ff + pid);
            }   
        }
    }

    public void climb(){
        if(!elevatorSwitchTriggered()){
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT-2;
            holdTarget();
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);
        }else{
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdTarget();
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS);
        }
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