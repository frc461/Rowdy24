package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private final TalonFX elevator, elevator2;
    private final Follower elevatorFollower;
    private final PIDController elevatorPIDController;
    private final DigitalInput elevatorSwitch;
    private final Servo elevatorClamp;
    private double target;
    private boolean clamped, movingAboveLimitSwitch;

    public Elevator() {
        elevator = new TalonFX(Constants.Elevator.ELEVATOR_ID);
        elevator.getConfigurator().apply(new TalonFXConfiguration()
                .withVoltage(new VoltageConfigs().withPeakForwardVoltage(6))
                .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(Constants.Elevator.ELEVATOR_INVERT)
                        .withNeutralMode(NeutralModeValue.Coast))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Constants.Elevator.ELEVATOR_CURRENT_LIMIT))
                .withAudio(new AudioConfigs().withBeepOnConfig(false)
                        .withBeepOnBoot(false)
                        .withAllowMusicDurDisable(true)));

        elevatorPIDController = new PIDController(
                Constants.Elevator.ELEVATOR_P,
                Constants.Elevator.ELEVATOR_I,
                Constants.Elevator.ELEVATOR_D
        );

        elevatorFollower = new Follower(Constants.Elevator.ELEVATOR_ID, true);

        elevator2 = new TalonFX(Constants.Elevator.ELEVATOR_FOLLOWER_ID);
        elevator2.setControl(elevatorFollower);

        elevatorSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_LIMIT_SWITCH);

        elevatorClamp = new Servo(Constants.Elevator.ELEVATOR_SERVO_PORT);
        elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);

        target = 0.0;
        clamped = false; // disables/enables clamp
        movingAboveLimitSwitch = false; // whether the elevator is trying to move above the limit switch
    }

   @Override
   public void periodic() {
       if (!clamped) {
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_UNCLAMPED_POS);
       } else {
            elevatorClamp.set(Constants.Elevator.ELEVATOR_SERVO_CLAMPED_POS);
       }
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
            if ((target > getPosition() ? getPosition() / target : target / getPosition()) > 0.95) {
                movingAboveLimitSwitch = false;
            }
        }
    }

    public void holdTarget() {
        checkLimitSwitch();
        elevator.set(elevatorSwitchTriggered() && !movingAboveLimitSwitch ? 0 : elevatorPIDController.calculate(getPosition(), target));
    }

    public void climb() {
        checkLimitSwitch();
        if (elevatorSwitchTriggered()) {
            target = Constants.Elevator.ELEVATOR_LOWER_LIMIT;
            holdTarget();
            setClamp(true);
        } else {
            elevator.set(-0.2); //TODO this is sketch
//            target = getPosition() - 0.05;
//            holdTarget();
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

    public double getClampPosition() {
        return elevatorClamp.getPosition();
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
        movingAboveLimitSwitch = true;
        holdTarget();
    }
}