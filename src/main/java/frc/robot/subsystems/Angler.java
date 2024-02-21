package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Configuration;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.RobotIdentity;
public class Angler extends SubsystemBase {
    private final CANSparkMax angler;
    private final PIDController pidController;
    private final RelativeEncoder encoder;
    private double target;
    private double error;
    private final RobotConstants robot = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    private final Configuration configuration = robot.getConfiguration();

    public Angler() {
        angler = new CANSparkMax(configuration.angler_id, MotorType.kBrushless);
        angler.restoreFactoryDefaults();
        angler.setSmartCurrentLimit(configuration.angler_current_limit);
        angler.setInverted(configuration.angler_invert);
        encoder = angler.getEncoder();
        
        pidController = new PIDController(
                configuration.angler_p,
                configuration.angler_i,
                configuration.angler_d
        );

        target = 0.0;
        error = 1000;
    }

    @Override
    public void periodic() {
        error = Math.abs(target - getPosition());
    }

    public double getPosition() { 
        return encoder.getPosition();
    }

    public double getTarget() {
        return target;
    }

    public double anglerPower() {
        return angler.getAppliedOutput();
    }

    public double getError() {
        return error;
    }

    public boolean lowerSwitchTriggered() { 
        return angler.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public boolean upperSwitchTriggered() {
        return angler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed();
    }

    public void checkLimitSwitches() {
        if (upperSwitchTriggered()) {
            encoder.setPosition(configuration.angler_upper_limit);
        }
        if (lowerSwitchTriggered()) {
            encoder.setPosition(configuration.angler_lower_limit);
        }
    }

    public void holdTarget() {
        checkLimitSwitches();
        angler.set(pidController.calculate(encoder.getPosition(), target));
    }

    public void moveAngle(double axisValue) {
        target = encoder.getPosition();
        checkLimitSwitches();
        if (axisValue < 0 && lowerSwitchTriggered()) {
            target = configuration.angler_lower_limit;
            holdTarget();
        } else if (axisValue > 0 && upperSwitchTriggered()) {
            target = configuration.angler_upper_limit;
            holdTarget();
        } else {
            angler.set(axisValue);
        }
    }

    public void setAngle(double encoderVal) {
        checkLimitSwitches();
        encoderVal = (encoderVal < encoder.getPosition() && lowerSwitchTriggered()) ?
                configuration.angler_lower_limit :
                (encoderVal > encoder.getPosition() && upperSwitchTriggered()) ?
                configuration.angler_upper_limit : encoderVal;
        target = encoderVal;
        holdTarget();
    }

    public void setAlignedAngle(double x, double z, boolean tag) {
        double dist = Math.hypot(x, z);
        if (tag) {
            if (dist < configuration.upper_bound_limit) {
                setAngle(Math.min(
                        configuration.tight_bound_coefficient *
                                Math.pow(dist, configuration.tight_bound_series) - 0.3, configuration.angler_upper_limit
                ));
            } else {
                setAngle(Math.min(
                        configuration.upper_bound_coefficient *
                                Math.pow(dist, configuration.upper_bound_series), configuration.angler_upper_limit
                ));
            }
        } else {
            target = getPosition();
        }
    }
}