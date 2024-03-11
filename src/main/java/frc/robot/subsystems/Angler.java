package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.*;

public class Angler extends SubsystemBase {
    private final CANSparkMax angler;
    private final RelativeEncoder encoder;
    private final SparkPIDController anglerPIDController;
    private final SparkLimitSwitch lowerMagnetLimitSwitch, upperMagnetLimitSwitch;
    private final DigitalInput lowerLimitSwitch;

    private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));

    /* Specified units are estimates because encoder "rotation" default unit position values are very inaccurate */
    private final MutableMeasure<Angle> angle = mutable(Degrees.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(DegreesPerSecond.of(0));
    private final SysIdRoutine sysIdRoutine;

    private double target, limelightTarget, error, accuracy;

    public Angler() {
        angler = new CANSparkMax(Constants.Angler.ANGLER_ID, MotorType.kBrushless);
        angler.restoreFactoryDefaults();
        angler.setSmartCurrentLimit(Constants.Angler.ANGLER_CURRENT_LIMIT);
        angler.setInverted(Constants.Angler.ANGLER_INVERT);
        encoder = angler.getEncoder();

        anglerPIDController = angler.getPIDController();
        anglerPIDController.setP(Constants.Angler.ANGLER_P);
        anglerPIDController.setI(Constants.Angler.ANGLER_I);
        anglerPIDController.setD(Constants.Angler.ANGLER_D);
        angler.burnFlash();

        lowerMagnetLimitSwitch = angler.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
        lowerLimitSwitch = new DigitalInput(Constants.Angler.ANGLER_LOWER_LIMIT_SWITCH);
        upperMagnetLimitSwitch = angler.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> angler.setVoltage(volts.in(Volts)),
                        log -> log.motor("angler")
                                .voltage(
                                        appliedVoltage.mut_replace(
                                                angler.get() * RobotController.getBatteryVoltage(), Volts
                                        ))
                                .angularPosition(
                                        angle.mut_replace(
                                                encoder.getPosition(), Degrees
                                        ))
                                .angularVelocity(
                                        velocity.mut_replace(
                                                encoder.getVelocity() / 60, DegreesPerSecond
                                        )),
                        this
                )
        );

        target = 0.0;
        limelightTarget = getPosition();
        error = 0.0;
        accuracy = 1.0;
    }

    @Override
    public void periodic() {
        updateLimelightTarget();
        error = Math.abs(target - getPosition());
        accuracy = Limelight.tagExists() ? (target > limelightTarget) ?
                limelightTarget / target : target / limelightTarget : -1.0;
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
        return !lowerLimitSwitch.get();
    }

    public void updateLimelightTarget() {
        limelightTarget = Limelight.tagExists() ?
                Math.min(
                        Constants.Angler.AUTO_ANGLER_AIM_EQUATION.apply(
                                Limelight.getRX(),
                                Limelight.getRZ()) + Constants.Angler.ANGLER_ENCODER_OFFSET,
                        Constants.Angler.ANGLER_UPPER_LIMIT
                ) : getPosition();
    }

    public void checkLimitSwitches() {
       if (lowerSwitchTriggered()) {
           encoder.setPosition(Constants.Angler.ANGLER_LOWER_LIMIT);
       }
    }

    public void holdTarget() {
        checkLimitSwitches();
        anglerPIDController.setReference(target, CANSparkBase.ControlType.kPosition);
    }

    public void moveAngle(double axisValue) {
        checkLimitSwitches();
        if (axisValue < 0 && lowerSwitchTriggered()) {
            target = Constants.Angler.ANGLER_LOWER_LIMIT;
            holdTarget();
        } else if (axisValue > 0 && getPosition() > Constants.Angler.ANGLER_UPPER_LIMIT) {
            target = Constants.Angler.ANGLER_UPPER_LIMIT;
            holdTarget();
        } else {
            angler.set(axisValue);
            target = encoder.getPosition();
        }
    }

    public void setEncoderVal(double encoderVal) {
        checkLimitSwitches();
        encoderVal = (encoderVal < encoder.getPosition() && lowerSwitchTriggered()) ?
                Constants.Angler.ANGLER_LOWER_LIMIT :
                (encoderVal > encoder.getPosition() && getPosition() > Constants.Angler.ANGLER_UPPER_LIMIT) ?
                Constants.Angler.ANGLER_UPPER_LIMIT : encoderVal;
        target = encoderVal;
        holdTarget();
    }

    public void setAlignedAngle() {
        updateLimelightTarget();
        setEncoderVal(limelightTarget);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }
}